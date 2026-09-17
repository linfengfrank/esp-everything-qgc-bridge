/* On-demand camera preview.
 *
 * While camera_stream.py sends keepalives, this task grabs camera frames,
 * JPEG-encodes a copy (the AprilTag detector needs the sensor in grayscale
 * mode, so the sensor cannot encode) and sends it to the laptop over UDP.
 *
 * Each JPEG goes out as data datagrams (header + up to 1400 bytes; 1430 <
 * 1472, so IP never fragments) in offset order, then one END datagram with
 * no payload.  A frame whose send fails gets no END, so the viewer drops it.
 */
#include "camera_stream.h"
#include "wifi_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "sdkconfig.h"

#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *TAG = "cam_stream";

#define CHUNK_BYTES        1400
#define MAX_PIXELS         (320 * 240)
#define MAX_FPS            15
#define MIN_QUALITY        10
#define MAX_QUALITY        90
#define POLL_MS            100
#define SEND_RETRIES       5                 /* 2 ms apart, for full WiFi TX buffers */
#define MAX_BACKOFF        10
#define MAX_PERIOD_US      (1000 * 1000)     /* backoff limit: 1 frame/s */
#define MIN_INTERNAL_FREE  (24 * 1024)       /* WiFi/lwIP buffers need internal RAM */
#define WARN_PERIOD_US     (5 * 1000 * 1000)
#define FLAG_END           0x02

/* Wire header, little-endian; must match laptop/protocol.py. */
typedef struct __attribute__((packed)) {
    uint8_t  magic[4];     /* "ECAM" */
    uint8_t  version;      /* 2 */
    uint8_t  flags;        /* FLAG_END */
    uint8_t  drone_id;
    uint8_t  boot_nonce;   /* random per boot, so the viewer notices a reboot */
    uint32_t frame_id;     /* +1 per frame */
    uint32_t offset;       /* payload offset in the JPEG; END: == frame_size */
    uint32_t frame_size;   /* END only */
    uint16_t width;
    uint16_t height;
    uint16_t payload_len;
    uint16_t age_ms;       /* capture -> this datagram sent */
    uint16_t esp_drops;    /* frames dropped since boot (wraps) */
} header_t;

_Static_assert(sizeof(header_t) == 30, "must match laptop/protocol.py");

typedef struct {
    header_t hdr;
    int64_t  capture_us;
    uint16_t fill;         /* payload bytes waiting in s_packet */
    bool     failed;       /* a send failed: send nothing more */
    int      err;          /* errno of that failure */
} frame_t;

static int      s_sock = -1;
static uint8_t *s_image;               /* PSRAM copy of the frame being encoded */
static uint8_t *s_packet;              /* PSRAM header + payload being sent */
static uint8_t  s_boot_nonce;
static uint16_t s_esp_drops;
static volatile bool s_active;

/* ---------------------------------------------------------------------------
 * Fresh frame fetch
 *
 * With fb_count = 2 the driver keeps capturing while a buffer is free, and a
 * newer frame replaces the queued one.  But a frame queued while the other
 * buffer is held (by the detector for a whole detection when the preview is
 * off) just waits and goes stale, and the detector pairs its image with the
 * drone pose read after detection.
 *
 * A frame the driver completed while esp_camera_fb_get() blocked is fresh;
 * its age (sensor dependent, ~90 ms on the OV3660) is learnt.  A queued frame
 * older than that + 25 ms, or any queued frame before the first sample, is
 * handed back and the next one taken instead (one retry).
 * --------------------------------------------------------------------------- */
#define FRESH_WAIT_US    3000
#define STALE_MARGIN_US  25000

static volatile int32_t s_fresh_age_us;   /* 0 = no sample; one word, so atomic */

static int64_t capture_us(const camera_fb_t *pic)
{
    return (int64_t)pic->timestamp.tv_sec * 1000000 + pic->timestamp.tv_usec;
}

camera_fb_t *camera_fb_get_fresh(void)
{
    for (int attempt = 0; ; attempt++) {
        int64_t start = esp_timer_get_time();
        camera_fb_t *pic = esp_camera_fb_get();
        int64_t now = esp_timer_get_time();
        if (pic == NULL) return NULL;

        int64_t age = now - capture_us(pic);
        int32_t fresh = s_fresh_age_us;
        if (now - start >= FRESH_WAIT_US) {             /* waited: fresh */
            if (age > 0 && age < 1000000) {
                s_fresh_age_us = fresh ? fresh + ((int32_t)age - fresh) / 4
                                       : (int32_t)age;
            }
            return pic;
        }
        if (attempt > 0 || (fresh > 0 && age <= fresh + STALE_MARGIN_US)) {
            return pic;
        }
        esp_camera_fb_return(pic);                      /* stale: take the next */
    }
}

bool camera_stream_active(void)
{
    return s_active;
}

/* ---------------------------------------------------------------------------
 * Sending
 * --------------------------------------------------------------------------- */
static void send_datagram(frame_t *f, uint8_t flags, uint16_t payload_len)
{
    if (f->failed) return;
    f->hdr.flags = flags;
    f->hdr.payload_len = payload_len;
    const size_t len = sizeof(header_t) + payload_len;

    for (int attempt = 0; ; attempt++) {
        int64_t age = (esp_timer_get_time() - f->capture_us) / 1000;
        f->hdr.age_ms = (uint16_t)MIN(MAX(age, 0), UINT16_MAX);
        memcpy(s_packet, &f->hdr, sizeof(header_t));
        ssize_t ret = send(s_sock, s_packet, len, 0);
        if (ret == (ssize_t)len) return;

        bool buffers_full = ret < 0
            && (errno == ENOMEM || errno == ENOBUFS || errno == EAGAIN);
        if (!buffers_full || attempt == SEND_RETRIES) {
            f->failed = true;
            f->err = (ret < 0) ? errno : 0;
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void flush(frame_t *f)
{
    if (f->fill == 0) return;
    send_datagram(f, 0, f->fill);
    f->hdr.offset += f->fill;
    f->fill = 0;
}

/* jpge output callback: pack its 512-byte pieces into full payloads. */
static size_t jpeg_out(void *arg, size_t index, const void *data, size_t len)
{
    (void)index;
    frame_t *f = arg;
    const uint8_t *src = data;
    for (size_t left = len; left > 0; ) {
        size_t n = MIN(left, (size_t)(CHUNK_BYTES - f->fill));
        memcpy(s_packet + sizeof(header_t) + f->fill, src, n);
        f->fill += n;
        src += n;
        left -= n;
        if (f->fill == CHUNK_BYTES) flush(f);
    }
    return len;
}

/* ---------------------------------------------------------------------------
 * Task
 * --------------------------------------------------------------------------- */
static int64_t period_us(int fps, int backoff)
{
    return MIN((1000000LL / fps) << backoff, MAX_PERIOD_US);
}

static void camera_stream_task(void *arg)
{
    (void)arg;
    uint32_t frame_id = 0;
    int64_t next_due = 0;
    int64_t last_start = INT64_MIN / 2;
    int64_t last_warn = INT64_MIN / 2;
    int backoff = 0;   /* failed frames in a row: each doubles the period */

    for (;;) {
        wifi_camera_stream_req_t req;
        if (!wifi_camera_stream_get(&req)) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }
        int fps = req.max_fps ? MIN(req.max_fps, MAX_FPS)
                              : CONFIG_CAMERA_STREAM_DEFAULT_FPS;
        int quality = req.quality ? MIN(MAX(req.quality, MIN_QUALITY), MAX_QUALITY)
                                  : CONFIG_CAMERA_STREAM_DEFAULT_QUALITY;
        int64_t period = period_us(fps, backoff);

        /* Pace.  A raised rate or an ended backoff applies at once; a late
         * frame restarts the schedule instead of bursting to catch up. */
        int64_t now = esp_timer_get_time();
        next_due = MIN(next_due, last_start + period);
        if (now < next_due) {
            int64_t wait_ms = MIN((next_due - now + 999) / 1000, POLL_MS);
            vTaskDelay(MAX(pdMS_TO_TICKS(wait_ms), 1));
            continue;
        }
        next_due += period;
        if (next_due < now + period / 2) next_due = now + period;
        last_start = now;

        frame_t f = {0};
        bool low_ram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < MIN_INTERNAL_FREE;
        bool sent = false;
        if (!low_ram) {
            camera_fb_t *pic = camera_fb_get_fresh();
            if (pic == NULL) {
                vTaskDelay(pdMS_TO_TICKS(POLL_MS));
                continue;
            }
            size_t pixels = (size_t)pic->width * pic->height;
            if (pic->format != PIXFORMAT_GRAYSCALE || pixels > MAX_PIXELS
                    || pic->len < pixels) {
                esp_camera_fb_return(pic);
                continue;
            }
            f.capture_us = capture_us(pic);
            f.hdr.width  = pic->width;
            f.hdr.height = pic->height;
            memcpy(s_image, pic->buf, pixels);
            esp_camera_fb_return(pic);        /* the detector needs it back */

            memcpy(f.hdr.magic, "ECAM", 4);
            f.hdr.version    = 2;
            f.hdr.drone_id   = CONFIG_DRONE_ID;
            f.hdr.boot_nonce = s_boot_nonce;
            f.hdr.frame_id   = ++frame_id;
            f.hdr.esp_drops  = s_esp_drops;
            if (fmt2jpg_cb(s_image, pixels, f.hdr.width, f.hdr.height,
                           PIXFORMAT_GRAYSCALE, quality, jpeg_out, &f)
                    && f.hdr.offset + f.fill > 0) {
                flush(&f);
                f.hdr.frame_size = f.hdr.offset;
                send_datagram(&f, FLAG_END, 0);
                sent = !f.failed;
            }
        }
        if (sent) {
            backoff = 0;
            continue;
        }

        s_esp_drops++;
        if (low_ram || f.failed) {            /* link or RAM short: back off */
            backoff = MIN(backoff + 1, MAX_BACKOFF);
            next_due = now + period_us(fps, backoff);
        }
        if (now - last_warn >= WARN_PERIOD_US) {
            last_warn = now;
            ESP_LOGW(TAG, "Preview frame dropped: %s (errno %d)",
                     low_ram ? "low internal RAM"
                     : f.failed ? "send failed" : "JPEG encode failed", f.err);
        }
    }
}

void camera_stream_start(void)
{
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_CAMERA_STREAM_PORT),
    };
    s_boot_nonce = esp_random() % 255 + 1;   /* WiFi is up: true RNG */
    s_image  = heap_caps_malloc(MAX_PIXELS, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_packet = heap_caps_malloc(sizeof(header_t) + CHUNK_BYTES,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_sock   = socket(AF_INET, SOCK_DGRAM, 0);

    /* The stack is in PSRAM too (internal RAM is tight), so this task must
     * never do flash operations (NVS, spi_flash). */
    if (s_image == NULL || s_packet == NULL || s_sock < 0
            || inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr) == 0
            || connect(s_sock, (struct sockaddr *)&dest, sizeof(dest)) < 0
            || xTaskCreatePinnedToCoreWithCaps(
                   camera_stream_task, "cam_stream", CAMERA_STREAM_TASK_STACK,
                   NULL, CAMERA_STREAM_TASK_PRIORITY, NULL,
                   CAMERA_STREAM_TASK_CORE,
                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGW(TAG, "Camera preview disabled: setup failed (errno %d)", errno);
        if (s_sock >= 0) close(s_sock);
        free(s_image);
        free(s_packet);
        return;
    }
    s_active = true;
    ESP_LOGI(TAG, "Camera preview ready -> %s:%d; internal heap %u B free",
             CONFIG_HOST_IPV4_ADDR, WIFI_CAMERA_STREAM_PORT,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}
