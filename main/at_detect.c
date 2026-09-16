#include <esp_log.h>
#include <esp_system.h>
#include "esp_timer.h"
#include <nvs_flash.h>
#include <stdio.h>
#include <sys/param.h>
#include <string.h>

// Apriltag dependencies
#include "apriltag.h"
#include "common/image_types.h"
#include "common/zarray.h"
#include "common/image_u8.h"
#include "lwip/sockets.h"
#include "sensor.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"

// Apriltag Pose Estimation dependencies
#include "apriltag_pose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "at_detect.h"
#include "odom.h"
#include "mavlink_task.h"

#include "freertos/semphr.h"

#include <math.h>

static at_detect_pose_t  s_pose;
static SemaphoreHandle_t s_pose_mutex;

static at_live_dets_t    s_live;
static SemaphoreHandle_t s_live_mutex;

at_detect_pose_t at_detect_get_pose(void)
{
    xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
    at_detect_pose_t copy = s_pose;
    xSemaphoreGive(s_pose_mutex);
    return copy;
}

at_live_dets_t at_detect_get_live(void)
{
    xSemaphoreTake(s_live_mutex, portMAX_DELAY);
    at_live_dets_t copy = s_live;
    xSemaphoreGive(s_live_mutex);
    return copy;
}
/* Camera is pitched 45° nose-down, 2 cm forward of drone centre.
 *
 * apriltag_pose camera frame (OpenCV): X=right, Y=down, Z=forward
 * Body frame:                          X=fwd,   Y=rgt,  Z=down
 *
 * The camera image is vertically flipped relative to the standard
 * OpenCV convention (the sensor is mounted with Y inverted), so ty
 * from estimate_tag_pose() has the opposite sign to the geometric
 * derivation.  The corrected body-frame transform is therefore:
 *
 *   body_x =  cos45·tz + sin45·ty   (forward)  ← note + not −
 *   body_y =  tx                     (right)
 *   body_z =  sin45·tz − cos45·ty   (down — unused)
 *
 * Mount offset: camera is CAM_FWD_OFFSET_M ahead of CoM. */
#define CAM_PITCH_DEG       45.0f
#define CAM_FWD_OFFSET_M    0.02f

void camera_to_ned(float tx, float ty, float tz,
                           float heading,
                           float *out_dn,      /* NED north offset to tag (m) */
                           float *out_de,      /* NED east  offset to tag (m) */
                           float *out_hdist)   /* horizontal distance to tag  */
{
    const float pitch = CAM_PITCH_DEG * (float)M_PI / 180.0f;
    const float cp    = cosf(pitch);   /* cos 45° = 0.7071 */
    const float sp    = sinf(pitch);   /* sin 45° = 0.7071 */

    /* Camera frame → body frame (ty sign inverted due to flipped image) */
    float body_x = cp * tz + sp * ty + CAM_FWD_OFFSET_M;   /* forward */
    float body_y = tx;                                        /* right   */
    /* float body_z = sp * tz - cp * ty; */   /* down — not needed yet */

    /* Horizontal distance from drone centre to tag (frame-invariant) */
    *out_hdist = sqrtf(body_x * body_x + body_y * body_y);

    /* Body frame → NED world frame */
    *out_dn = body_x * cosf(heading) - body_y * sinf(heading);
    *out_de = body_x * sinf(heading) + body_y * cosf(heading);
}
// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include "img_converters.h"
#include "wifi_task.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>

#define CAMERA_MODEL_XIAO_ESP32S3

#ifdef CAMERA_MODEL_XIAO_ESP32S3
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
#endif

static const char *TAG = "apriltag_detect";

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    // .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    // Initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, -2); // lower the saturation
    } else {
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, -2); // lower the saturation
    }
    s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
    return ESP_OK;
}
#endif

/* Yield between frames so the idle task (watchdog feed) still runs.  Kept
 * short: detection time dominates the loop period, and this delay adds
 * directly to tag-detection latency. */
#define LOOP_DELAY_MS 20

/* -------------------------------------------------------------------------
 * On-demand camera preview
 *
 * The detector needs the native QVGA grayscale frame, so the sensor cannot
 * simply be switched to JPEG mode.  Once detection is complete, frame2jpg_cb
 * software-encodes that same frame and the callback sends MTU-safe UDP
 * chunks.  camera_stream.py reassembles them.  No work is done unless the
 * viewer's keepalive has enabled the stream.
 * ------------------------------------------------------------------------- */
#define CAMERA_STREAM_MAGIC          "ECAM"
#define CAMERA_STREAM_VERSION        1
#define CAMERA_STREAM_FLAG_START     0x01
#define CAMERA_STREAM_FLAG_END       0x02
#define CAMERA_STREAM_CHUNK_BYTES    1200
#define CAMERA_STREAM_JPEG_QUALITY   60
#define CAMERA_STREAM_INTERVAL_MS    500u   /* at most 2 preview fps */

typedef struct __attribute__((packed)) {
    uint8_t  magic[4];
    uint8_t  version;
    uint8_t  flags;
    uint8_t  drone_id;
    uint8_t  reserved;
    uint32_t frame_id;
    uint32_t offset;
    uint32_t frame_size;  /* zero for data chunks; total bytes in END packet */
    uint16_t width;
    uint16_t height;
    uint16_t payload_len;
} camera_stream_header_t;

typedef struct {
    int       sock;
    uint8_t  *packet;
    uint32_t  frame_id;
    uint32_t  total_bytes;
    uint32_t  last_frame_ms;
    uint16_t  width;
    uint16_t  height;
} camera_stream_ctx_t;

static void camera_stream_send_packet(camera_stream_ctx_t *ctx,
                                      uint8_t flags, uint32_t offset,
                                      uint32_t frame_size,
                                      const uint8_t *payload,
                                      uint16_t payload_len)
{
    if (ctx->sock < 0 || ctx->packet == NULL) return;

    camera_stream_header_t hdr = {};
    memcpy(hdr.magic, CAMERA_STREAM_MAGIC, sizeof(hdr.magic));
    hdr.version     = CAMERA_STREAM_VERSION;
    hdr.flags       = flags;
    hdr.drone_id    = CONFIG_DRONE_ID;
    hdr.frame_id    = ctx->frame_id;
    hdr.offset      = offset;
    hdr.frame_size  = frame_size;
    hdr.width       = ctx->width;
    hdr.height      = ctx->height;
    hdr.payload_len = payload_len;

    memcpy(ctx->packet, &hdr, sizeof(hdr));
    if (payload_len > 0) {
        memcpy(ctx->packet + sizeof(hdr), payload, payload_len);
    }

    /* The socket is non-blocking.  Losing a chunk drops only this preview
     * frame; flight-control and detector tasks must never wait for video. */
    send(ctx->sock, ctx->packet, sizeof(hdr) + payload_len, 0);
}

static size_t camera_stream_jpeg_cb(void *arg, size_t index,
                                   const void *data, size_t len)
{
    camera_stream_ctx_t *ctx = (camera_stream_ctx_t *)arg;
    const uint8_t *src = (const uint8_t *)data;
    size_t sent = 0;

    while (sent < len) {
        size_t remaining = len - sent;
        uint16_t chunk_len = (uint16_t)(remaining > CAMERA_STREAM_CHUNK_BYTES
                                       ? CAMERA_STREAM_CHUNK_BYTES : remaining);
        uint8_t flags = (index + sent == 0) ? CAMERA_STREAM_FLAG_START : 0;
        camera_stream_send_packet(ctx, flags, (uint32_t)(index + sent), 0,
                                  src + sent, chunk_len);
        sent += chunk_len;
    }
    ctx->total_bytes = (uint32_t)(index + len);

    /* Tell the encoder all bytes were consumed even if UDP dropped one. */
    return len;
}

static void camera_stream_init(camera_stream_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->sock = -1;
    ctx->packet = malloc(sizeof(camera_stream_header_t)
                         + CAMERA_STREAM_CHUNK_BYTES);
    if (ctx->packet == NULL) {
        ESP_LOGW(TAG, "Camera preview packet allocation failed");
        return;
    }

    ctx->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->sock < 0) {
        ESP_LOGW(TAG, "Camera preview socket failed: errno %d", errno);
        free(ctx->packet);
        ctx->packet = NULL;
        return;
    }

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_CAMERA_STREAM_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    connect(ctx->sock, (struct sockaddr *)&dest, sizeof(dest));

    int flags = fcntl(ctx->sock, F_GETFL, 0);
    if (flags >= 0) fcntl(ctx->sock, F_SETFL, flags | O_NONBLOCK);
}

static void camera_stream_frame(camera_stream_ctx_t *ctx, camera_fb_t *pic)
{
    if (!wifi_camera_stream_enabled() || ctx->sock < 0) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if ((uint32_t)(now_ms - ctx->last_frame_ms) < CAMERA_STREAM_INTERVAL_MS)
        return;

    ctx->last_frame_ms = now_ms;
    ctx->frame_id++;
    ctx->total_bytes = 0;
    ctx->width  = (uint16_t)pic->width;
    ctx->height = (uint16_t)pic->height;

    if (!frame2jpg_cb(pic, CAMERA_STREAM_JPEG_QUALITY,
                      camera_stream_jpeg_cb, ctx)) {
        ESP_LOGW(TAG, "Camera preview JPEG encoding failed");
        return;
    }

    camera_stream_send_packet(ctx, CAMERA_STREAM_FLAG_END,
                              ctx->total_bytes, ctx->total_bytes,
                              NULL, 0);
}

static volatile bool  s_land_requested = false;
static volatile int   s_last_tag_id = -1;
static volatile int8_t s_my_tag_id  = -1;   /* latched: set once, never overwritten */

static int8_t            s_known_tags[AT_MAX_KNOWN_TAGS];
static int               s_known_count = 0;
static SemaphoreHandle_t s_known_mutex;

/* Is this tag ID already found by the fleet? */
static bool tag_is_known(int id)
{
    bool found = false;
    xSemaphoreTake(s_known_mutex, portMAX_DELAY);
    for (int i = 0; i < s_known_count; i++) {
        if (s_known_tags[i] == (int8_t)id) { found = true; break; }
    }
    xSemaphoreGive(s_known_mutex);
    return found;
}

void at_detect_init(void)
{
    s_land_requested = false;
    s_last_tag_id    = -1;
    s_my_tag_id      = -1;
    s_known_count    = 0;
    memset(&s_pose, 0, sizeof(s_pose));
    memset(&s_live, 0, sizeof(s_live));
    s_pose_mutex  = xSemaphoreCreateMutex();
    s_known_mutex = xSemaphoreCreateMutex();
    s_live_mutex  = xSemaphoreCreateMutex();
    configASSERT(s_pose_mutex != NULL);
    configASSERT(s_known_mutex != NULL);
    configASSERT(s_live_mutex != NULL);
}

bool at_detect_land_requested(void)
{
  return s_land_requested;
}

void at_detect_clear_land_request(void)
{
  s_land_requested = false;
}

void at_detect_reset_latch(void)
{
    s_my_tag_id      = -1;
    s_land_requested = false;
    xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
    s_pose.valid = false;
    xSemaphoreGive(s_pose_mutex);
}

int at_detect_last_id(void)
{
  return s_last_tag_id;
}

int8_t at_detect_my_tag_id(void)
{
    return s_my_tag_id;
}

void at_detect_set_known_tags(const int8_t *ids, int count)
{
    xSemaphoreTake(s_known_mutex, portMAX_DELAY);
    s_known_count = 0;
    for (int i = 0; i < count && s_known_count < AT_MAX_KNOWN_TAGS; i++) {
        if (ids[i] >= 0) s_known_tags[s_known_count++] = ids[i];
    }
    xSemaphoreGive(s_known_mutex);
}

void print_img(image_u8_t* im) {
  for (int i = 0; i < im->height; i++) {
    for (int j = 0; j < im->width; j++)
      printf("%d ", im->buf[i*(im->stride) + j]);
    printf("\n");
  }
}

int avg_img(image_u8_t* im) {
  int sum = 0;
  int len = (im->width) * (im->height);
  for (int i = 0; i < im->height; i++) {
    for (int j = 0; j < im->width; j++)
      sum += (im->buf[i*(im->stride) + j]);
  }

  sum /= len;
  return sum;
}
// ESP32 Cam Parameters!
// Tag Size in meters?
#define TAG_SIZE 0.12
#define F_X 163.5047216
#define F_Y 153.22210511
#define C_X 154.00573087
#define C_Y 107.10222796

void at_detect_task(void* pvParams)
{
#if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }
    camera_stream_ctx_t stream;
    camera_stream_init(&stream);

    // Create tag family object
    apriltag_family_t *tf = tag16h5_create();

    // Create AprilTag detector object
    apriltag_detector_t *td = apriltag_detector_create();

    // Add tag family to the detector
    apriltag_detector_add_family(td, tf);

    // Tag detector configs
    // quad_sigma is Gaussian blur's sigma
    // quad_decimate: quad search runs on the image downscaled by this factor —
    //                BIGGER = faster but far/small tags are lost
    //                (payload decoding still runs at full resolution)
    // With quad_sigma = 1.0 and quad_decimate = 4.0, ESP32-CAM can detect 16h5 tag
    // from the distance of about 1 meter (tested with tag on screen. not on paper)
    // 2.0 roughly halves per-frame latency vs 1.5 (watch the ms/frame figure in
    // tag_debug.py); drop back to 1.5 if tags beyond ~1 m stop being detected.
    td->quad_sigma = 1.0;
    td->quad_decimate = 1.5;//1.5;//6.0;//5.0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.75;
    td->nthreads = 1;
    td->debug = 0;

    while (1)
    {
        camera_fb_t *pic = esp_camera_fb_get();
      if (pic == NULL) {
        ESP_LOGW(TAG, "Camera frame grab failed");
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        continue;
      }

        // ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes (h=%zu, w=%zu, len=%zu)", pic->len, pic->height, pic->width, pic->len);

        image_u8_t at_im = {
          .width  = pic->width,
          .height = pic->height,
          .stride = pic->width,
          .buf    = pic->buf
        };

        // Testing responsiveness of camera
        // print_img(&at_im);
        // ESP_LOGI(TAG, "avg_img=%d", avg_img(&at_im));

        uint32_t proc_start_ms = (uint32_t)(esp_timer_get_time() / 1000);

        zarray_t *at_detections = apriltag_detector_detect(td, &at_im);

        at_live_dets_t live = {0};

        for (int i = 0; i < zarray_size(at_detections); i++) {
          apriltag_detection_t *det;
          zarray_get(at_detections, i, &det);

          /* Record every raw detection for the live debug stream, before
           * the nav-tag / known-tag / latch filters below drop it.  Pose is
           * only computed for detections passing the same quality gate the
           * filters use, so pose_err < 0 marks a gate-rejected detection. */
          if (live.count < AT_LIVE_MAX) {
              at_live_det_t *ld = &live.det[live.count++];
              ld->id       = (int8_t)det->id;
              ld->hamming  = (uint8_t)det->hamming;
              ld->margin   = det->decision_margin;
              ld->cx       = (float)det->c[0];
              ld->cy       = (float)det->c[1];
              ld->pose_err = -1.0f;
              if (det->hamming <= 1 && det->decision_margin > 55.0) {
                  apriltag_detection_info_t live_info = {
                      .det     = det,
                      .tagsize = TAG_SIZE,
                      .fx = F_X, .fy = F_Y,
                      .cx = C_X, .cy = C_Y,
                  };
                  apriltag_pose_t live_pose;
                  ld->pose_err = (float)estimate_tag_pose(&live_info, &live_pose);
                  ld->tx = (float)MATD_EL(live_pose.t, 0, 0);
                  ld->ty = (float)MATD_EL(live_pose.t, 1, 0);
                  ld->tz = (float)MATD_EL(live_pose.t, 2, 0);
                  matd_destroy(live_pose.R);
                  matd_destroy(live_pose.t);
              }
          }

          if (det->hamming > 1 || det->decision_margin <= 55.0) continue;

          /* Pose estimation info — shared by both nav and landing paths */
          apriltag_detection_info_t info = {
            .det     = det,
            .tagsize = TAG_SIZE,
            .fx = F_X, .fy = F_Y,
            .cx = C_X, .cy = C_Y,
          };

          /* ---- Navigation tag → odometry correction (no landing) ---- */
          nav_tag_t nav;
          if (odom_find_nav_tag(det->id, &nav)) {
              apriltag_pose_t pose;
              double err = estimate_tag_pose(&info, &pose);
              if (err < 0.5) {
                  /* cam_in_tag = -R^T * t  (camera position in tag frame) */
                  float tx = (float)MATD_EL(pose.t, 0, 0);
                  float ty = (float)MATD_EL(pose.t, 1, 0);
                  float tz = (float)MATD_EL(pose.t, 2, 0);
                  float r00 = (float)MATD_EL(pose.R, 0, 0);
                  float r10 = (float)MATD_EL(pose.R, 1, 0);
                  float r20 = (float)MATD_EL(pose.R, 2, 0);
                  float r01 = (float)MATD_EL(pose.R, 0, 1);
                  float r11 = (float)MATD_EL(pose.R, 1, 1);
                  float r21 = (float)MATD_EL(pose.R, 2, 1);
                  float cam_tag_x = -(r00 * tx + r10 * ty + r20 * tz);
                  float cam_tag_y = -(r01 * tx + r11 * ty + r21 * tz);
                  odom_on_tag_seen(det->id, cam_tag_x, cam_tag_y);
              } else {
                  ESP_LOGW(TAG, "Nav tag %d pose error too high (%.3f)", det->id, err);
              }
              matd_destroy(pose.R);
              matd_destroy(pose.t);
              continue;   /* nav tags never trigger landing */
          }

          /* ---- Landing tag — existing behaviour ---- */

          /* Skip tags the fleet already found (unless it's our own) */
          if (s_my_tag_id >= 0 && det->id != s_my_tag_id) continue;
          if (s_my_tag_id < 0 && tag_is_known(det->id)) {
              ESP_LOGI(TAG, "Tag %d already known — ignoring", det->id);
              continue;
          }

          ESP_LOGI(TAG, "Apriltag found! ID=%d, DM=%f, hamming=%d", det->id, det->decision_margin, det->hamming);
          s_last_tag_id = det->id;

          apriltag_pose_t pose;
          double err = estimate_tag_pose(&info, &pose);

          /* Only trust estimates with low reprojection error */
          if (err < 0.5) {
            drone_state_t det_drone = mavlink_get_state();
            xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
            s_pose.tx           = (float)MATD_EL(pose.t, 0, 0);
            s_pose.ty           = (float)MATD_EL(pose.t, 1, 0);
            s_pose.tz           = (float)MATD_EL(pose.t, 2, 0);
            s_pose.valid        = true;
            s_pose.tag_id       = det->id;
            s_pose.drone_x      = det_drone.x;
            s_pose.drone_y      = det_drone.y;
            s_pose.drone_heading = det_drone.heading;
            s_pose.detect_ms    = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreGive(s_pose_mutex);

            if (!s_land_requested) {
              s_my_tag_id = (int8_t)det->id;
              s_land_requested = true;
              ESP_LOGW(TAG, "Land request raised (id=%d err=%.3f "
                  "t=[%.2f,%.2f,%.2f])",
                  det->id, err,
                  s_pose.tx, s_pose.ty, s_pose.tz);
            }
          } else {
            ESP_LOGW(TAG, "Pose error too high (%.3f) — skipping", err);
          }

          matd_destroy(pose.R);
          matd_destroy(pose.t);

        }

        /* Publish this frame's detections (count may be 0 — that means
         * "camera alive, no tag in view", which the debug UI relies on). */
        live.frame_ms  = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t proc_ms = live.frame_ms - proc_start_ms;
        live.proc_ms   = (proc_ms > 65535u) ? 65535u : (uint16_t)proc_ms;
        live.raw_count = (uint8_t)zarray_size(at_detections);
        xSemaphoreTake(s_live_mutex, portMAX_DELAY);
        s_live = live;
        xSemaphoreGive(s_live_mutex);

        ESP_LOGI(TAG, "%d Apriltags Found!", zarray_size(at_detections));

        // cleanup
        apriltag_detections_destroy(at_detections);

        camera_stream_frame(&stream, pic);

        esp_camera_fb_return(pic);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif
}
