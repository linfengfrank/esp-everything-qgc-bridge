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

#include "freertos/semphr.h"

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
// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "camera_stream.h"
#include "wifi_task.h"

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
    /* Two buffers keep the driver capturing while one is held, so a frame is
     * usually ready at once: the preview runs at the sensor rate (~9 fps on
     * the OV3660) instead of ~4 fps with one buffer.  ("JPEG only" upstream
     * is just a note; grayscale QVGA works.) */
    .fb_count = 2,
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

/* While the preview is on, detect on a copy so the camera buffer goes back
 * at once and the stream task is not left with a single buffer. */
#define AT_FRAME_COPY_BYTES (320 * 240)

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
    /* A task must not return (ESP-IDF aborts): end only this task. */
    if(ESP_OK != init_camera()) {
        ESP_LOGE(TAG, "Camera init failed — AprilTag detection and camera "
                      "stream disabled; flight is unaffected");
        vTaskDelete(NULL);
    }

    /* NULL: always detect on the camera buffer (the preview just slows). */
    uint8_t *frame_copy = heap_caps_malloc(AT_FRAME_COPY_BYTES,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    camera_stream_start();

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
        /* Detections are paired with the pose read after detection, so a
         * frame left queued during the last detection must not be used. */
        camera_fb_t *pic = camera_fb_get_fresh();
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

        /* Preview on: detect on a copy and give the buffer back now. */
        size_t pixels = (size_t)pic->width * pic->height;
        if (camera_stream_active() && wifi_camera_stream_enabled()
                && frame_copy != NULL
                && pic->format == PIXFORMAT_GRAYSCALE
                && pixels <= AT_FRAME_COPY_BYTES && pic->len >= pixels) {
          memcpy(frame_copy, pic->buf, pixels);
          at_im.buf = frame_copy;
          esp_camera_fb_return(pic);
          pic = NULL;
        }

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

          /* Nav tags: landmarks only, never claimed. */
          if (odom_find_nav_tag(det->id, NULL)) continue;

          /* ---- Other tag → claim + pose, telemetry only ---- */

          /* Skip tags the fleet already found (unless it's our own) */
          if (s_my_tag_id >= 0 && det->id != s_my_tag_id) continue;
          if (s_my_tag_id < 0 && tag_is_known(det->id)) {
              ESP_LOGI(TAG, "Tag %d already known — ignoring", det->id);
              continue;
          }

          ESP_LOGI(TAG, "Apriltag found! ID=%d, DM=%f, hamming=%d", det->id, det->decision_margin, det->hamming);

          apriltag_detection_info_t info = {
            .det     = det,
            .tagsize = TAG_SIZE,
            .fx = F_X, .fy = F_Y,
            .cx = C_X, .cy = C_Y,
          };
          apriltag_pose_t pose;
          double err = estimate_tag_pose(&info, &pose);

          /* Only trust estimates with low reprojection error */
          if (err < 0.5) {
            xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
            s_pose.tx           = (float)MATD_EL(pose.t, 0, 0);
            s_pose.ty           = (float)MATD_EL(pose.t, 1, 0);
            s_pose.tz           = (float)MATD_EL(pose.t, 2, 0);
            s_pose.valid        = true;
            xSemaphoreGive(s_pose_mutex);

            if (s_my_tag_id < 0) {
              s_my_tag_id = (int8_t)det->id;
              ESP_LOGW(TAG, "Tag %d claimed (err=%.3f t=[%.2f,%.2f,%.2f]) — "
                  "reported in telemetry, no flight action",
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

        if (pic != NULL) {
          esp_camera_fb_return(pic);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    vTaskDelete(NULL);
#endif
}
