#pragma once

#include <stdbool.h>

#include "esp_camera.h"

/* On-demand JPEG camera preview for laptop/camera_stream.py.  Core 0 at the
 * lowest priority, so encoding only uses time mavlink_task and wifi_task
 * leave idle.  The stack is in PSRAM; jpge + send() peak at ~4 KB. */
#define CAMERA_STREAM_TASK_CORE      0
#define CAMERA_STREAM_TASK_PRIORITY  1
#define CAMERA_STREAM_TASK_STACK     6144

/* Called once by at_detect_task after the camera is initialised.  A failure
 * only logs a warning and leaves the preview off. */
void camera_stream_start(void);

/* True once the stream task is running. */
bool camera_stream_active(void);

/* esp_camera_fb_get() that never returns a frame left stale in the driver's
 * queue.  Used by both camera consumers; return the frame as usual. */
camera_fb_t *camera_fb_get_fresh(void);
