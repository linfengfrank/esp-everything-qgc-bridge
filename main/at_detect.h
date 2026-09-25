#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Core 1, low priority.  Detection only — never affects flight. */
#define AT_TASK_CORE        1
#define AT_TASK_PRIORITY    1
#define AT_TASK_STACK       12288

#define AT_MAX_KNOWN_TAGS   12

/* Max detections reported per camera frame in the live debug snapshot. */
#define AT_LIVE_MAX         8

void at_detect_init(void);
void at_detect_task(void* pvParams);

/* Latched tag ID this drone found (−1 if none yet). Set once, never overwritten. */
int8_t at_detect_my_tag_id(void);

/* Update the fleet-wide known-tags list (from laptop command).
 * Entries equal to −1 are ignored. Thread-safe. */
void at_detect_set_known_tags(const int8_t *ids, int count);

/* Claimed tag's pose at its last good sighting (camera frame, m; never
 * cleared).  X = right, Y = down, Z = forward.  Read via at_detect_get_pose(). */
typedef struct {
    float tx, ty, tz;   /* translation from camera origin to tag centre */
    bool  valid;        /* true once a low-error estimate exists         */
} at_detect_pose_t;

/* Thread-safe snapshot of the latest pose estimate.
 * Returns {.valid = false} if no estimate has been computed yet. */
at_detect_pose_t at_detect_get_pose(void);

/* One raw detection from the most recent camera frame (debug stream).
 * Recorded before any nav-tag / known-tag / latch filtering, so this is
 * the ground truth of what the detector currently sees. */
typedef struct {
    int8_t  id;         /* tag ID                                        */
    uint8_t hamming;    /* corrected bit errors                          */
    float   margin;     /* decision margin (higher = more confident)     */
    float   cx, cy;     /* tag centre in image pixels                    */
    float   tx, ty, tz; /* camera-frame translation (m), see pose_err    */
    float   pose_err;   /* pose reprojection error; < 0 = no pose
                           (detection failed the hamming/margin gate)    */
} at_live_det_t;

typedef struct {
    uint32_t      frame_ms;   /* esp_timer ms when this frame was processed */
    uint16_t      proc_ms;    /* frame processing time (detect + pose), ms  */
    uint8_t       raw_count;  /* detections returned by the detector        */
    uint8_t       count;      /* entries stored in det[] (≤ AT_LIVE_MAX)    */
    at_live_det_t det[AT_LIVE_MAX];
} at_live_dets_t;

/* Thread-safe snapshot of all detections in the most recent camera frame.
 * frame_ms == 0 until the first frame has been processed. */
at_live_dets_t at_detect_get_live(void);
