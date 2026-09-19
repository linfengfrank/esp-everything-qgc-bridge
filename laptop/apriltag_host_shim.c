/* Flat C API over the in-tree esp-apriltag detector, for laptop/apriltag_host.py.
 *
 * Wraps the same sources the firmware compiles (components/esp-apriltag)
 * rather than a pip-installed AprilTag, so the code table, the decision-margin
 * scale (3.4.5 computes it after decode_sharpening) and the pose call all
 * match main/at_detect.c.  Everything apriltag-internal stays on this side of
 * the wall: Python only ever sees ath_detection_t.
 *
 * Built on demand by apriltag_host.py; never compiled into the firmware.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag16h5.h"
#include "common/image_types.h"
#include "common/matd.h"
#include "common/zarray.h"

/* Bumped whenever ath_detection_t or a signature below changes, so a stale
 * cached library cannot be loaded by a newer apriltag_host.py. */
#define ATH_ABI_VERSION 1

/* Plain, explicitly padded struct — must match _CDetection in apriltag_host.py. */
typedef struct {
    int32_t id;
    int32_t hamming;
    int32_t pose_valid;      /* 1 = t/R/pose_err filled (detection passed the gate) */
    int32_t reserved;
    double  decision_margin;
    double  c[2];            /* centre, AprilTag pixel convention */
    double  p[4][2];         /* corners, counter-clockwise, same convention */
    double  t[3];            /* camera-frame translation (m): X right, Y down, Z fwd */
    double  R[9];            /* rotation, row-major */
    double  pose_err;        /* estimate_tag_pose() reprojection error */
} ath_detection_t;

typedef struct {
    apriltag_family_t   *tf;
    apriltag_detector_t *td;
} ath_handle_t;

int ath_abi_version(void)
{
    return ATH_ABI_VERSION;
}

/* Parameters are the firmware's (at_detect_task in main/at_detect.c); Python
 * passes them in so they are written down in exactly one place. */
void *ath_detector_create(double quad_decimate, double quad_sigma,
                          int refine_edges, double decode_sharpening,
                          int nthreads)
{
    ath_handle_t *h = calloc(1, sizeof(ath_handle_t));
    if (h == NULL)
        return NULL;
    h->tf = tag16h5_create();
    h->td = apriltag_detector_create();
    if (h->tf == NULL || h->td == NULL) {
        if (h->td != NULL) apriltag_detector_destroy(h->td);
        if (h->tf != NULL) tag16h5_destroy(h->tf);
        free(h);
        return NULL;
    }
    apriltag_detector_add_family(h->td, h->tf);
    h->td->quad_decimate     = (float)quad_decimate;
    h->td->quad_sigma        = (float)quad_sigma;
    h->td->refine_edges      = refine_edges ? 1 : 0;
    h->td->decode_sharpening = decode_sharpening;
    h->td->nthreads          = nthreads < 1 ? 1 : nthreads;
    h->td->debug             = 0;
    return h;
}

void ath_detector_destroy(void *handle)
{
    ath_handle_t *h = (ath_handle_t *)handle;
    if (h == NULL)
        return;
    /* The detector does not own the family (see apriltag.h). */
    apriltag_detector_destroy(h->td);
    tag16h5_destroy(h->tf);
    free(h);
}

/* How many IDs this family can decode — guards against an ncodes that overruns
 * the truncated tag16h5 code table. */
int ath_family_ncodes(void *handle)
{
    ath_handle_t *h = (ath_handle_t *)handle;
    return (h == NULL || h->tf == NULL) ? -1 : (int)h->tf->ncodes;
}

/* Detect on a grayscale buffer (stride is usually width — the firmware runs
 * straight on the camera frame buffer).
 *
 * Fills up to max_out entries of out[] and returns the number of detections
 * in the frame, which may be larger; < 0 means bad arguments.  A pose is
 * estimated only for detections passing the caller's gate, mirroring the
 * firmware, which never poses a detection it would have thrown away. */
int ath_detect(void *handle, const uint8_t *buf,
               int32_t width, int32_t height, int32_t stride,
               double tagsize, double fx, double fy, double cx, double cy,
               int32_t gate_max_hamming, double gate_min_margin,
               ath_detection_t *out, int32_t max_out)
{
    ath_handle_t *h = (ath_handle_t *)handle;
    if (h == NULL || buf == NULL || width <= 0 || height <= 0 || stride < width)
        return -1;
    if (max_out < 0 || (max_out > 0 && out == NULL))
        return -1;

    /* image_u8_t borrows the caller's buffer, exactly like at_detect.c. */
    image_u8_t im = { .width = width, .height = height,
                      .stride = stride, .buf = (uint8_t *)buf };

    zarray_t *dets = apriltag_detector_detect(h->td, &im);
    if (dets == NULL)
        return -1;
    int total = zarray_size(dets);

    for (int i = 0; i < total && i < max_out; i++) {
        apriltag_detection_t *d;
        zarray_get(dets, i, &d);
        ath_detection_t *o = &out[i];
        memset(o, 0, sizeof(*o));
        o->id              = d->id;
        o->hamming         = d->hamming;
        o->decision_margin = d->decision_margin;
        o->c[0]            = d->c[0];
        o->c[1]            = d->c[1];
        memcpy(o->p, d->p, sizeof(o->p));
        o->pose_err        = -1.0;

        if (d->hamming <= gate_max_hamming
                && d->decision_margin > gate_min_margin) {
            apriltag_detection_info_t info = {
                .det = d, .tagsize = tagsize,
                .fx = fx, .fy = fy, .cx = cx, .cy = cy,
            };
            apriltag_pose_t pose;
            o->pose_err = estimate_tag_pose(&info, &pose);
            for (int k = 0; k < 3; k++)
                o->t[k] = MATD_EL(pose.t, k, 0);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    o->R[r * 3 + c] = MATD_EL(pose.R, r, c);
            o->pose_valid = 1;
            matd_destroy(pose.R);
            matd_destroy(pose.t);
        }
    }

    apriltag_detections_destroy(dets);
    return total;
}
