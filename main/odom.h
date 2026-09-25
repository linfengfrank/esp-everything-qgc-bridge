#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * map_T_odom transform management
 *
 * Frames
 * ------
 *   odom  — PX4's local NED, origin at takeoff.  Continuous, never modified.
 *   map   — Global arena frame (setup.yaml coordinates).
 *
 * Transform  (translation only — both frames share the same NED orientation)
 * ---------
 *   map_pos  = odom_pos + map_T_odom
 *   odom_pos = map_pos  - map_T_odom
 *
 *   map_T_odom = start position from CMD_SET_NAV_TAGS (0,0 until then); fixed.
 *   The nav-tag table only lets at_detect skip nav tags.
 * --------------------------------------------------------------------------- */

#define ODOM_MAX_NAV_TAGS   16

/* A navigation tag at its known position in the drone's odom frame.
 * x, y are set by the laptop as:  tag_map_x − drone_start_x  (and y). */
typedef struct {
    int8_t  id;
    float   x;      /* odom-frame NED north (m) */
    float   y;      /* odom-frame NED east  (m) */
} nav_tag_t;

/* Call once before spawning tasks. */
void odom_init(void);

/* Set the initial map_T_odom from the drone's known start position.
 * Called when CMD_SET_NAV_TAGS is received from the laptop.  Thread-safe. */
void odom_set_initial_offset(float start_map_x, float start_map_y);

/* Set / replace the nav-tag table.  Positions must be in odom frame
 * (pre-computed by laptop).  Thread-safe. */
void odom_set_nav_tags(const nav_tag_t *tags, int count);

/* Look up a tag ID; fills *out if non-NULL.  Thread-safe. */
bool odom_find_nav_tag(int tag_id, nav_tag_t *out);

/* ---------------------------------------------------------------------------
 * Frame conversion  (thread-safe, use current map_T_odom)
 * --------------------------------------------------------------------------- */

void odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y);
void map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y);
