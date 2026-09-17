/* Force-included into every esp-apriltag source (see CMakeLists.txt).
 * Plain malloc() puts blocks under CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL in
 * internal RAM, and the detector's many small blocks drained it to a few
 * bytes during detection, starving WiFi/lwIP.  Prefer PSRAM for this
 * library only; free() handles blocks from either heap. */
#pragma once
#include <stdlib.h>
#include "esp_heap_caps.h"

#define APRILTAG_CAPS_PSRAM    (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define APRILTAG_CAPS_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

#define malloc(size)       heap_caps_malloc_prefer((size), 2, APRILTAG_CAPS_PSRAM, APRILTAG_CAPS_INTERNAL)
#define calloc(n, size)    heap_caps_calloc_prefer((n), (size), 2, APRILTAG_CAPS_PSRAM, APRILTAG_CAPS_INTERNAL)
#define realloc(ptr, size) heap_caps_realloc_prefer((ptr), (size), 2, APRILTAG_CAPS_PSRAM, APRILTAG_CAPS_INTERNAL)
