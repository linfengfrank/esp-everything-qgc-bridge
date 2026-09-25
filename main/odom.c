#include "odom.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "odom";

/* ---------------------------------------------------------------------------
 * Shared state
 * --------------------------------------------------------------------------- */
static nav_tag_t         s_nav_tags[ODOM_MAX_NAV_TAGS];
static int               s_nav_tag_count = 0;

/* map_T_odom: map_pos = odom_pos + s_offset (start position; fixed) */
static float             s_offset_x = 0.0f;
static float             s_offset_y = 0.0f;

static SemaphoreHandle_t s_mutex;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

void odom_init(void)
{
    s_nav_tag_count = 0;
    s_offset_x = s_offset_y = 0.0f;
    memset(s_nav_tags, 0, sizeof(s_nav_tags));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

/* ---------------------------------------------------------------------------
 * Initial offset
 * --------------------------------------------------------------------------- */

void odom_set_initial_offset(float start_map_x, float start_map_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_offset_x = start_map_x;
    s_offset_y = start_map_y;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Initial map_T_odom set to (%.2f, %.2f)", start_map_x, start_map_y);
}

/* ---------------------------------------------------------------------------
 * Nav-tag table
 * --------------------------------------------------------------------------- */

void odom_set_nav_tags(const nav_tag_t *tags, int count)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav_tag_count = 0;
    for (int i = 0; i < count && s_nav_tag_count < ODOM_MAX_NAV_TAGS; i++) {
        if (tags[i].id >= 0) {
            s_nav_tags[s_nav_tag_count++] = tags[i];
        }
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Nav-tag table updated (%d tags, odom frame)", s_nav_tag_count);
    for (int i = 0; i < s_nav_tag_count; i++) {
        ESP_LOGI(TAG, "  tag %d → odom (%.2f, %.2f)",
                 s_nav_tags[i].id, s_nav_tags[i].x, s_nav_tags[i].y);
    }
}

bool odom_find_nav_tag(int tag_id, nav_tag_t *out)
{
    bool found = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_nav_tag_count; i++) {
        if (s_nav_tags[i].id == (int8_t)tag_id) {
            if (out) *out = s_nav_tags[i];
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return found;
}

/* ---------------------------------------------------------------------------
 * Frame conversion
 * --------------------------------------------------------------------------- */

void odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float ox = s_offset_x;
    float oy = s_offset_y;
    xSemaphoreGive(s_mutex);
    *map_x = odom_x + ox;
    *map_y = odom_y + oy;
}

void map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float ox = s_offset_x;
    float oy = s_offset_y;
    xSemaphoreGive(s_mutex);
    *odom_x = map_x - ox;
    *odom_y = map_y - oy;
}
