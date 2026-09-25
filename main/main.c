#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "wifi_task.h"
#include "odom.h"

#include <math.h>

static const char *TAG = "mission";

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define CRUISE_ALT_M        0.5f    /* target altitude above takeoff (m)        */
#define ARM_TIMEOUT_S       10      /* give up on OFFBOARD + arm (s)            */
#define TAKEOFF_TIMEOUT_S   10      /* max seconds to wait for altitude (s)     */
#define LAND_TIMEOUT_S      20      /* warn if still armed after landing (s)    */
#define ALT_TOLERANCE_M     0.15f   /* altitude band considered "at altitude"   */

/* Consume a pending CMD_LAND. */
static bool land_requested(void)
{
    if (!wifi_land_requested()) return false;
    wifi_clear_land_request();
    ESP_LOGI(TAG, "CMD_LAND received from laptop");
    return true;
}

/* ---------------------------------------------------------------------------
 * Mission task: START → OFFBOARD → arm → takeoff → laptop control → LAND,
 * repeat.  Owns the setpoint until a GOTO/trajectory; nav_cancel() returns it.
 * --------------------------------------------------------------------------- */
static void mission_task(void *arg)
{
    float target_z   = -(CRUISE_ALT_M);   /* NED: negative = above ground */
    float takeoff_x  = 0.0f;
    float takeoff_y  = 0.0f;

    for (;;) {

    /* ------------------------------------------------------------------ */
    /* Phase 1: Wait for valid telemetry                                   */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for telemetry...");
    while (1) {
        drone_state_t st = mavlink_get_state();
        if (mavlink_position_valid() && st.last_hb_ms != 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Telemetry valid");

    /* ------------------------------------------------------------------ */
    /* Phase 2: Pre-stream hold setpoint                                   */
    /* PX4 requires setpoints to already be streaming before it will       */
    /* accept an OFFBOARD mode switch.  Stream for 2 s.                    */
    /* ------------------------------------------------------------------ */
    mavlink_set_hold();
    nav_cancel();                 /* drop stale goal + commands */
    wifi_clear_start_request();
    wifi_clear_land_request();
    wifi_set_mission_phase(MISSION_READY);
    ESP_LOGI(TAG, "Pre-streaming hold setpoint for 2 s...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ------------------------------------------------------------------ */
    /* Phase 2b: Wait for CMD_START (wifi_task switches to BUSY on it)     */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for CMD_START from laptop...");
    while (!wifi_start_requested()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    wifi_clear_start_request();
    ESP_LOGI(TAG, "CMD_START received");

    /* Pre-arm: refuse to arm until all ToF sensors are up. */
#if TOF_ENABLED
    {
        ESP_LOGI(TAG, "Checking ToF sensors...");
        int tof_ok = tof_sensors_ok_count();
        while (tof_ok < TOF_SENSOR_COUNT) {
            ESP_LOGE(TAG, "TOF CHECK FAILED: %d / %d sensors OK — refusing to arm",
                     tof_ok, TOF_SENSOR_COUNT);
            vTaskDelay(pdMS_TO_TICKS(2000));
            tof_ok = tof_sensors_ok_count();
        }
        ESP_LOGI(TAG, "All %d ToF sensors OK — proceeding to arm", tof_ok);
    }
#else
    ESP_LOGW(TAG, "ToF disabled (TOF_ENABLED=0) — skipping pre-arm sensor check");
#endif

    /* ------------------------------------------------------------------ */
    /* Phase 3: OFFBOARD, then arm — retry every 500 ms.                   */
    /* CMD_LAND or ARM_TIMEOUT_S aborts and returns to waiting.            */
    /* ------------------------------------------------------------------ */
    const char *abort_why = NULL;
    int tries = ARM_TIMEOUT_S * 2;

    ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
    while (mavlink_get_state().custom_main_mode != PX4_MAIN_MODE_OFFBOARD) {
        if (land_requested())  { abort_why = "CMD_LAND"; break; }
        if (tries-- == 0)      { abort_why = "OFFBOARD refused (check QGC)"; break; }
        mavlink_set_offboard_mode();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!abort_why) {
        ESP_LOGI(TAG, "OFFBOARD mode confirmed");
        ESP_LOGI(TAG, "Arming...");
    }
    while (!abort_why && !mavlink_get_state().armed) {
        if (land_requested())  { abort_why = "CMD_LAND"; break; }
        if (tries-- == 0)      { abort_why = "arming refused (check QGC)"; break; }
        mavlink_arm(true);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (abort_why) {
        mavlink_arm(false);   /* in case the arm just went through */
        ESP_LOGW(TAG, "Takeoff aborted: %s", abort_why);
        continue;
    }
    ESP_LOGI(TAG, "Armed confirmed");

    /* ------------------------------------------------------------------ */
    /* Phase 4: Take off (mission_task owns the setpoint); CMD_LAND lands  */
    /* ------------------------------------------------------------------ */
    {
        drone_state_t st = mavlink_get_state();
        takeoff_x = st.x;
        takeoff_y = st.y;
        mavlink_set_position_ned(takeoff_x, takeoff_y, target_z, st.heading);
    }
    ESP_LOGI(TAG, "Taking off to %.1f m AGL (NED z=%.2f)...", CRUISE_ALT_M, target_z);

    bool land_now = false;
    for (int i = 0; i < TAKEOFF_TIMEOUT_S * 10 && !land_now; i++) {
        drone_state_t st = mavlink_get_state();
        if (!st.armed || fabsf(st.z - target_z) < ALT_TOLERANCE_M) break;
        land_now = land_requested();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool armed = mavlink_get_state().armed;
    if (!armed) ESP_LOGW(TAG, "Disarmed without CMD_LAND");

    /* ------------------------------------------------------------------ */
    /* Phase 5: Laptop control (GOTO / HOLD / trajectory) until LAND      */
    /* ------------------------------------------------------------------ */
    if (!land_now && armed) {
        float z = mavlink_get_state().z;
        if (fabsf(z - target_z) < ALT_TOLERANCE_M) {
            ESP_LOGI(TAG, "Altitude reached: NED z=%.2f (target=%.2f)", z, target_z);
        } else {
            ESP_LOGW(TAG, "Takeoff timeout: NED z=%.2f (target=%.2f)", z, target_z);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        wifi_set_mission_phase(MISSION_FLYING);
        ESP_LOGI(TAG, "Exploration mode — waiting for laptop goals...");
        while (!land_requested()) {
            if (!mavlink_get_state().armed) {   /* landed by RC / QGC */
                ESP_LOGW(TAG, "Disarmed without CMD_LAND");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        wifi_set_mission_phase(MISSION_BUSY);
    }

    /* ------------------------------------------------------------------ */
    /* Phase 6: Land, wait for disarm.  LAND is (re)sent every 2 s only    */
    /* while in OFFBOARD, so an RC takeover is never overridden.           */
    /* ------------------------------------------------------------------ */
    nav_cancel();
    for (int i = 0; mavlink_get_state().armed; i++) {
        bool offboard = mavlink_get_state().custom_main_mode == PX4_MAIN_MODE_OFFBOARD;
        if (i % 20 == 0 && offboard) {
            mavlink_send_land_command();
            if (i == 0) ESP_LOGI(TAG, "Land command sent");
        }
        if (i == 0 && !offboard) ESP_LOGW(TAG, "Not in OFFBOARD — RC has control, LAND not sent");
        if (i == LAND_TIMEOUT_S * 10) {
            ESP_LOGW(TAG, "Still armed after %d s — land with RC", LAND_TIMEOUT_S);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Disarmed — mission complete");

    ESP_LOGI(TAG, "Mission loop complete — waiting for next CMD_START");
    }
}

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all tasks
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    /* NVS must be init before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Init all modules before spawning tasks */
    mavlink_task_init();
    tof_task_init();    /* always: creates the scan mutex the tof_get_*() accessors take */
    nav_task_init();
    at_detect_init();
    odom_init();
    wifi_task_init();   /* blocks until IP obtained */

    /* Core 0: hardware-facing tasks + WiFi telemetry */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
#if TOF_ENABLED
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );
#endif
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", WIFI_TASK_STACK,
        NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE
    );

    /* Core 1: navigator + AprilTag + mission
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        at_detect_task, "apriltag", AT_TASK_STACK,
        NULL, AT_TASK_PRIORITY, NULL, AT_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 6144,
        NULL, 2, NULL, 1
    );

    ESP_LOGI(TAG, "All tasks spawned");
}
