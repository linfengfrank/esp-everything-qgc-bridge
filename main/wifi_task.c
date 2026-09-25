#include "wifi_task.h"
#include "mavlink_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "odom.h"
#include "tof_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT  BIT0

/* Cruise altitude for CMD_GOTO — must match CRUISE_ALT_M in main.c */
#define WIFI_CRUISE_ALT_M   0.5f

static volatile bool s_land_requested  = false;
static volatile bool s_start_requested = false;
static volatile mission_phase_t s_phase = MISSION_BUSY;
static volatile bool s_wifi_connected  = false;
static volatile bool s_camera_stream_requested = false;
static volatile uint32_t s_camera_stream_keepalive_ms = 0;
/* IPv4 address of the latest enabled camera keepalive sender, in network byte
 * order.  One aligned word keeps the cross-task snapshot atomic. */
static volatile uint32_t s_camera_stream_viewer_ipv4 = 0;

/* Viewer-requested preview fps | quality << 8 (0 = firmware default), in one
 * word so the stream task reads a consistent pair. */
static volatile uint32_t s_camera_stream_params = 0;

/* Viewer refreshes its request once per second. */
#define CAMERA_STREAM_TIMEOUT_MS 2500u

/* Peer drone positions (map frame), protected by s_peer_mutex */
static wifi_peer_list_t   s_peers = { .count = 0 };
static SemaphoreHandle_t  s_peer_mutex;

static EventGroupHandle_t s_wifi_events;

static uint32_t parse_ipv4_or_abort(const char *name, const char *value)
{
    uint32_t addr = esp_ip4addr_aton(value);
    if (addr == IPADDR_NONE) {
        ESP_LOGE(TAG, "Invalid %s IPv4 address: %s", name, value);
        abort();
    }
    return addr;
}

static void configure_static_ip(esp_netif_t *sta_netif)
{
    esp_err_t err = esp_netif_dhcpc_stop(sta_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = parse_ipv4_or_abort(
        "drone static", CONFIG_DRONE_STATIC_IPV4_ADDR);
    ip_info.gw.addr = parse_ipv4_or_abort(
        "gateway", CONFIG_WIFI_GATEWAY_IPV4_ADDR);
    ip_info.netmask.addr = parse_ipv4_or_abort(
        "netmask", CONFIG_WIFI_NETMASK_IPV4_ADDR);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    ESP_LOGI(TAG, "Static IP: %s (gateway %s, netmask %s)",
             CONFIG_DRONE_STATIC_IPV4_ADDR,
             CONFIG_WIFI_GATEWAY_IPV4_ADDR,
             CONFIG_WIFI_NETMASK_IPV4_ADDR);
}

/* ---------------------------------------------------------------------------
 * WiFi event handler — auto-reconnect on disconnect
 * --------------------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Disconnected — reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task_init — connect to AP, block until IP assigned
 * --------------------------------------------------------------------------- */
void wifi_task_init(void)
{
    s_peer_mutex  = xSemaphoreCreateMutex();
    configASSERT(s_peer_mutex != NULL);
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    configASSERT(sta_netif != NULL);
    configure_static_ip(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Disable modem sleep.  The default WIFI_PS_MIN_MODEM powers the PHY down
     * between DTIM beacons and re-runs esp_phy_enable() on every wake, which
     * allocates from internal DRAM.  With the camera + AprilTag detector
     * running, internal RAM is tight enough that this allocation fails and
     * ESP_ERROR_CHECK inside phy_track_pll_init() aborts the whole system.
     * Keeping the PHY on also cuts telemetry/command latency. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to '%s'...", CONFIG_WIFI_SSID);
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------------------------------------------------------------------------
 * Process an incoming command packet from the laptop
 * --------------------------------------------------------------------------- */
static void handle_command(const wifi_cmd_pkt_t *cmd)
{
    /* Always update the fleet's known-tag list */
    at_detect_set_known_tags(cmd->found_tag_ids, WIFI_MAX_FOUND_TAGS);

    switch (cmd->cmd_type) {
    case CMD_START:
        if (s_phase != MISSION_READY) {
            ESP_LOGW(TAG, "CMD_START ignored — not waiting for it");
            break;
        }
        s_phase = MISSION_BUSY;   /* one START per flight */
        s_start_requested = true;
        ESP_LOGI(TAG, "CMD_START");
        break;
    case CMD_GOTO:
        if (s_phase != MISSION_FLYING) {
            ESP_LOGW(TAG, "CMD_GOTO ignored — not flying");
            break;
        }
        nav_set_goal_map(cmd->goal_x, cmd->goal_y, -(WIFI_CRUISE_ALT_M));
        ESP_LOGI(TAG, "CMD_GOTO map(%.2f,%.2f)", cmd->goal_x, cmd->goal_y);
        break;
    case CMD_LAND:
        if (s_phase == MISSION_READY) {
            ESP_LOGW(TAG, "CMD_LAND ignored — not flying");
            break;
        }
        ESP_LOGI(TAG, "CMD_LAND");
        s_land_requested = true;
        break;
    case CMD_HOLD:
        nav_cancel();
        ESP_LOGI(TAG, "CMD_HOLD");
        break;
    default:
        ESP_LOGW(TAG, "Unknown cmd type 0x%02x", cmd->cmd_type);
        break;
    }
}

/* Parse and apply a CMD_SET_NAV_TAGS packet */
static void handle_nav_tags(const uint8_t *buf, int len)
{
    /* Header: pkt_type(1) + cmd_type(1) + tag_count(1) + start_x(4) + start_y(4) = 11 */
    if (len < 11) return;
    uint8_t tag_count = buf[2];
    if (tag_count > WIFI_MAX_NAV_TAGS) tag_count = WIFI_MAX_NAV_TAGS;

    int expected = 11 + tag_count * (int)sizeof(wifi_nav_tag_entry_t);
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_NAV_TAGS truncated (%d < %d)", len, expected);
        return;
    }

    float start_map_x, start_map_y;
    memcpy(&start_map_x, buf + 3, sizeof(float));
    memcpy(&start_map_y, buf + 7, sizeof(float));
    odom_set_initial_offset(start_map_x, start_map_y);

    const wifi_nav_tag_entry_t *entries =
        (const wifi_nav_tag_entry_t *)(buf + 11);

    nav_tag_t tags[WIFI_MAX_NAV_TAGS];
    for (int i = 0; i < tag_count; i++) {
        tags[i].id = entries[i].id;
        tags[i].x  = entries[i].odom_x;
        tags[i].y  = entries[i].odom_y;
    }
    odom_set_nav_tags(tags, tag_count);
    ESP_LOGI(TAG, "CMD_SET_NAV_TAGS: %d tags, start map(%.2f,%.2f)",
             tag_count, start_map_x, start_map_y);
}

/* Parse and apply a CMD_SET_PEERS packet.
 * Wire format: pkt_type(1) + cmd_type(1) + count(1) + count * (float, float) */
static void handle_peers(const uint8_t *buf, int len)
{
    if (len < 3) return;
    uint8_t count = buf[2];
    if (count > WIFI_MAX_PEERS) count = WIFI_MAX_PEERS;

    int expected = 3 + count * (int)(2 * sizeof(float));
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_PEERS truncated (%d < %d)", len, expected);
        return;
    }

    wifi_peer_list_t list;
    list.count = count;
    const uint8_t *p = buf + 3;
    for (int i = 0; i < count; i++) {
        memcpy(&list.peers[i].map_x, p, sizeof(float)); p += sizeof(float);
        memcpy(&list.peers[i].map_y, p, sizeof(float)); p += sizeof(float);
    }

    list.update_ms = (uint32_t)(esp_timer_get_time() / 1000);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    s_peers = list;
    xSemaphoreGive(s_peer_mutex);

    ESP_LOGD(TAG, "CMD_SET_PEERS: %d peers", count);
}

/* CMD_CAMERA_STREAM keepalive: pkt, cmd, enable [, max_fps, quality].
 * The sender becomes the preview destination.  An old viewer cannot turn off
 * a newer viewer's stream when it exits. */
static void handle_camera_stream(const uint8_t *buf, int len,
                                 uint32_t source_ipv4)
{
    if (len < 3) return;
    bool enable = buf[2] != 0;
    uint32_t params = (len >= 5) ? (buf[3] | (uint32_t)buf[4] << 8) : 0;
    bool changed = enable != wifi_camera_stream_enabled()
                   || (enable && (params != s_camera_stream_params
                                  || source_ipv4 != s_camera_stream_viewer_ipv4));

    if (enable) {
        s_camera_stream_params = params;
        s_camera_stream_viewer_ipv4 = source_ipv4;
        s_camera_stream_keepalive_ms = (uint32_t)(esp_timer_get_time() / 1000);
    } else if (source_ipv4 != s_camera_stream_viewer_ipv4) {
        return;
    }
    s_camera_stream_requested = enable;
    if (changed && enable) {
        struct in_addr viewer = { .s_addr = source_ipv4 };
        ESP_LOGI(TAG, "Camera preview -> %s (fps=%u q=%u, 0 = default)",
                 inet_ntoa(viewer), (unsigned)(params & 0xFF),
                 (unsigned)(params >> 8));
    } else if (changed) {
        ESP_LOGI(TAG, "Camera preview off");
    }
}

/* CMD_TRAJ_DATA: pkt, cmd, id, total(u16), offset(u16), n, n × (x, y, z) float32 */
static void handle_traj_data(const uint8_t *buf, int len)
{
    if (len < 8) return;
    uint16_t total, offset;
    memcpy(&total,  buf + 3, sizeof(total));
    memcpy(&offset, buf + 5, sizeof(offset));
    int n = buf[7];
    if (len < 8 + n * 12) {
        ESP_LOGW(TAG, "CMD_TRAJ_DATA truncated (%d < %d)", len, 8 + n * 12);
        return;
    }
    nav_traj_put(buf[2], total, offset, buf + 8, n);
}

/* CMD_TRAJ_START: pkt, cmd, id, dt_ms(u16) */
static void handle_traj_start(const uint8_t *buf, int len)
{
    if (len < 5) return;
    if (s_phase != MISSION_FLYING) {
        ESP_LOGW(TAG, "CMD_TRAJ_START ignored — not flying");
        return;
    }
    uint16_t dt_ms;
    memcpy(&dt_ms, buf + 3, sizeof(dt_ms));
    nav_traj_start(buf[2], dt_ms);
}

wifi_peer_list_t wifi_get_peers(void)
{
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    wifi_peer_list_t copy = s_peers;
    xSemaphoreGive(s_peer_mutex);

    /* Discard peer data older than 1 s — positions are too stale to trust */
    if (copy.count > 0) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - copy.update_ms) > 1000) {
            copy.count = 0;
        }
    }
    return copy;
}

/* ---------------------------------------------------------------------------
 * wifi_task — 10 Hz telemetry loop + non-blocking command receive
 * --------------------------------------------------------------------------- */
void wifi_task(void *arg)
{
    /* Telemetry send socket */
    int tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tx_sock >= 0);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(CONFIG_TELEM_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    connect(tx_sock, (struct sockaddr *)&dest, sizeof(dest));

    /* ToF debug send socket */
    int tof_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tof_sock >= 0);
    struct sockaddr_in tof_dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_TOF_DEBUG_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &tof_dest.sin_addr);
    connect(tof_sock, (struct sockaddr *)&tof_dest, sizeof(tof_dest));

    /* AprilTag debug send socket.  These small packets always go to the
     * configured host and are also copied to a remote active camera viewer,
     * which gives tag_stream.py --detail its ESP overlay. */
    int at_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(at_sock >= 0);
    struct sockaddr_in at_dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_AT_DEBUG_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &at_dest.sin_addr);

    /* Command receive socket (non-blocking) */
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(rx_sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(WIFI_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Command socket bind failed (port %d)", WIFI_CMD_PORT);
    }
    int flags = fcntl(rx_sock, F_GETFL, 0);
    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Telemetry → %s:%d | Commands ← port %d",
             CONFIG_HOST_IPV4_ADDR, CONFIG_TELEM_PORT, WIFI_CMD_PORT);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* ---- Drain all queued commands (non-blocking) ---- */
        {
            uint8_t cmd_buf[WIFI_CMD_BUF_SIZE];
            int len;
            struct sockaddr_in source;
            socklen_t source_len;
            while (source_len = sizeof(source),
                   (len = recvfrom(rx_sock, cmd_buf, sizeof(cmd_buf), 0,
                                   (struct sockaddr *)&source, &source_len)) > 0) {
                if (len < 2 || cmd_buf[0] != WIFI_PKT_CMD) continue;
                if (cmd_buf[1] == CMD_TRAJ_DATA) {
                    handle_traj_data(cmd_buf, len);
                } else if (cmd_buf[1] == CMD_TRAJ_START) {
                    handle_traj_start(cmd_buf, len);
                } else if (cmd_buf[1] == CMD_CAMERA_STREAM) {
                    handle_camera_stream(cmd_buf, len, source.sin_addr.s_addr);
                } else if (cmd_buf[1] == CMD_SET_NAV_TAGS) {
                    handle_nav_tags(cmd_buf, len);
                } else if (cmd_buf[1] == CMD_SET_PEERS) {
                    handle_peers(cmd_buf, len);
                } else if (len >= (int)sizeof(wifi_cmd_pkt_t)) {
                    handle_command((const wifi_cmd_pkt_t *)cmd_buf);
                }
            }
        }

        /* ---- Snapshot all shared state ---- */
        drone_state_t    drone = mavlink_get_state();
        nav_status_t     ns    = nav_get_status();
        at_detect_pose_t pose  = at_detect_get_pose();

        /* ---- Build telemetry packet ---- */
        wifi_telem_pkt_t pkt = {};
        pkt.pkt_type    = WIFI_PKT_TELEM;
        pkt.drone_id    = CONFIG_DRONE_ID;
        /* Report position in map frame so laptop works in a consistent frame */
        odom_to_map(drone.x, drone.y, &pkt.ned_x, &pkt.ned_y);
        pkt.heading_rad = drone.heading;
        pkt.nav_state   = (uint8_t)ns.state;
        pkt.is_stuck    = (ns.state == NAV_STUCK) ? 1 : 0;

        /* AprilTag (info only): claim latched since boot, range at last sighting */
        pkt.tag_id = at_detect_my_tag_id();
        if (pose.valid) {
            pkt.tag_dist_m = sqrtf(pose.tx * pose.tx + pose.tz * pose.tz);
        } else {
            pkt.tag_dist_m = 0.0f;
        }

        /* VFH blocked state */
        for (int b = 0; b < VFH_BINS; b++)
            pkt.vfh_blocked[b] = ns.vfh_blocked[b] ? 1 : 0;

        pkt.reloc_age_s = 0xFFFF;   /* never relocalised */

        /* ---- Send telemetry ---- */
        send(tx_sock, &pkt, sizeof(pkt), 0);

        /* ---- Send raw ToF debug frame (front sensor only) ---- */
        {
            tof_scan_t scan = tof_get_scan();
            wifi_tof_debug_pkt_t dbg = {};
            dbg.pkt_type    = WIFI_PKT_TOF_DEBUG;
            dbg.sensor_idx  = TOF_FRONT_SENSOR_IDX;
            dbg.sensor_ok   = scan.sensor_ok[TOF_FRONT_SENSOR_IDX];
            dbg.timestamp_ms = scan.frame[TOF_FRONT_SENSOR_IDX].timestamp_ms;
            memcpy(dbg.distance_mm,   scan.frame[TOF_FRONT_SENSOR_IDX].distance_mm,
                   sizeof(dbg.distance_mm));
            memcpy(dbg.target_status, scan.frame[TOF_FRONT_SENSOR_IDX].target_status,
                   sizeof(dbg.target_status));
            send(tof_sock, &dbg, sizeof(dbg), 0);
        }

        /* ---- Send live AprilTag detections (most recent camera frame) ---- */
        {
            _Static_assert(AT_LIVE_MAX == WIFI_AT_DEBUG_MAX,
                           "AT debug wire format out of sync with at_detect");
            at_live_dets_t live = at_detect_get_live();
            wifi_at_debug_pkt_t dbg = {};
            dbg.pkt_type   = WIFI_PKT_AT_DEBUG;
            dbg.drone_id   = CONFIG_DRONE_ID;
            dbg.frame_ms   = live.frame_ms;
            dbg.proc_ms    = live.proc_ms;
            dbg.latched_id = at_detect_my_tag_id();
            dbg.raw_count  = live.raw_count;
            dbg.count      = live.count;
            for (int i = 0; i < live.count; i++) {
                dbg.det[i].id       = live.det[i].id;
                dbg.det[i].hamming  = live.det[i].hamming;
                dbg.det[i].margin   = live.det[i].margin;
                dbg.det[i].cx       = live.det[i].cx;
                dbg.det[i].cy       = live.det[i].cy;
                dbg.det[i].tx       = live.det[i].tx;
                dbg.det[i].ty       = live.det[i].ty;
                dbg.det[i].tz       = live.det[i].tz;
                dbg.det[i].pose_err = live.det[i].pose_err;
            }
            size_t wire_len = sizeof(dbg)
                - (size_t)(WIFI_AT_DEBUG_MAX - dbg.count) * sizeof(wifi_at_det_t);
            sendto(at_sock, &dbg, wire_len, 0,
                   (const struct sockaddr *)&at_dest, sizeof(at_dest));

            wifi_camera_stream_req_t preview;
            if (wifi_camera_stream_get(&preview)
                    && preview.viewer_ipv4 != at_dest.sin_addr.s_addr) {
                struct sockaddr_in viewer_dest = at_dest;
                viewer_dest.sin_addr.s_addr = preview.viewer_ipv4;
                sendto(at_sock, &dbg, wire_len, 0,
                       (const struct sockaddr *)&viewer_dest,
                       sizeof(viewer_dest));
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}

bool wifi_land_requested(void)
{
    return s_land_requested;
}

void wifi_clear_land_request(void)
{
    s_land_requested = false;
}

bool wifi_start_requested(void)
{
    return s_start_requested;
}

void wifi_clear_start_request(void)
{
    s_start_requested = false;
}

void wifi_set_mission_phase(mission_phase_t phase)
{
    s_phase = phase;
}

bool wifi_is_connected(void)
{
    return s_wifi_connected;
}

bool wifi_camera_stream_enabled(void)
{
    if (!s_camera_stream_requested) return false;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return (uint32_t)(now_ms - s_camera_stream_keepalive_ms)
           < CAMERA_STREAM_TIMEOUT_MS;
}

bool wifi_camera_stream_get(wifi_camera_stream_req_t *out)
{
    uint32_t params = s_camera_stream_params;
    out->max_fps = (uint8_t)params;
    out->quality = (uint8_t)(params >> 8);
    out->viewer_ipv4 = s_camera_stream_viewer_ipv4;
    return wifi_camera_stream_enabled();
}
