#include "web_server.h"
#include "index_html.h"

#include "ble_central.h"
#include "bridge.h"
#include "captive_portal.h"
#include "config_store.h"
#include "controller_manager.h"
#include "dongle_state.h"
#include "mouse_mode.h"

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_coexist.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static httpd_handle_t s_httpd;
static esp_netif_t *s_ap_netif;
static bool s_wifi_started;
static bool s_netif_created;
static volatile bool s_start_requested;
static volatile bool s_start_explicit;
static bool s_wifi_event_registered;
static bool s_ip_event_registered;
static int s_sta_count;
static int64_t s_last_sta_join_us;
static int64_t s_last_ip_assigned_us;
static uint8_t s_last_sta_mac[6];
static bool s_dhcp_wait_logged;
static volatile bool s_dhcp_ensure_requested;
static volatile bool s_ble_scan_pause_requested;
static volatile bool s_ble_scan_resume_requested;
static volatile bool s_ap_stopped_for_suspend;
static volatile bool s_usb_suspended;

static void parse_form_value(const char *body, const char *key, char *out, size_t out_len);
static void update_webui_deadline(void);

static void ensure_ap_dhcp_running(void)
{
    if (!s_ap_netif) return;

    esp_netif_dhcp_status_t status = ESP_NETIF_DHCP_INIT;
    esp_err_t err = esp_netif_dhcps_get_status(s_ap_netif, &status);
    if (err != ESP_OK) {
        ESP_LOGW("WEB", "AP DHCP status read failed: %s", esp_err_to_name(err));
        return;
    }
    if (status == ESP_NETIF_DHCP_STARTED) {
        ESP_LOGI("WEB", "AP DHCP running on default SoftAP netif");
        return;
    }

    err = esp_netif_dhcps_start(s_ap_netif);
    if (err == ESP_OK || err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGI("WEB", "AP DHCP restarted on default SoftAP netif (previous status=%d)", (int)status);
    } else {
        ESP_LOGW("WEB", "AP DHCP restart failed: status=%d err=%s", (int)status, esp_err_to_name(err));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *ev = data;
        s_sta_count++;
        memcpy(s_last_sta_mac, ev->mac, sizeof(s_last_sta_mac));
        s_last_sta_join_us = esp_timer_get_time();
        s_dhcp_wait_logged = false;
        s_dhcp_ensure_requested = true;
        ble_central_set_scan_paused(true);
        s_ble_scan_pause_requested = true;
        ESP_LOGI("WEB", "AP station connected " MACSTR " aid=%d", MAC2STR(ev->mac), ev->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *ev = data;
        if (s_sta_count > 0) s_sta_count--;
        if (s_sta_count == 0) {
            ble_central_set_scan_paused(false);
            s_ble_scan_resume_requested = true;
        }
        ESP_LOGW("WEB", "AP station disconnected " MACSTR " aid=%d reason=%d",
                 MAC2STR(ev->mac), ev->aid, ev->reason);
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        const ip_event_assigned_ip_to_client_t *ev = data;
        s_last_ip_assigned_us = esp_timer_get_time();
        s_dhcp_wait_logged = false;
        ble_central_set_scan_paused(false);
        s_ble_scan_resume_requested = true;
        ESP_LOGI("WEB", "AP assigned IP " IPSTR, IP2STR(&ev->ip));
    }
}

static void log_dhcp_wait(void)
{
    esp_netif_dhcp_status_t status = ESP_NETIF_DHCP_INIT;
    esp_err_t status_err = s_ap_netif ? esp_netif_dhcps_get_status(s_ap_netif, &status) : ESP_FAIL;
    esp_netif_pair_mac_ip_t pair = {0};
    memcpy(pair.mac, s_last_sta_mac, sizeof(pair.mac));
    esp_err_t client_err = s_ap_netif ? esp_netif_dhcps_get_clients_by_mac(s_ap_netif, 1, &pair) : ESP_FAIL;
    ESP_LOGW("WEB",
             "AP station has no DHCP lease after 10 s; dhcps_status=%d(%s) lookup=%s ip=" IPSTR,
             (int)status, esp_err_to_name(status_err), esp_err_to_name(client_err), IP2STR(&pair.ip));
    if (status != ESP_NETIF_DHCP_STARTED) {
        s_dhcp_ensure_requested = true;
    }
}

static esp_err_t send_text(httpd_req_t *req, const char *text, const char *type)
{
    httpd_resp_set_type(req, type);
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    dongle_status_t st;
    dongle_state_get_status(&st);
    int auto_off = 0;
    if (st.webui_auto_off_deadline_us > 0) {
        int64_t remain = st.webui_auto_off_deadline_us - esp_timer_get_time();
        auto_off = remain > 0 ? (int)(remain / 1000000) : 0;
    }
    char battery_value[4];
    const char *battery_json = "null";
    if (st.battery_percent >= 0) {
        int pct = st.battery_percent > 100 ? 100 : st.battery_percent;
        snprintf(battery_value, sizeof(battery_value), "%u", (unsigned)pct);
        battery_json = battery_value;
    } else {
        battery_json = "null";
    }
    char json[1300];
    snprintf(json, sizeof(json),
             "{\"firmware_version\":\"%s\",\"build_date\":\"%s\",\"uptime_ms\":%lld,"
             "\"state\":\"%s\",\"webui_active\":%s,\"webui_auto_off_seconds\":%d,"
             "\"webui_clients\":%d,\"ble_connected\":%s,\"connected_count\":%d,\"controller_slots\":%u,"
             "\"pairing_mode\":%s,\"controller_name\":\"%s\","
             "\"controller_address\":\"%s\",\"battery_percent\":%s,\"usb_configured\":%s,"
             "\"usb_suspended\":%s,\"usb_remote_wakeup_enabled\":%s,"
             "\"last_wake_attempt_us\":%lld,\"last_wake_attempt_allowed\":%s,\"last_error\":\"%s\"}",
             st.firmware_version, st.build_date, esp_timer_get_time() / 1000,
             dongle_state_name(st.state), st.webui_active ? "true" : "false", auto_off,
             s_sta_count, st.ble_connected ? "true" : "false", st.connected_count,
             (unsigned)DONGLE_MAX_CONTROLLERS, st.pairing_mode ? "true" : "false",
             st.controller_name, st.controller_address, battery_json,
             st.usb_configured ? "true" : "false", st.usb_suspended ? "true" : "false",
             st.usb_remote_wakeup_enabled ? "true" : "false",
             st.last_wake_attempt_us, st.last_wake_attempt_allowed ? "true" : "false",
             st.last_error);
    return send_text(req, json, "application/json");
}

static esp_err_t buttons_handler(httpd_req_t *req)
{
    dongle_status_t st;
    dongle_state_get_status(&st);
    char names[256];
    stadia_pressed_names(&st.stadia, names, sizeof(names));
    char raw[3 * DONGLE_MAX_RAW_REPORT_SIZE + 1] = {0};
    for (size_t i = 0; i < st.stadia.raw_len && i < DONGLE_MAX_RAW_REPORT_SIZE; i++) {
        snprintf(raw + strlen(raw), sizeof(raw) - strlen(raw), "%02X%s", st.stadia.raw[i],
                 i + 1 == st.stadia.raw_len ? "" : " ");
    }
    char json[600];
    snprintf(json, sizeof(json),
             "{\"pressed\":\"%s\",\"raw\":\"%s\",\"raw_len\":%u,"
             "\"lt\":%u,\"rt\":%u,\"lx\":%d,\"ly\":%d,\"rx\":%d,\"ry\":%d,"
             "\"assistant_known\":true,\"capture_known\":true}",
             names, raw, (unsigned)st.stadia.raw_len, st.stadia.lt, st.stadia.rt,
             st.stadia.lx, st.stadia.ly, st.stadia.rx, st.stadia.ry);
    return send_text(req, json, "application/json");
}

static size_t append_live_controller_json(char *json, size_t json_len, size_t off,
                                          const dongle_controller_status_t *c,
                                          bool first)
{
    char names[256];
    char raw[3 * DONGLE_MAX_RAW_REPORT_SIZE + 1] = {0};
    stadia_pressed_names(&c->stadia, names, sizeof(names));
    for (size_t i = 0; i < c->stadia.raw_len && i < DONGLE_MAX_RAW_REPORT_SIZE; i++) {
        snprintf(raw + strlen(raw), sizeof(raw) - strlen(raw), "%02X%s", c->stadia.raw[i],
                 i + 1 == c->stadia.raw_len ? "" : " ");
    }

    off += snprintf(json + off, json_len - off,
                    "%s{\"slot_index\":%u,\"usb_gamepad_index\":%u,"
                    "\"address\":\"%s\",\"name\":\"%s\",\"bonded\":%s,"
                    "\"connected\":%s,\"ready\":%s,\"battery_percent\":",
                    first ? "" : ",", c->slot_index, c->usb_gamepad_index,
                    c->controller_address, c->controller_name,
                    c->bonded ? "true" : "false",
                    c->connected ? "true" : "false", c->ready ? "true" : "false");
    if (c->battery_percent >= 0) {
        off += snprintf(json + off, json_len - off, "%u", (unsigned)c->battery_percent);
    } else {
        off += snprintf(json + off, json_len - off, "null");
    }
    off += snprintf(json + off, json_len - off,
                    ",\"pressed\":\"%s\",\"raw\":\"%s\",\"raw_len\":%u,"
                    "\"lt\":%u,\"rt\":%u,\"lx\":%d,\"ly\":%d,\"rx\":%d,\"ry\":%d,"
                    "\"mouse_mode\":%s}",
                    names, raw, (unsigned)c->stadia.raw_len,
                    c->stadia.lt, c->stadia.rt, c->stadia.lx, c->stadia.ly,
                    c->stadia.rx, c->stadia.ry,
                    c->mouse_mode ? "true" : "false");
    return off;
}

static esp_err_t controllers_handler(httpd_req_t *req)
{
    controller_info_t list[CONTROLLER_MANAGER_MAX_CONTROLLERS];
    int count = controller_manager_list(list, CONTROLLER_MANAGER_MAX_CONTROLLERS);
    dongle_status_t st;
    dongle_state_get_status(&st);
    static char json[4800];
    size_t off = snprintf(json, sizeof(json),
                          "{\"mouse_mode_0\":%s,\"mouse_mode_1\":%s,\"controllers\":[",
                          st.controllers[0].mouse_mode ? "true" : "false",
                          st.controllers[1].mouse_mode ? "true" : "false");
    bool first = true;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS && off < sizeof(json); i++) {
        const dongle_controller_status_t *c = &st.controllers[i];
        if (!c->used && !c->connected) continue;
        off = append_live_controller_json(json, sizeof(json), off, c, first);
        first = false;
    }
    for (int i = 0; i < count && off < sizeof(json); i++) {
        bool already = false;
        for (uint8_t j = 0; j < DONGLE_MAX_CONTROLLERS; j++) {
            const dongle_controller_status_t *c = &st.controllers[j];
            if (c->used && strcasecmp(c->controller_address, list[i].address) == 0) {
                already = true;
                break;
            }
        }
        if (already) continue;
        off += snprintf(json + off, sizeof(json) - off,
                        "%s{\"slot_index\":-1,\"usb_gamepad_index\":-1,"
                        "\"address\":\"%s\",\"name\":\"%s\",\"bonded\":%s,"
                        "\"connected\":false,\"ready\":false,\"battery_percent\":null,"
                        "\"pressed\":\"\",\"raw\":\"\",\"raw_len\":0,"
                        "\"lt\":0,\"rt\":0,\"lx\":0,\"ly\":0,\"rx\":0,\"ry\":0,"
                        "\"mouse_mode\":false}",
                        first ? "" : ",", list[i].address, list[i].name,
                        list[i].bonded ? "true" : "false");
        first = false;
    }
    if (off < sizeof(json)) {
        snprintf(json + off, sizeof(json) - off, "]}");
    }
    return send_text(req, json, "application/json");
}

static esp_err_t simple_ok(httpd_req_t *req)
{
    return send_text(req, "{\"ok\":true}", "application/json");
}

static esp_err_t pairing_start_handler(httpd_req_t *req)
{
    controller_manager_start_pairing(120000);
    ble_central_start_scan();
    return simple_ok(req);
}

static esp_err_t pairing_stop_handler(httpd_req_t *req)
{
    controller_manager_stop_pairing();
    dongle_state_set(DONGLE_STATE_NO_BOND_SETUP);
    return simple_ok(req);
}

static esp_err_t forget_current_handler(httpd_req_t *req)
{
    controller_manager_forget_current();
    return simple_ok(req);
}

static esp_err_t forget_one_handler(httpd_req_t *req)
{
    char body[128] = {0};
    int got = httpd_req_recv(req, body, sizeof(body) - 1);
    if (got > 0) {
        char address[32];
        parse_form_value(body, "address", address, sizeof(address));
        controller_manager_forget_address(address);
    }
    return simple_ok(req);
}

static esp_err_t forget_all_handler(httpd_req_t *req)
{
    controller_manager_forget_all();
    return simple_ok(req);
}

static esp_err_t keymap_get_handler(httpd_req_t *req)
{
    dongle_config_t cfg;
    config_store_get(&cfg);
    char json[420];
    snprintf(json, sizeof(json),
             "{\"assistant_short\":\"%s\",\"assistant_long\":\"%s\","
             "\"capture_short\":\"%s\",\"capture_long\":\"%s\","
             "\"long_press_ms\":%u,\"webui_timeout_seconds\":%u,"
             "\"disable_ap_on_suspend\":%s}",
             config_store_action_name(cfg.assistant_short_action),
             config_store_action_name(cfg.assistant_long_action),
             config_store_action_name(cfg.capture_short_action),
             config_store_action_name(cfg.capture_long_action),
             cfg.long_press_ms,
             (unsigned)(cfg.webui_timeout_after_ble_connected_ms / 1000),
             cfg.disable_ap_on_usb_suspend ? "true" : "false");
    return send_text(req, json, "application/json");
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void parse_form_value(const char *body, const char *key, char *out, size_t out_len)
{
    out[0] = '\0';
    const char *p = strstr(body, key);
    if (!p) return;
    p += strlen(key);
    if (*p != '=') return;
    p++;
    size_t n = 0;
    while (p[n] && p[n] != '&' && n + 1 < out_len) {
        if (p[n] == '%' && p[n + 1] && p[n + 2]) {
            int hi = hex_nibble(p[n + 1]);
            int lo = hex_nibble(p[n + 2]);
            if (hi >= 0 && lo >= 0) {
                *out++ = (char)((hi << 4) | lo);
                out_len--;
                p += 3;
                n = 0;
                continue;
            }
        }
        *out++ = p[n] == '+' ? ' ' : p[n];
        out_len--;
        p++;
    }
    *out = '\0';
}

static esp_err_t keymap_post_handler(httpd_req_t *req)
{
    char body[384] = {0};
    int got = httpd_req_recv(req, body, sizeof(body) - 1);
    if (got < 0) return ESP_FAIL;
    dongle_config_t cfg;
    config_store_get(&cfg);
    char v[40];
    parse_form_value(body, "assistant_short", v, sizeof(v));
    cfg.assistant_short_action = config_store_action_from_name(v);
    parse_form_value(body, "assistant_long", v, sizeof(v));
    cfg.assistant_long_action = config_store_action_from_name(v);
    parse_form_value(body, "capture_short", v, sizeof(v));
    cfg.capture_short_action = config_store_action_from_name(v);
    parse_form_value(body, "capture_long", v, sizeof(v));
    cfg.capture_long_action = config_store_action_from_name(v);
    parse_form_value(body, "long_press_ms", v, sizeof(v));
    if (v[0]) {
        unsigned ms = (unsigned)strtoul(v, NULL, 10);
        if (ms < 300) ms = 300;
        if (ms > 5000) ms = 5000;
        cfg.long_press_ms = (uint16_t)ms;
    }
    parse_form_value(body, "webui_timeout_seconds", v, sizeof(v));
    if (v[0]) {
        unsigned seconds = (unsigned)strtoul(v, NULL, 10);
        if (seconds < 15) seconds = 15;
        if (seconds > 1800) seconds = 1800;
        cfg.webui_timeout_after_ble_connected_ms = seconds * 1000U;
    }
    parse_form_value(body, "disable_ap_on_suspend", v, sizeof(v));
    if (v[0]) {
        cfg.disable_ap_on_usb_suspend = (strcmp(v, "on") == 0 || strcmp(v, "true") == 0 || strcmp(v, "1") == 0);
    } else {
        cfg.disable_ap_on_usb_suspend = false;
    }
    config_store_set(&cfg);
    update_webui_deadline();
    return keymap_get_handler(req);
}

static esp_err_t disable_handler(httpd_req_t *req)
{
    simple_ok(req);
    web_server_stop();
    return ESP_OK;
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    simple_ok(req);
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    return ESP_OK;
}

static esp_err_t update_handler(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) return send_text(req, "No OTA partition available", "text/plain");
    dongle_state_set(DONGLE_STATE_OTA_UPDATE);
    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &handle);
    if (err != ESP_OK) {
        dongle_state_set(DONGLE_STATE_ERROR);
        return send_text(req, "OTA begin failed", "text/plain");
    }
    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? sizeof(buf) : remaining);
        if (r <= 0) {
            esp_ota_abort(handle);
            dongle_state_set(DONGLE_STATE_ERROR);
            return send_text(req, "Upload failed", "text/plain");
        }
        err = esp_ota_write(handle, buf, r);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            dongle_state_set(DONGLE_STATE_ERROR);
            return send_text(req, "OTA write failed", "text/plain");
        }
        remaining -= r;
    }
    err = esp_ota_end(handle);
    if (err == ESP_OK) err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        dongle_state_set(DONGLE_STATE_ERROR);
        return send_text(req, "OTA validation failed", "text/plain");
    }
    send_text(req, "Update OK. Rebooting.", "text/plain");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

static void register_uri(const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t h = { .uri = uri, .method = method, .handler = handler, .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd, &h);
}

static bool any_controller_connected(void)
{
    dongle_status_t st;
    dongle_state_get_status(&st);
    return st.connected_count > 0 || st.ble_connected;
}

static void update_webui_deadline(void)
{
    if (!s_httpd) {
        dongle_state_set_webui(false, 0);
        return;
    }

    if (!any_controller_connected()) {
        dongle_state_set_webui(true, 0);
        return;
    }

    dongle_config_t cfg;
    config_store_get(&cfg);
    uint32_t timeout_ms = cfg.webui_timeout_after_ble_connected_ms;
    if (timeout_ms < 15000) timeout_ms = 15000;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    dongle_state_set_webui(true, deadline);
}

static void web_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_start_requested) {
            bool explicit_request = s_start_explicit;
            s_start_requested = false;
            s_start_explicit = false;
            web_server_start(explicit_request);
        }
        if (s_dhcp_ensure_requested) {
            s_dhcp_ensure_requested = false;
            ensure_ap_dhcp_running();
        }
        if (s_ble_scan_pause_requested) {
            s_ble_scan_pause_requested = false;
            ble_central_set_scan_paused(true);
        }
        if (s_ble_scan_resume_requested) {
            s_ble_scan_resume_requested = false;
            ble_central_set_scan_paused(false);
        }
        if (s_sta_count > 0 && !s_dhcp_wait_logged && s_last_sta_join_us > s_last_ip_assigned_us &&
            esp_timer_get_time() - s_last_sta_join_us > 10000000) {
            s_dhcp_wait_logged = true;
            log_dhcp_wait();
        }
        dongle_config_t cfg;
        config_store_get(&cfg);
        if (cfg.disable_ap_on_usb_suspend) {
            if (s_usb_suspended && s_httpd && !s_ap_stopped_for_suspend) {
                ESP_LOGI("WEB", "Stopping Web UI/AP due to USB suspend");
                web_server_stop();
                s_ap_stopped_for_suspend = true;
            } else if (!s_usb_suspended && s_ap_stopped_for_suspend) {
                ESP_LOGI("WEB", "Restarting Web UI/AP after USB resume");
                web_server_request_start(false);
                s_ap_stopped_for_suspend = false;
            }
        }
        dongle_status_t st;
        dongle_state_get_status(&st);
        if (s_httpd && st.webui_auto_off_deadline_us > 0 &&
            esp_timer_get_time() >= st.webui_auto_off_deadline_us &&
            s_sta_count == 0 && st.state != DONGLE_STATE_OTA_UPDATE) {
            ESP_LOGI("WEB", "Stopping Web UI/AP after timeout");
            web_server_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void web_server_init(void)
{
    xTaskCreate(web_monitor_task, "web_monitor", 3072, NULL, 1, NULL);
}

void web_server_start(bool explicit_request)
{
    (void)explicit_request;
    if (s_httpd) {
        update_webui_deadline();
        return;
    }

    if (!s_netif_created) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        s_netif_created = true;
    }
    if (!s_wifi_started) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
        if (!s_wifi_event_registered) {
            ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                       wifi_event_handler, NULL));
            s_wifi_event_registered = true;
        }
        if (!s_ip_event_registered) {
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                                       ip_event_handler, NULL));
            s_ip_event_registered = true;
        }
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        wifi_config_t ap = {0};
        snprintf((char *)ap.ap.ssid, sizeof(ap.ap.ssid), "StadiaDongle-%02X%02X", mac[4], mac[5]);
        ap.ap.ssid_len = strlen((char *)ap.ap.ssid);
        ap.ap.channel = 6;
        ap.ap.max_connection = 1;
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ap.ap.ssid_hidden = 0;
        ap.ap.beacon_interval = 100;
        ap.ap.pmf_cfg.capable = false;
        ap.ap.pmf_cfg.required = false;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        /*
         * Use ESP-IDF's default AP protocol set. Forcing 11b/g made some
         * clients associate but never complete DHCP.
        */
        ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
        ESP_ERROR_CHECK(esp_wifi_start());
        vTaskDelay(pdMS_TO_TICKS(50));
        ensure_ap_dhcp_running();
        esp_wifi_set_inactive_time(WIFI_IF_AP, 300);
        esp_err_t txp_err = esp_wifi_set_max_tx_power(78);
        if (txp_err != ESP_OK) {
            ESP_LOGW("WEB", "Could not set AP TX power: %s", esp_err_to_name(txp_err));
        }
        ESP_LOGI("WEB", "Setup AP started: SSID=%s auth=open url=http://192.168.4.1", ap.ap.ssid);
        s_wifi_started = true;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 6144;
    cfg.max_uri_handlers = 18;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &cfg));
    register_uri("/", HTTP_GET, index_handler);
    register_uri("/api/status", HTTP_GET, status_handler);
    register_uri("/api/buttons", HTTP_GET, buttons_handler);
    register_uri("/api/battery", HTTP_GET, status_handler);
    register_uri("/api/controllers", HTTP_GET, controllers_handler);
    register_uri("/api/controllers/forget", HTTP_POST, forget_one_handler);
    register_uri("/api/pairing/start", HTTP_POST, pairing_start_handler);
    register_uri("/api/pairing/stop", HTTP_POST, pairing_stop_handler);
    register_uri("/api/controllers/forget-current", HTTP_POST, forget_current_handler);
    register_uri("/api/controllers/forget-all", HTTP_POST, forget_all_handler);
    register_uri("/api/config/keymap", HTTP_GET, keymap_get_handler);
    register_uri("/api/config/keymap", HTTP_POST, keymap_post_handler);
    register_uri("/api/webui/disable", HTTP_POST, disable_handler);
    register_uri("/api/reboot", HTTP_POST, reboot_handler);
    register_uri("/api/update", HTTP_POST, update_handler);
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, not_found_handler);
    captive_portal_start();

    update_webui_deadline();
}

void web_server_request_start(bool explicit_request)
{
    s_start_explicit = s_start_explicit || explicit_request;
    s_start_requested = true;
}

void web_server_stop(void)
{
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    captive_portal_stop();
    s_start_requested = false;
    s_start_explicit = false;
    s_sta_count = 0;
    s_dhcp_wait_logged = false;
    s_dhcp_ensure_requested = false;
    s_ble_scan_pause_requested = false;
    s_ble_scan_resume_requested = true;
    if (s_wifi_started) {
        if (s_ap_netif) esp_netif_dhcps_stop(s_ap_netif);
        esp_wifi_stop();
        esp_wifi_deinit();
        esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
        s_wifi_started = false;
    }
    dongle_state_set_webui(false, 0);
}

void web_server_notify_usb_suspend(bool suspended)
{
    s_usb_suspended = suspended;
}

bool web_server_is_active(void)
{
    return s_httpd != NULL;
}

bool web_server_is_active_or_requested(void)
{
    return s_httpd != NULL || s_start_requested;
}

bool web_server_has_clients(void)
{
    return s_sta_count > 0;
}
