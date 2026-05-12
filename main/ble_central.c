/*
 * ble_central.c - Slot-based NimBLE central for concurrent Stadia
 * controllers. Each slot owns its BLE handles, parsed state, USB gamepad index,
 * battery handle, and rumble target.
 */

#include "ble_central.h"

#include "bridge.h"
#include "button_actions.h"
#include "controller_manager.h"
#include "dongle_config.h"
#include "dongle_state.h"
#include "mouse_mode.h"
#include "web_server.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/hci_common.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/ble_store.h"
#include "os/os_mbuf.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "BLE";

static bool s_nimble_synced;

#define MAX_REPORT_CHRS 8
#define BLE_SCAN_RETRY_US 1000000
#define BLE_RESTART_SCAN_US 1000000
#define BLE_BOOT_SCAN_DELAY_US 2000000
#define BLE_BATTERY_POLL_US 60000000

typedef enum {
    CONTROLLER_SLOT_EMPTY,
    CONTROLLER_SLOT_CONNECTING,
    CONTROLLER_SLOT_DISCOVERING,
    CONTROLLER_SLOT_ENCRYPTING,
    CONTROLLER_SLOT_READY,
    CONTROLLER_SLOT_DISCONNECTING,
} controller_slot_state_t;

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
} report_chr_t;

typedef struct {
    bool used;
    uint8_t slot_index;
    uint32_t generation;

    ble_addr_t peer_addr;
    char addr_str[DONGLE_ADDR_STR_LEN];
    char name[DONGLE_CONTROLLER_NAME_LEN];
    uint16_t conn_handle;

    uint16_t hid_svc_start;
    uint16_t hid_svc_end;
    uint16_t input_val_handle;
    uint16_t output_val_handle;
    uint16_t input_cccd_handle;

    report_chr_t report_chrs[MAX_REPORT_CHRS];
    int chr_count;
    int cur_chr;
    uint16_t cur_2908;
    uint16_t cur_2902;

    uint16_t battery_svc_start;
    uint16_t battery_svc_end;
    uint16_t battery_val_handle;
    int battery_percent;
    bool battery_available;

    stadia_controller_state_t stadia_state;
    uint8_t usb_gamepad_index;
    bool last_report_valid;
    uint8_t last_xbox_report[DONGLE_XBOX_REPORT_SIZE];

    controller_slot_state_t state;
    struct ble_npl_callout rumble_callout;
    uint8_t rumble_payload[DONGLE_RUMBLE_REPORT_SIZE];
} controller_slot_t;

static controller_slot_t s_slots[DONGLE_MAX_CONTROLLERS];
static SemaphoreHandle_t s_slot_lock;
static SemaphoreHandle_t s_scan_lock;
static esp_timer_handle_t s_reconnect_timer;
static esp_timer_handle_t s_battery_timer;
static bool s_force_active_scan;
static bool s_scan_paused;

static void start_scan(void);
static int gap_event_fn(struct ble_gap_event *event, void *arg);
static void process_next_chr(controller_slot_t *slot);
static void rumble_callout_fn(struct ble_npl_event *ev);
static int battery_read_cb(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg);

static void addr_to_str(const ble_addr_t *addr, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

static void init_slot_callout(controller_slot_t *slot)
{
    if (!slot) return;
    ble_npl_callout_init(&slot->rumble_callout,
                         nimble_port_get_dflt_eventq(),
                         rumble_callout_fn, slot);
}

static bool addr_equal(const ble_addr_t *a, const ble_addr_t *b)
{
    return a && b && a->type == b->type && memcmp(a->val, b->val, sizeof(a->val)) == 0;
}

static controller_slot_t *slot_by_conn(uint16_t conn_handle)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_slots[i].used && s_slots[i].conn_handle == conn_handle) return &s_slots[i];
    }
    return NULL;
}

static controller_slot_t *slot_by_addr(const ble_addr_t *addr)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_slots[i].used && addr_equal(&s_slots[i].peer_addr, addr)) return &s_slots[i];
    }
    return NULL;
}

static controller_slot_t *slot_by_usb(uint8_t usb_gamepad_index)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_slots[i].used && s_slots[i].usb_gamepad_index == usb_gamepad_index) return &s_slots[i];
    }
    return NULL;
}

static bool slot_valid(controller_slot_t *slot, uint16_t conn_handle)
{
    return slot && slot->used && slot->conn_handle == conn_handle;
}

static int occupied_slots(void)
{
    int count = 0;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_slots[i].used) count++;
    }
    return count;
}

static bool have_capacity(void)
{
    return occupied_slots() < DONGLE_MAX_CONTROLLERS;
}

static controller_slot_t *alloc_slot(const ble_addr_t *addr)
{
    if (s_slot_lock) xSemaphoreTake(s_slot_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (!s_slots[i].used) {
            controller_slot_t *slot = &s_slots[i];
            uint32_t generation = slot->generation + 1;
            memset(slot, 0, sizeof(*slot));
            init_slot_callout(slot);
            slot->used = true;
            slot->slot_index = i;
            slot->generation = generation;
            slot->peer_addr = *addr;
            slot->conn_handle = BLE_HS_CONN_HANDLE_NONE;
            slot->usb_gamepad_index = i;
            slot->battery_percent = -1;
            slot->state = CONTROLLER_SLOT_CONNECTING;
            addr_to_str(addr, slot->addr_str, sizeof(slot->addr_str));
            snprintf(slot->name, sizeof(slot->name), "Stadia Controller");
            dongle_state_controller_set_info(i, i, slot->addr_str, slot->name, false);
            ESP_LOGI(TAG, "slot %u usb %u allocated addr=%s", i, i, slot->addr_str);
            if (s_slot_lock) xSemaphoreGive(s_slot_lock);
            return slot;
        }
    }
    if (s_slot_lock) xSemaphoreGive(s_slot_lock);
    return NULL;
}

static void clear_slot(controller_slot_t *slot)
{
    if (!slot) return;
    if (s_slot_lock) xSemaphoreTake(s_slot_lock, portMAX_DELAY);
    uint8_t idx = slot->slot_index;
    uint8_t usb = slot->usb_gamepad_index;
    uint32_t generation = slot->generation + 1;
    ble_npl_callout_stop(&slot->rumble_callout);
    memset(slot, 0, sizeof(*slot));
    init_slot_callout(slot);
    slot->slot_index = idx;
    slot->usb_gamepad_index = usb;
    slot->generation = generation;
    slot->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    slot->battery_percent = -1;
    dongle_state_controller_clear(idx);
    mouse_mode_reset(usb);
    bridge_send_neutral(usb);
    if (s_slot_lock) xSemaphoreGive(s_slot_lock);
    ESP_LOGI(TAG, "slot %u usb %u cleared", idx, usb);
}

static void update_global_state_after_slot_change(void)
{
    int ready = 0;
    int used = 0;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_slots[i].used) {
            used++;
            if (s_slots[i].state == CONTROLLER_SLOT_READY) ready++;
        }
    }
    if (ready > 0) {
        dongle_state_set(web_server_is_active() ? DONGLE_STATE_CONNECTED_WEBUI_ACTIVE : DONGLE_STATE_CONNECTED);
    } else if (used > 0) {
        dongle_state_set(DONGLE_STATE_CONNECTING);
    } else if (controller_manager_bond_count() == 0) {
        dongle_state_set(DONGLE_STATE_NO_BOND_SETUP);
    } else {
        dongle_state_set(DONGLE_STATE_SCANNING);
    }
}

static void reconnect_timer_cb(void *arg)
{
    (void)arg;
    start_scan();
}

static void battery_timer_cb(void *arg)
{
    (void)arg;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        controller_slot_t *slot = &s_slots[i];
        if (slot->used && slot->state == CONTROLLER_SLOT_READY && slot->battery_val_handle) {
            int rc = ble_gattc_read(slot->conn_handle, slot->battery_val_handle, battery_read_cb, slot);
            if (rc != 0 && rc != BLE_HS_EBUSY) {
                ESP_LOGW(TAG, "slot %u battery poll start failed: %d", slot->slot_index, rc);
            }
        }
    }
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset: reason=%d", reason);
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE synced");
    s_nimble_synced = true;
    controller_manager_init();
    web_server_request_start(false);
    if (controller_manager_bond_count() == 0) {
        dongle_state_set(DONGLE_STATE_NO_BOND_SETUP);
        ESP_LOGI(TAG, "No bonded controllers; auto-scanning for any Stadia device");
        ble_central_start_scan();
        return;
    }
    ESP_LOGI(TAG, "Starting bonded-controller scan in %u ms", (unsigned)(BLE_BOOT_SCAN_DELAY_US / 1000));
    if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
    if (s_reconnect_timer) esp_timer_start_once(s_reconnect_timer, BLE_BOOT_SCAN_DELAY_US);
}

static bool adv_contains_stadia(const uint8_t *data, uint8_t len)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, data, len) != 0) return false;
    return fields.name && fields.name_len >= 6 && memcmp(fields.name, "Stadia", 6) == 0;
}

static void start_scan(void)
{
    ble_gap_disc_cancel();
    if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    if (s_scan_paused) {
        if (s_scan_lock) xSemaphoreGive(s_scan_lock);
        ESP_LOGI(TAG, "Not scanning: BLE discovery paused for Wi-Fi AP client");
        return;
    }
    if (!have_capacity()) {
        if (s_scan_lock) xSemaphoreGive(s_scan_lock);
        ESP_LOGI(TAG, "Not scanning: controller capacity full (%u)", (unsigned)DONGLE_MAX_CONTROLLERS);
        return;
    }

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        if (s_scan_lock) xSemaphoreGive(s_scan_lock);
        ESP_LOGW(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    bool force_active = s_force_active_scan;
    s_force_active_scan = false;
    if (s_scan_lock) xSemaphoreGive(s_scan_lock);

    bool web_active = web_server_is_active_or_requested();
    bool pairing = controller_manager_is_pairing_mode();
    struct ble_gap_disc_params params = {
        .itvl = web_active && !pairing && !force_active ? 800 : 16,
        .window = web_active && !pairing && !force_active ? 16 : 16,
        .passive = web_active && !pairing && !force_active,
        .filter_duplicates = 0,
    };
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_fn, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc failed: %d; retrying", rc);
        if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
        if (s_reconnect_timer) esp_timer_start_once(s_reconnect_timer, BLE_SCAN_RETRY_US);
        return;
    }
    dongle_state_set(DONGLE_STATE_SCANNING);
    ESP_LOGI(TAG, "Scanning capacity=%d/%u mode=%s itvl=%u window=%u",
             occupied_slots(), (unsigned)DONGLE_MAX_CONTROLLERS,
             params.passive ? "passive" : "active", params.itvl, params.window);
}

void ble_central_start_scan(void)
{
    if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    s_scan_paused = false;
    s_force_active_scan = true;
    if (s_scan_lock) xSemaphoreGive(s_scan_lock);
    start_scan();
}

void ble_central_set_scan_paused(bool paused)
{
    if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    if (s_scan_paused == paused) {
        if (s_scan_lock) xSemaphoreGive(s_scan_lock);
        return;
    }
    s_scan_paused = paused;
    if (s_scan_lock) xSemaphoreGive(s_scan_lock);
    if (paused) {
        ble_gap_disc_cancel();
        if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
        ESP_LOGI(TAG, "BLE discovery paused for Wi-Fi AP client");
    } else {
        ESP_LOGI(TAG, "BLE discovery resumed after Wi-Fi AP client");
        if (s_nimble_synced) ble_central_start_scan();
    }
}

static int cccd_write_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg);

static void subscribe(controller_slot_t *slot)
{
    if (!slot_valid(slot, slot->conn_handle)) return;
    slot->state = CONTROLLER_SLOT_ENCRYPTING;
    ESP_LOGI(TAG, "slot %u usb %u initiating security conn=%u in=0x%04x cccd=0x%04x out=0x%04x",
             slot->slot_index, slot->usb_gamepad_index, slot->conn_handle,
             slot->input_val_handle, slot->input_cccd_handle, slot->output_val_handle);
    int rc = ble_gap_security_initiate(slot->conn_handle);
    if (rc == 0) return;

    if (rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "slot %u security initiate failed: %d; trying CCCD anyway", slot->slot_index, rc);
    }
    uint8_t val[2] = {0x01, 0x00};
    ble_gattc_write_flat(slot->conn_handle, slot->input_cccd_handle,
                         val, sizeof(val), cccd_write_fn, slot);
}

static int svc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg);
static int chr_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg);
static int dsc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
static int report_ref_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg);

static int svc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg)
{
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == BLE_HS_EDONE) {
        if (!slot->hid_svc_start) {
            ESP_LOGE(TAG, "slot %u HID service not found", slot->slot_index);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        ESP_LOGI(TAG, "slot %u HID service 0x%04x-0x%04x",
                 slot->slot_index, slot->hid_svc_start, slot->hid_svc_end);
        slot->chr_count = 0;
        ble_gattc_disc_all_chrs(conn_handle, slot->hid_svc_start, slot->hid_svc_end,
                                chr_disc_fn, slot);
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "slot %u svc disc error: %d", slot->slot_index, err->status);
        return 0;
    }
    slot->hid_svc_start = svc->start_handle;
    slot->hid_svc_end = svc->end_handle;
    return 0;
}

static int chr_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "slot %u found %d report characteristics", slot->slot_index, slot->chr_count);
        slot->cur_chr = 0;
        process_next_chr(slot);
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "slot %u chr disc error: %d", slot->slot_index, err->status);
        return 0;
    }
    if (chr->uuid.u.type == BLE_UUID_TYPE_16 &&
        chr->uuid.u16.value == 0x2A4D &&
        slot->chr_count < MAX_REPORT_CHRS) {
        slot->report_chrs[slot->chr_count].def_handle = chr->def_handle;
        slot->report_chrs[slot->chr_count].val_handle = chr->val_handle;
        slot->chr_count++;
    }
    return 0;
}

static void process_next_chr(controller_slot_t *slot)
{
    if (!slot_valid(slot, slot->conn_handle)) return;
    if (slot->cur_chr >= slot->chr_count) {
        if (!slot->input_val_handle || !slot->output_val_handle || !slot->input_cccd_handle) {
            ESP_LOGE(TAG, "slot %u incomplete handles in=0x%04x out=0x%04x cccd=0x%04x",
                     slot->slot_index, slot->input_val_handle,
                     slot->output_val_handle, slot->input_cccd_handle);
            ble_gap_terminate(slot->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return;
        }
        subscribe(slot);
        return;
    }

    uint16_t start = slot->report_chrs[slot->cur_chr].val_handle + 1;
    uint16_t end = (slot->cur_chr + 1 < slot->chr_count)
                       ? (slot->report_chrs[slot->cur_chr + 1].def_handle - 1)
                       : slot->hid_svc_end;
    if (start > end) {
        slot->cur_chr++;
        process_next_chr(slot);
        return;
    }
    slot->cur_2908 = 0;
    slot->cur_2902 = 0;
    ble_gattc_disc_all_dscs(slot->conn_handle, slot->report_chrs[slot->cur_chr].val_handle,
                            end, dsc_disc_fn, slot);
}

static int dsc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == BLE_HS_EDONE) {
        if (slot->cur_2908) {
            ble_gattc_read(conn_handle, slot->cur_2908, report_ref_fn, slot);
        } else {
            slot->cur_chr++;
            process_next_chr(slot);
        }
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "slot %u dsc disc error: %d", slot->slot_index, err->status);
        slot->cur_chr++;
        process_next_chr(slot);
        return 0;
    }
    if (dsc->uuid.u.type == BLE_UUID_TYPE_16) {
        if (dsc->uuid.u16.value == 0x2908) slot->cur_2908 = dsc->handle;
        if (dsc->uuid.u16.value == 0x2902) slot->cur_2902 = dsc->handle;
    }
    return 0;
}

static int report_ref_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg)
{
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == 0 && attr) {
        uint8_t buf[2];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), &len) == 0 && len == 2) {
            uint8_t report_id = buf[0];
            uint8_t report_type = buf[1];
            ESP_LOGI(TAG, "slot %u chr[%d] val=0x%04x id=%u type=%u",
                     slot->slot_index, slot->cur_chr,
                     slot->report_chrs[slot->cur_chr].val_handle, report_id, report_type);
            if (report_type == 1) {
                slot->input_val_handle = slot->report_chrs[slot->cur_chr].val_handle;
                slot->input_cccd_handle = slot->cur_2902;
            } else if (report_type == 2) {
                slot->output_val_handle = slot->report_chrs[slot->cur_chr].val_handle;
            }
        }
    }
    slot->cur_chr++;
    process_next_chr(slot);
    return 0;
}

static int battery_read_cb(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg)
{
    controller_slot_t *slot = arg ? arg : slot_by_conn(conn_handle);
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == 0 && attr && attr->om && OS_MBUF_PKTLEN(attr->om) >= 1) {
        uint8_t pct = 0;
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(attr->om, &pct, sizeof(pct), &len) == 0 && len >= 1) {
            if (pct > 100) pct = 100;
            slot->battery_percent = pct;
            slot->battery_available = true;
            dongle_state_controller_set_battery(slot->slot_index, pct);
            ESP_LOGI(TAG, "slot %u battery level: %u%%", slot->slot_index, pct);
        }
    } else if (err->status != 0) {
        ESP_LOGW(TAG, "slot %u battery read failed: %d", slot->slot_index, err->status);
    }
    return 0;
}

static int battery_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *err,
                          const struct ble_gatt_chr *chr, void *arg)
{
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == 0 && chr) {
        slot->battery_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "slot %u battery characteristic 0x%04x", slot->slot_index, chr->val_handle);
        int rc = ble_gattc_read(conn_handle, slot->battery_val_handle, battery_read_cb, slot);
        if (rc != 0) ESP_LOGW(TAG, "slot %u battery read start failed: %d", slot->slot_index, rc);
    } else if (err->status == BLE_HS_EDONE && !slot->battery_val_handle) {
        dongle_state_controller_set_battery(slot->slot_index, -1);
        ESP_LOGI(TAG, "slot %u battery characteristic not found", slot->slot_index);
    }
    return 0;
}

static int battery_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *err,
                          const struct ble_gatt_svc *svc, void *arg)
{
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == 0 && svc) {
        slot->battery_svc_start = svc->start_handle;
        slot->battery_svc_end = svc->end_handle;
        ble_uuid16_t uuid = BLE_UUID16_INIT(0x2A19);
        int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, svc->start_handle, svc->end_handle,
                                             &uuid.u, battery_chr_cb, slot);
        if (rc != 0) ESP_LOGW(TAG, "slot %u battery chr discovery failed to start: %d", slot->slot_index, rc);
    } else if (err->status == BLE_HS_EDONE && !slot->battery_svc_start) {
        dongle_state_controller_set_battery(slot->slot_index, -1);
        ESP_LOGI(TAG, "slot %u battery service not found", slot->slot_index);
    }
    return 0;
}

static void battery_start(controller_slot_t *slot)
{
    if (!slot_valid(slot, slot->conn_handle)) return;
    slot->battery_val_handle = 0;
    slot->battery_available = false;
    dongle_state_controller_set_battery(slot->slot_index, -1);
    ble_uuid16_t uuid = BLE_UUID16_INIT(0x180F);
    int rc = ble_gattc_disc_svc_by_uuid(slot->conn_handle, &uuid.u, battery_svc_cb, slot);
    if (rc != 0) ESP_LOGW(TAG, "slot %u battery service discovery start failed: %d", slot->slot_index, rc);
}

static int cccd_write_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    controller_slot_t *slot = arg;
    if (!slot_valid(slot, conn_handle)) return 0;
    if (err->status == 0) {
        slot->state = CONTROLLER_SLOT_READY;
        dongle_state_controller_set_connected(slot->slot_index, true, true);
        controller_manager_stop_pairing();
        web_server_request_start(false);
        update_global_state_after_slot_change();
        ESP_LOGI(TAG, "slot %u usb %u ready conn=%u addr=%s",
                 slot->slot_index, slot->usb_gamepad_index, conn_handle, slot->addr_str);
        battery_start(slot);
        if (have_capacity()) {
            if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
            s_force_active_scan = true;
            if (s_scan_lock) xSemaphoreGive(s_scan_lock);
            start_scan();
        }
    } else {
        ESP_LOGE(TAG, "slot %u CCCD write failed: %d", slot->slot_index, err->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

static void rumble_callout_fn(struct ble_npl_event *ev)
{
    controller_slot_t *slot = ble_npl_event_get_arg(ev);
    if (!slot || !slot->used || slot->state != CONTROLLER_SLOT_READY || !slot->output_val_handle) return;
    int rc = ble_gattc_write_flat(slot->conn_handle, slot->output_val_handle,
                                  slot->rumble_payload, DONGLE_RUMBLE_REPORT_SIZE, NULL, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "slot %u usb %u rumble write start failed conn=%u rc=%d",
                 slot->slot_index, slot->usb_gamepad_index, slot->conn_handle, rc);
    } else {
        ESP_LOGI(TAG, "slot %u usb %u rumble routed", slot->slot_index, slot->usb_gamepad_index);
    }
}

void ble_central_send_rumble(uint8_t gamepad_index, const uint8_t *payload, size_t len)
{
    if (!payload || len < DONGLE_RUMBLE_REPORT_SIZE) return;
    if (s_slot_lock) xSemaphoreTake(s_slot_lock, portMAX_DELAY);
    controller_slot_t *slot = slot_by_usb(gamepad_index);
    if (!slot || slot->state != CONTROLLER_SLOT_READY) {
        if (s_slot_lock) xSemaphoreGive(s_slot_lock);
        ESP_LOGW(TAG, "Ignoring rumble for disconnected usb %u", gamepad_index);
        return;
    }
    memcpy(slot->rumble_payload, payload, DONGLE_RUMBLE_REPORT_SIZE);
    ble_npl_callout_reset(&slot->rumble_callout, 0);
    if (s_slot_lock) xSemaphoreGive(s_slot_lock);
}

static bool is_bonded_addr(const ble_addr_t *addr)
{
    struct ble_store_key_sec key = {0};
    key.peer_addr = *addr;
    key.idx = 0;
    struct ble_store_value_sec val;
    return ble_store_read_peer_sec(&key, &val) == 0;
}

static int gap_event_fn(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        if (!have_capacity()) {
            ble_gap_disc_cancel();
            break;
        }
        if (slot_by_addr(&event->disc.addr)) break;

        bool is_directed = (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND);
        bool is_stadia = adv_contains_stadia(event->disc.data, event->disc.length_data);
        bool is_bonded = (!is_stadia && !is_directed) ? is_bonded_addr(&event->disc.addr) : false;
        if (!is_stadia && !is_directed && !is_bonded) break;
        if (!controller_manager_should_connect(&event->disc.addr, is_stadia, is_directed)) break;

        controller_slot_t *slot = alloc_slot(&event->disc.addr);
        if (!slot) {
            ESP_LOGW(TAG, "No free slot for discovered controller");
            break;
        }

        ble_gap_disc_cancel();
        dongle_state_set(DONGLE_STATE_CONNECTING);
        static const struct ble_gap_conn_params conn_params = {
            .scan_itvl = 16,
            .scan_window = 16,
            .itvl_min = DONGLE_MAX_CONTROLLERS > 1 ? 9 : 6,
            .itvl_max = DONGLE_MAX_CONTROLLERS > 1 ? 12 : 6,
            .latency = 0,
            .supervision_timeout = 300,
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        uint8_t own_addr_type;
        ble_hs_id_infer_auto(0, &own_addr_type);
        int rc = ble_gap_connect(own_addr_type, &event->disc.addr, 5000,
                                 &conn_params, gap_event_fn, slot);
        if (rc != 0) {
            ESP_LOGW(TAG, "slot %u connect start failed: %d", slot->slot_index, rc);
            clear_slot(slot);
            if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
            s_force_active_scan = true;
            if (s_scan_lock) xSemaphoreGive(s_scan_lock);
            start_scan();
        } else {
            ESP_LOGI(TAG, "slot %u usb %u connecting addr=%s",
                     slot->slot_index, slot->usb_gamepad_index, slot->addr_str);
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT: {
        controller_slot_t *slot = arg;
        if (!slot || !slot->used) break;
        if (event->connect.status == 0) {
            slot->conn_handle = event->connect.conn_handle;
            slot->state = CONTROLLER_SLOT_DISCOVERING;
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(slot->conn_handle, &desc) == 0) {
                slot->peer_addr = desc.peer_id_addr;
                addr_to_str(&slot->peer_addr, slot->addr_str, sizeof(slot->addr_str));
                controller_manager_note_connected(&desc.peer_id_addr, slot->name);
                dongle_state_controller_set_info(slot->slot_index, slot->usb_gamepad_index,
                                                 slot->addr_str, slot->name, is_bonded_addr(&slot->peer_addr));
            }
            dongle_state_controller_set_connected(slot->slot_index, true, false);
            ESP_LOGI(TAG, "slot %u usb %u connected addr=%s conn=%u",
                     slot->slot_index, slot->usb_gamepad_index, slot->addr_str, slot->conn_handle);

            ble_uuid16_t hid_uuid = BLE_UUID16_INIT(0x1812);
            ble_gattc_disc_svc_by_uuid(slot->conn_handle, &hid_uuid.u, svc_disc_fn, slot);
        } else {
            ESP_LOGW(TAG, "slot %u connect failed: %d", slot->slot_index, event->connect.status);
            clear_slot(slot);
            if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
            s_force_active_scan = true;
            if (s_scan_lock) xSemaphoreGive(s_scan_lock);
            start_scan();
        }
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        controller_slot_t *slot = slot_by_conn(event->disconnect.conn.conn_handle);
        uint8_t usb = slot ? slot->usb_gamepad_index : 0;
        ESP_LOGW(TAG, "slot %u usb %u disconnected reason=%d",
                 slot ? slot->slot_index : 255, usb, event->disconnect.reason);
        if (slot) {
            slot->state = CONTROLLER_SLOT_DISCONNECTING;
            bridge_send_neutral(usb);
            clear_slot(slot);
        }
        controller_manager_note_disconnected();
        update_global_state_after_slot_change();
        if (occupied_slots() == 0) {
            web_server_request_start(false);
        }
        if (s_scan_lock) xSemaphoreTake(s_scan_lock, portMAX_DELAY);
        s_force_active_scan = true;
        if (s_scan_lock) xSemaphoreGive(s_scan_lock);
        if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
        if (s_reconnect_timer) esp_timer_start_once(s_reconnect_timer, BLE_RESTART_SCAN_US);
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        controller_slot_t *slot = slot_by_conn(event->notify_rx.conn_handle);
        if (!slot || event->notify_rx.attr_handle != slot->input_val_handle) break;
        uint8_t raw[16];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, raw, sizeof(raw), &len) != 0 || len < 9) {
            ESP_LOGW(TAG, "slot %u malformed input len=%u", slot->slot_index, len);
            break;
        }
        uint8_t xbox[DONGLE_XBOX_REPORT_SIZE];
        stadia_controller_state_t state;
        stadia_parse_state(raw, len, &state);
        stadia_to_xbox360(raw, xbox);
        slot->stadia_state = state;
        memcpy(slot->last_xbox_report, xbox, sizeof(xbox));
        slot->last_report_valid = true;
        dongle_state_update_controller_reports(slot->slot_index, raw, len, &state, xbox);
        button_actions_on_state(slot->usb_gamepad_index, &state);

        if (mouse_mode_is_active(slot->usb_gamepad_index)) {
            ble_to_usb_msg_t neutral = {
                .gamepad_index = slot->usb_gamepad_index,
                .data = {0x00, 0x14},
                .len = DONGLE_XBOX_REPORT_SIZE,
            };
            if (xQueueSendToBack(ble_to_usb_queue, &neutral, 0) != pdTRUE) {
                ble_to_usb_msg_t dummy;
                xQueueReceive(ble_to_usb_queue, &dummy, 0);
                xQueueSendToBack(ble_to_usb_queue, &neutral, 0);
            }
            mouse_mode_cache_state(slot->usb_gamepad_index, &state);
        } else {
            ble_to_usb_msg_t msg = {
                .gamepad_index = slot->usb_gamepad_index,
                .len = DONGLE_XBOX_REPORT_SIZE,
            };
            memcpy(msg.data, xbox, sizeof(xbox));
            if (xQueueSendToBack(ble_to_usb_queue, &msg, 0) != pdTRUE) {
                ble_to_usb_msg_t dummy;
                xQueueReceive(ble_to_usb_queue, &dummy, 0);
                xQueueSendToBack(ble_to_usb_queue, &msg, 0);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
        controller_slot_t *slot = slot_by_conn(event->enc_change.conn_handle);
        ESP_LOGI(TAG, "slot %u encryption status=%d",
                 slot ? slot->slot_index : 255, event->enc_change.status);
        if (!slot) break;
        if (event->enc_change.status == 0 && slot->input_cccd_handle) {
            uint8_t val[2] = {0x01, 0x00};
            ble_gattc_write_flat(slot->conn_handle, slot->input_cccd_handle,
                                 val, sizeof(val), cccd_write_fn, slot);
        } else if (event->enc_change.status != 0) {
            ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        break;
    }

    default:
        break;
    }
    return 0;
}

void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_central_init(void)
{
    s_slot_lock = xSemaphoreCreateMutex();
    s_scan_lock = xSemaphoreCreateMutex();
    configASSERT(s_slot_lock != NULL);
    configASSERT(s_scan_lock != NULL);

    esp_timer_create_args_t rt = {
        .callback = reconnect_timer_cb,
        .name = "ble_reconnect",
    };
    ESP_ERROR_CHECK(esp_timer_create(&rt, &s_reconnect_timer));

    esp_timer_create_args_t bt = {
        .callback = battery_timer_cb,
        .name = "battery_poll",
    };
    ESP_ERROR_CHECK(esp_timer_create(&bt, &s_battery_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_battery_timer, BLE_BATTERY_POLL_US));

    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        s_slots[i].slot_index = i;
        s_slots[i].usb_gamepad_index = i;
        s_slots[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_slots[i].battery_percent = -1;
        ble_npl_callout_init(&s_slots[i].rumble_callout,
                             nimble_port_get_dflt_eventq(),
                             rumble_callout_fn, &s_slots[i]);
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
}
