/*
 * ble_central.c — NimBLE GATT central for the Stadia controller.
 *
 * Discovery sequence (async / callback-driven):
 *   1. Scan (active) for advertisement name containing "Stadia"
 *   2. Connect
 *   3. Discover HID service (0x1812)
 *   4. Discover all Report characteristics (0x2A4D) within the service
 *   5. For each 0x2A4D chr: discover descriptors, read 0x2908 (Report Reference)
 *      to identify:
 *        - Input chr  (Report ID 3, type 1) → g_input_val_handle + g_input_cccd_handle
 *        - Output chr (Report ID 5, type 2) → g_output_val_handle
 *   6. Write 0x0001 to CCCD of input chr to enable notifications
 *   7. On BLE_GAP_EVENT_NOTIFY_RX: translate → push to ble_to_usb_queue
 *   8. Rumble: ble_npl_callout → ble_gattc_write_flat (Write-With-Response)
 *   9. On disconnect: 1 s esp_timer → restart scan
 *
 * Pairing: Just Works, bonding, Secure Connections (no MITM).
 * Bonds are persisted via CONFIG_BT_NIMBLE_NVS_PERSIST=y.
 */

#include "ble_central.h"
#include "bridge.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"

#include <string.h>

static const char *TAG = "BLE";

/* ---- Global connection / GATT handles ------------------------------------ */

static uint16_t g_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_hid_svc_start     = 0;
static uint16_t g_hid_svc_end       = 0;
static uint16_t g_input_val_handle  = 0;
static uint16_t g_output_val_handle = 0;
static uint16_t g_input_cccd_handle = 0;

/* ---- Discovery state ----------------------------------------------------- */

#define MAX_REPORT_CHRS 8

static struct {
    uint16_t def_handle;
    uint16_t val_handle;
} s_chrs[MAX_REPORT_CHRS];

static int      s_chr_count   = 0;
static int      s_cur_chr     = 0;
static uint16_t s_cur_2908    = 0; // Report Reference descriptor handle for current chr
static uint16_t s_cur_2902    = 0; // CCCD handle for current chr

/* ---- Rumble callout (must execute on NimBLE event queue) ----------------- */

static struct ble_npl_callout s_rumble_callout;
static uint8_t                s_rumble_payload[4];

/* ---- Reconnect timer ----------------------------------------------------- */

static esp_timer_handle_t s_reconnect_timer;

/* ---- Forward declarations ------------------------------------------------ */

static void start_scan(void);
static int  gap_event_fn(struct ble_gap_event *event, void *arg);
static void process_next_chr(void);

/* ---- Reconnect timer callback -------------------------------------------- */

static void reconnect_timer_cb(void *arg)
{
    start_scan();
}

/* ---- NimBLE host callbacks ----------------------------------------------- */

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset: reason=%d", reason);
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE synced");
    start_scan();
}

/* ---- Scan ---------------------------------------------------------------- */

static void start_scan(void)
{
    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);

    struct ble_gap_disc_params params = {
        .passive           = 0, // active scan to get scan-response (name)
        .filter_duplicates = 1,
    };
    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_fn, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Scanning for Stadia controller…");
    }
}

static bool adv_contains_stadia(const uint8_t *data, uint8_t len)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, data, len) != 0) return false;
    if (fields.name != NULL && fields.name_len >= 6) {
        if (memcmp(fields.name, "Stadia", 6) == 0) return true;
    }
    return false;
}

/* ---- GATT discovery helpers --------------------------------------------- */

static void subscribe(void);
static int  svc_disc_fn (uint16_t conn_handle, const struct ble_gatt_error *err,
                          const struct ble_gatt_svc *svc, void *arg);
static int  chr_disc_fn (uint16_t conn_handle, const struct ble_gatt_error *err,
                          const struct ble_gatt_chr *chr, void *arg);
static int  dsc_disc_fn (uint16_t conn_handle, const struct ble_gatt_error *err,
                          uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
static int  report_ref_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg);
static int  cccd_write_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg);

/* Step 1: service discovery callback */
static int svc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                        const struct ble_gatt_svc *svc, void *arg)
{
    if (err->status == BLE_HS_EDONE) {
        if (g_hid_svc_start == 0) {
            ESP_LOGE(TAG, "HID service 0x1812 not found");
            return 0;
        }
        ESP_LOGI(TAG, "HID service: 0x%04x–0x%04x", g_hid_svc_start, g_hid_svc_end);
        s_chr_count = 0;
        ble_gattc_disc_all_chrs(conn_handle, g_hid_svc_start, g_hid_svc_end,
                                chr_disc_fn, NULL);
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "svc disc error: %d", err->status);
        return 0;
    }
    g_hid_svc_start = svc->start_handle;
    g_hid_svc_end   = svc->end_handle;
    return 0;
}

/* Step 2: characteristic discovery — collect all 0x2A4D (Report) chars */
static int chr_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                        const struct ble_gatt_chr *chr, void *arg)
{
    if (err->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Found %d 0x2A4D characteristics", s_chr_count);
        s_cur_chr = 0;
        process_next_chr();
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "chr disc error: %d", err->status);
        return 0;
    }
    if (chr->uuid.u.type == BLE_UUID_TYPE_16 &&
        chr->uuid.u16.value == 0x2A4D &&
        s_chr_count < MAX_REPORT_CHRS) {
        s_chrs[s_chr_count].def_handle = chr->def_handle;
        s_chrs[s_chr_count].val_handle = chr->val_handle;
        s_chr_count++;
    }
    return 0;
}

/*
 * process_next_chr — iterate through collected 0x2A4D characteristics.
 * For each, discover its descriptors to find 0x2908 (Report Reference) and
 * 0x2902 (CCCD), then read the report reference to identify input vs output.
 */
static void process_next_chr(void)
{
    if (s_cur_chr >= s_chr_count) {
        // All characteristics processed
        if (g_input_val_handle == 0 || g_output_val_handle == 0 ||
            g_input_cccd_handle == 0) {
            ESP_LOGE(TAG,
                     "Incomplete handles — in=0x%04x out=0x%04x cccd=0x%04x",
                     g_input_val_handle, g_output_val_handle, g_input_cccd_handle);
            return;
        }
        subscribe();
        return;
    }

    // Descriptor range: [val_handle+1 … next chr def_handle−1] or svc end
    uint16_t start = s_chrs[s_cur_chr].val_handle + 1;
    uint16_t end   = (s_cur_chr + 1 < s_chr_count)
                        ? (s_chrs[s_cur_chr + 1].def_handle - 1)
                        : g_hid_svc_end;

    if (start > end) {
        s_cur_chr++;
        process_next_chr();
        return;
    }

    s_cur_2908 = 0;
    s_cur_2902 = 0;
    ble_gattc_disc_all_dscs(g_conn_handle, s_chrs[s_cur_chr].val_handle, end,
                             dsc_disc_fn, NULL);
}

/* Step 3: descriptor discovery per characteristic */
static int dsc_disc_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                        uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    if (err->status == BLE_HS_EDONE) {
        if (s_cur_2908 != 0) {
            ble_gattc_read(conn_handle, s_cur_2908, report_ref_fn, NULL);
        } else {
            s_cur_chr++;
            process_next_chr();
        }
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGE(TAG, "dsc disc error: %d", err->status);
        s_cur_chr++;
        process_next_chr();
        return 0;
    }
    if (dsc->uuid.u.type == BLE_UUID_TYPE_16) {
        if      (dsc->uuid.u16.value == 0x2908) s_cur_2908 = dsc->handle;
        else if (dsc->uuid.u16.value == 0x2902) s_cur_2902 = dsc->handle;
    }
    return 0;
}

/* Step 4: read Report Reference (0x2908) → identify input / output chr */
static int report_ref_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                          struct ble_gatt_attr *attr, void *arg)
{
    if (err->status == 0 && attr != NULL) {
        uint8_t buf[2];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), &len) == 0 && len == 2) {
            uint8_t report_id   = buf[0];
            uint8_t report_type = buf[1];
            ESP_LOGI(TAG, "chr[%d] val=0x%04x id=%d type=%d",
                     s_cur_chr, s_chrs[s_cur_chr].val_handle, report_id, report_type);
            if (report_type == 1) { // Input
                g_input_val_handle  = s_chrs[s_cur_chr].val_handle;
                g_input_cccd_handle = s_cur_2902;
            } else if (report_type == 2) { // Output
                g_output_val_handle = s_chrs[s_cur_chr].val_handle;
            }
        }
    }
    s_cur_chr++;
    process_next_chr();
    return 0;
}

/* Step 5: write 0x0001 to input characteristic CCCD */
static void subscribe(void)
{
    ESP_LOGI(TAG, "Subscribing: val=0x%04x cccd=0x%04x out=0x%04x",
             g_input_val_handle, g_input_cccd_handle, g_output_val_handle);

    // Ensure the link is encrypted — Stadia requires it for HID notifications
    int rc = ble_gap_security_initiate(g_conn_handle);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Security initiate failed: %d (continuing anyway)", rc);
    }

    uint8_t val[2] = {0x01, 0x00};
    ble_gattc_write_flat(g_conn_handle, g_input_cccd_handle,
                         val, sizeof(val), cccd_write_fn, NULL);
}

static int cccd_write_fn(uint16_t conn_handle, const struct ble_gatt_error *err,
                          struct ble_gatt_attr *attr, void *arg)
{
    if (err->status == 0) {
        ESP_LOGI(TAG, "Notifications enabled — controller ready");
    } else {
        ESP_LOGE(TAG, "CCCD write failed: %d", err->status);
    }
    return 0;
}

/* ---- Rumble callout (runs on NimBLE event queue) ------------------------- */

static void rumble_callout_fn(struct ble_npl_event *ev)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_output_val_handle == 0) return;
    // Write-With-Response (not _no_rsp) — Stadia requires acknowledgement
    ble_gattc_write_flat(g_conn_handle, g_output_val_handle,
                         s_rumble_payload, 4, NULL, NULL);
}

void ble_central_send_rumble(const uint8_t *payload)
{
    memcpy(s_rumble_payload, payload, 4);
    ble_npl_callout_reset(&s_rumble_callout, 0);
}

/* ---- GAP event handler --------------------------------------------------- */

static int gap_event_fn(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC:
        if (adv_contains_stadia(event->disc.data, event->disc.length_data)) {
            ESP_LOGI(TAG, "Stadia found, connecting…");
            ble_gap_disc_cancel();

            uint8_t own_addr_type;
            ble_hs_id_infer_auto(0, &own_addr_type);
            ble_gap_connect(own_addr_type, &event->disc.addr,
                            5000, NULL, gap_event_fn, NULL);
        }
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle       = event->connect.conn_handle;
            g_hid_svc_start     = 0;
            g_hid_svc_end       = 0;
            g_input_val_handle  = 0;
            g_output_val_handle = 0;
            g_input_cccd_handle = 0;
            s_chr_count         = 0;
            ESP_LOGI(TAG, "Connected (handle=%d)", g_conn_handle);

            ble_uuid16_t hid_uuid = BLE_UUID16_INIT(0x1812);
            ble_gattc_disc_svc_by_uuid(g_conn_handle, &hid_uuid.u,
                                       svc_disc_fn, NULL);
        } else {
            ESP_LOGE(TAG, "Connect failed: %d", event->connect.status);
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason=%d), retry in 1 s",
                 event->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        esp_timer_start_once(s_reconnect_timer, 1000000 /* 1 s in µs */);
        break;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.attr_handle != g_input_val_handle) break;

        uint8_t raw[16];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, raw, sizeof(raw), &len) != 0) {
            ESP_LOGW(TAG, "mbuf extract failed");
            break;
        }

        #if DONGLE_DEBUG
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, raw, len, ESP_LOG_INFO);
        #endif

        // Stadia BLE sends 10 bytes: 9-byte report + 1 trailing byte.
        // Report ID is NOT included (stripped per HOGP spec); use bytes [0..8].
        if (len < 9) {
            ESP_LOGW(TAG, "HID report too short: len=%d", len);
            break;
        }
        const uint8_t *stadia = raw;

        uint8_t xbox[20];
        stadia_to_xbox360(stadia, xbox);
        xQueueSendToBack(ble_to_usb_queue, xbox, 0);
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption status: %d", event->enc_change.status);
        break;

    default:
        break;
    }
    return 0;
}

/* ---- NimBLE host task ---------------------------------------------------- */

void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---- Init ---------------------------------------------------------------- */

void ble_central_init(void)
{
    // Reconnect timer
    esp_timer_create_args_t ta = {
        .callback = reconnect_timer_cb,
        .name     = "ble_reconnect",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_reconnect_timer));

    // Rumble callout (executes on NimBLE's default event queue)
    ble_npl_callout_init(&s_rumble_callout,
                         nimble_port_get_dflt_eventq(),
                         rumble_callout_fn, NULL);

    // NimBLE host configuration
    ble_hs_cfg.reset_cb  = on_reset;
    ble_hs_cfg.sync_cb   = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; // Just Works
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm    = 0;
    ble_hs_cfg.sm_sc      = 1; // Secure Connections
}
