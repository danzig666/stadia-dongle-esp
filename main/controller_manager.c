#include "controller_manager.h"

#include "dongle_state.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"

#include <stdio.h>
#include <string.h>

static SemaphoreHandle_t s_lock;
static bool s_pairing;
static int64_t s_pairing_deadline_us;
static ble_addr_t s_current_addr;
static bool s_have_current;
static char s_current_addr_str[18];

static void addr_to_str(const ble_addr_t *addr, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

static bool str_to_addr(const char *str, ble_addr_t *addr)
{
    unsigned int b[6];
    if (!str || sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
                       &b[5], &b[4], &b[3], &b[2], &b[1], &b[0]) != 6) {
        return false;
    }
    addr->type = BLE_ADDR_PUBLIC;
    for (int i = 0; i < 6; i++) addr->val[i] = (uint8_t)b[i];
    return true;
}

void controller_manager_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        configASSERT(s_lock != NULL);
    }
    dongle_state_set_stored_bonds(controller_manager_bond_count());
}

int controller_manager_bond_count(void)
{
    int count = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &count);
    dongle_state_set_stored_bonds(count);
    return count;
}

int controller_manager_list(controller_info_t *out, int max_count)
{
    if (!out || max_count <= 0) return 0;
    ble_addr_t peers[CONTROLLER_MANAGER_MAX_CONTROLLERS];
    int count = 0;
    if (ble_store_util_bonded_peers(peers, &count, CONTROLLER_MANAGER_MAX_CONTROLLERS) != 0) return 0;
    if (count > max_count) count = max_count;
    for (int i = 0; i < count; i++) {
        addr_to_str(&peers[i], out[i].address, sizeof(out[i].address));
        snprintf(out[i].name, sizeof(out[i].name), "Stadia Controller");
        out[i].bonded = true;
    }
    dongle_state_set_stored_bonds(count);
    return count;
}

bool controller_manager_is_pairing_mode(void)
{
    if (!s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_pairing && s_pairing_deadline_us > 0 && esp_timer_get_time() > s_pairing_deadline_us) {
        s_pairing = false;
        s_pairing_deadline_us = 0;
        xSemaphoreGive(s_lock);
        dongle_state_set_pairing_mode(false);
        return false;
    }
    bool result = s_pairing;
    xSemaphoreGive(s_lock);
    return result;
}

void controller_manager_start_pairing(uint32_t timeout_ms)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pairing = true;
    s_pairing_deadline_us = timeout_ms ? esp_timer_get_time() + (int64_t)timeout_ms * 1000 : 0;
    xSemaphoreGive(s_lock);
    dongle_state_set_pairing_mode(true);
    dongle_state_set(DONGLE_STATE_PAIRING);
}

void controller_manager_stop_pairing(void)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pairing = false;
    s_pairing_deadline_us = 0;
    xSemaphoreGive(s_lock);
    dongle_state_set_pairing_mode(false);
}

bool controller_manager_should_connect(const void *ble_addr_ptr, bool name_matches, bool directed)
{
    const ble_addr_t *addr = (const ble_addr_t *)ble_addr_ptr;
    if (controller_manager_is_pairing_mode()) return name_matches || directed;
    if (directed) return true;
    if (name_matches) return true;
    struct ble_store_key_sec key = {0};
    key.peer_addr = *addr;
    key.idx = 0;
    struct ble_store_value_sec val;
    return ble_store_read_peer_sec(&key, &val) == 0;
}

void controller_manager_note_seen(const void *ble_addr_ptr, const char *name)
{
    (void)ble_addr_ptr;
    (void)name;
}

void controller_manager_note_connected(const void *ble_addr_ptr, const char *name)
{
    const ble_addr_t *addr = (const ble_addr_t *)ble_addr_ptr;
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_current_addr = *addr;
    s_have_current = true;
    addr_to_str(addr, s_current_addr_str, sizeof(s_current_addr_str));
    xSemaphoreGive(s_lock);
    struct ble_store_key_sec key = {0};
    key.peer_addr = *addr;
    key.idx = 0;
    struct ble_store_value_sec val;
    bool bonded = ble_store_read_peer_sec(&key, &val) == 0;
    dongle_state_set_controller(s_current_addr_str, name ? name : "Stadia Controller", bonded);
}

void controller_manager_note_disconnected(void)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_have_current = false;
    s_current_addr_str[0] = '\0';
    xSemaphoreGive(s_lock);
}

int controller_manager_forget_address(const char *address)
{
    ble_addr_t addr;
    if (!str_to_addr(address, &addr)) return -1;
    int rc = ble_gap_unpair(&addr);
    if (rc != 0) rc = ble_store_util_delete_peer(&addr);
    dongle_state_set_stored_bonds(controller_manager_bond_count());
    return rc;
}

int controller_manager_forget_current(void)
{
    if (!s_lock) return -1;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_have_current) {
        xSemaphoreGive(s_lock);
        return -1;
    }
    ble_addr_t addr = s_current_addr;
    xSemaphoreGive(s_lock);
    int rc = ble_gap_unpair(&addr);
    if (rc != 0) rc = ble_store_util_delete_peer(&addr);
    dongle_state_set_stored_bonds(controller_manager_bond_count());
    return rc;
}

int controller_manager_forget_all(void)
{
    int rc = ble_store_clear();
    dongle_state_set_stored_bonds(controller_manager_bond_count());
    return rc;
}

void controller_manager_current_address(char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    if (!s_lock) { out[0] = '\0'; return; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    snprintf(out, out_len, "%s", s_current_addr_str);
    xSemaphoreGive(s_lock);
}
