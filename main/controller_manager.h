#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CONTROLLER_MANAGER_MAX_CONTROLLERS 8

typedef struct {
    char address[18];
    char name[32];
    bool bonded;
} controller_info_t;

void controller_manager_init(void);
int controller_manager_bond_count(void);
int controller_manager_list(controller_info_t *out, int max_count);
bool controller_manager_is_pairing_mode(void);
void controller_manager_start_pairing(uint32_t timeout_ms);
void controller_manager_stop_pairing(void);
bool controller_manager_should_connect(const void *ble_addr, bool name_matches, bool directed);
void controller_manager_note_seen(const void *ble_addr, const char *name);
void controller_manager_note_connected(const void *ble_addr, const char *name);
void controller_manager_note_disconnected(void);
int controller_manager_forget_address(const char *address);
int controller_manager_forget_current(void);
int controller_manager_forget_all(void);
void controller_manager_current_address(char *out, size_t out_len);

