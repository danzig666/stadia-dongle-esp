#pragma once

#include <stdbool.h>

void web_server_init(void);
void web_server_start(bool explicit_request);
void web_server_request_start(bool explicit_request);
void web_server_stop(void);
void web_server_notify_usb_suspend(bool suspended);
bool web_server_is_active(void);
bool web_server_is_active_or_requested(void);
bool web_server_has_clients(void);
