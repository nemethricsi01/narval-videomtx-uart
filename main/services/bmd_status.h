#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Init the underlying UART driver and start listening for status frames. */
esp_err_t bmd_status_init(void);

/**
 * Register the callback that fires (via lv_async_call) whenever a new status
 * frame arrives. The callback runs in the LVGL task — LVGL APIs may be called
 * freely inside it.
 */
void bmd_status_set_notify(void (*fn)(void));

/** True once at least one status frame has been received. */
bool bmd_status_has_data(void);

/** True when the last received frame reported the BMD online. Undefined before the first frame. */
bool bmd_status_is_online(void);

/** Format the last known IP address as "a.b.c.d" into buf. Thread-safe. */
void bmd_status_get_ip_str(char *buf, size_t buf_len);
