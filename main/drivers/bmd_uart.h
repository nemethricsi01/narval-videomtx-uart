#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Callback invoked (from the bmd_uart RX task) whenever a complete, valid
 * status frame is received. ip1..ip4 form the dotted address ip1.ip2.ip3.ip4;
 * online is true when the matrix reports the BMD as up.
 */
typedef void (*bmd_uart_notify_fn_t)(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4, bool online);

/**
 * Configure BOARD_BMD_UART_PORT for RX-only input on BOARD_PIN_BMD_UART_RX
 * and start the frame-parsing task. Call once at startup.
 */
esp_err_t bmd_uart_init(bmd_uart_notify_fn_t notify_fn);
