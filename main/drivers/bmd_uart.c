// Parses status frames from the matrix:
//   [0xAA][ip4][ip3][ip2][ip1][status][0x55]
// ip4..ip1 are the IPv4 octets sent last-octet-first, so the dotted address
// reads ip1.ip2.ip3.ip4. status is 0 (offline) or 1 (online).

#include "drivers/bmd_uart.h"
#include "board.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "bmd_uart";

#define FRAME_START         0xAA
#define FRAME_END           0x55
#define FRAME_PAYLOAD_LEN   5   // ip4, ip3, ip2, ip1, status
#define BYTE_TIMEOUT_MS      200

static bmd_uart_notify_fn_t s_notify_fn;

static bool read_byte(uint8_t *out)
{
    return uart_read_bytes(BOARD_BMD_UART_PORT, out, 1, pdMS_TO_TICKS(BYTE_TIMEOUT_MS)) == 1;
}

static void rx_task(void *arg)
{
    uint8_t b;

    while (1) {
        // Resync: block until the next start marker.
        if (uart_read_bytes(BOARD_BMD_UART_PORT, &b, 1, portMAX_DELAY) != 1) continue;
        if (b != FRAME_START) continue;

        uint8_t payload[FRAME_PAYLOAD_LEN];
        bool ok = true;
        for (int i = 0; i < FRAME_PAYLOAD_LEN; i++) {
            if (!read_byte(&payload[i])) { ok = false; break; }
        }
        if (!ok) continue;

        uint8_t end = 0;
        if (!read_byte(&end) || end != FRAME_END) {
            ESP_LOGW(TAG, "bad frame terminator: 0x%02X", end);
            continue;
        }

        uint8_t ip4 = payload[0], ip3 = payload[1], ip2 = payload[2], ip1 = payload[3];
        bool    online = payload[4] != 0;
        if (s_notify_fn) s_notify_fn(ip1, ip2, ip3, ip4, online);
    }
}

esp_err_t bmd_uart_init(bmd_uart_notify_fn_t notify_fn)
{
    s_notify_fn = notify_fn;

    const uart_config_t cfg = {
        .baud_rate  = BOARD_BMD_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BOARD_BMD_UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_BMD_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_BMD_UART_PORT, UART_PIN_NO_CHANGE,
                                  BOARD_PIN_BMD_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(rx_task, "bmd_uart_rx", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "ready (RX=%d, %d baud)", BOARD_PIN_BMD_UART_RX, BOARD_BMD_UART_BAUD);
    return ESP_OK;
}
