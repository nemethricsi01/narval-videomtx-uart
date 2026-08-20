// Threading model
// ---------------
// on_frame()  — called from the bmd_uart RX task whenever a full frame parses.
// getters     — called from the LVGL task via the notify callback / screen refresh.
//
// s_mutex protects s_ip/s_online/s_has_data AND s_notify_pending, preventing the
// dual-core race where two frames both pass the "!pending" check simultaneously.
// lv_async_call() is invoked only after s_mutex is released, and is wrapped in
// display_lock/display_unlock because lv_async_call internally calls lv_malloc,
// which is NOT thread-safe — see can_monitor.c for the same pattern.

#include "services/bmd_status.h"
#include "services/display.h"
#include "drivers/bmd_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "bmd_status";

static SemaphoreHandle_t s_mutex     = NULL;
static uint8_t           s_ip[4]     = {0};
static bool               s_online   = false;
static bool               s_has_data = false;

static void (*s_notify_fn)(void) = NULL;
static bool  s_notify_pending    = false; // guarded by s_mutex

static void notify_trampoline(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_notify_pending = false;
    xSemaphoreGive(s_mutex);

    if (s_notify_fn) s_notify_fn();
}

static void on_frame(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4, bool online)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ip[0] = ip1; s_ip[1] = ip2; s_ip[2] = ip3; s_ip[3] = ip4;
    s_online   = online;
    s_has_data = true;

    bool do_notify = (s_notify_fn && !s_notify_pending);
    if (do_notify) s_notify_pending = true;
    xSemaphoreGive(s_mutex);

    if (do_notify) {
        display_lock();
        lv_async_call(notify_trampoline, NULL);
        display_unlock();
    }
}

esp_err_t bmd_status_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    esp_err_t err = bmd_uart_init(on_frame);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

void bmd_status_set_notify(void (*fn)(void))
{
    s_notify_fn = fn;
}

bool bmd_status_has_data(void)
{
    if (!s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_has_data;
    xSemaphoreGive(s_mutex);
    return v;
}

bool bmd_status_is_online(void)
{
    if (!s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_online;
    xSemaphoreGive(s_mutex);
    return v;
}

void bmd_status_get_ip_str(char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;
    if (!s_mutex) { buf[0] = '\0'; return; }

    uint8_t ip[4];
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(ip, s_ip, sizeof(ip));
    xSemaphoreGive(s_mutex);

    snprintf(buf, buf_len, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}
