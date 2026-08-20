#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include <string.h>

#include "services/settings.h"
#include "services/display.h"
#include "services/can.h"
#include "services/can_monitor.h"
#include "services/can_latest.h"
#include "services/ui.h"
#include "services/usb_bridge.h"
#include "services/prog.h"
#include "services/videomtx.h"
#include "services/bmd_status.h"
#include "drivers/twai_can.h"
#include "drivers/encoder.h"
#include "drivers/ws2812.h"
#include "board.h"

// ---------------------------------------------------------------------------
// Encoder → LVGL bridge
//
// encoder_read_cb always reports RELEASED / diff=0.  LVGL owns no encoder
// state, so it can never fire spurious clicks when groups are switched.
// All encoder logic lives in ui_encoder_event (runs in the LVGL task via
// lv_async_call, so LVGL widgets may be touched freely there).
// ---------------------------------------------------------------------------

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    data->enc_diff = 0;
    data->state    = LV_INDEV_STATE_RELEASED;
}

static void on_encoder_event(encoder_event_t event, void *user_data)
{
    display_lock();
    lv_async_call(ui_encoder_event, (void *)(uintptr_t)event);
    display_unlock();
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // let the dust settle after boot
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load settings before any subsystem that depends on them.
    settings_t cfg;
    settings_load(&cfg);

    can_mon_init();    // must be before any task that calls can_mon_push
    can_latest_init(); // must be before any task that calls can_latest_update
    videomtx_init();
    ESP_ERROR_CHECK(bmd_status_init());

    twai_node_handle_t can_node = NULL;
    ESP_ERROR_CHECK(twai_can_init(&can_node));
    ESP_ERROR_CHECK(can_service_init(can_node));
    prog_init();
    can_latest_reconfigure();
    prog_set_commit_notify(can_latest_reconfigure); // live-apply future USB config uploads, no reboot needed
#ifdef USB_BRIDGE_ENABLED
    ESP_ERROR_CHECK(usb_bridge_init());
#endif

    ESP_ERROR_CHECK(encoder_init(on_encoder_event, NULL));

    // Blink WS2812 5x white at power-up
    // Disabled for now — the splash screen already signals startup on the LCD.
    /*
    if (ws2812_init(BOARD_PIN_WS2812) == ESP_OK) {
        for (int i = 0; i < 3; i++) {
            ws2812_set(128, 128, 128);
            vTaskDelay(pdMS_TO_TICKS(150));
            ws2812_off();
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        ws2812_set(0, 32, 0);
        ws2812_deinit();
    }
    */

    lv_display_t *disp = NULL;
    ESP_ERROR_CHECK(display_init(&disp));

    display_lock();

    lv_indev_t *enc_indev = lv_indev_create();
    lv_indev_set_type(enc_indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(enc_indev, encoder_read_cb);

    ESP_ERROR_CHECK(ui_init(enc_indev, &cfg));

    display_unlock();
}
