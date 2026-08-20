#include "services/display.h"
#include "drivers/lcd_st7789.h"
#include "board.h"
#include "driver/ledc.h"
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "display";

#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_CHANNEL     LEDC_CHANNEL_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ     1000
#define LEDC_RESOLUTION  LEDC_TIMER_8_BIT

#define LVGL_DRAW_BUF_LINES    80
#define LVGL_TICK_PERIOD_MS     2
#define LVGL_TASK_STACK_SIZE   (24 * 1024)
#define LVGL_TASK_PRIORITY      4
#define LVGL_TASK_MAX_DELAY_MS  500
#define LVGL_TASK_MIN_DELAY_MS (1000 / CONFIG_FREERTOS_HZ)

static _lock_t s_lvgl_lock;

// ---------------------------------------------------------------------------
// Backlight (LEDC)
// ---------------------------------------------------------------------------

static void ledc_bl_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t ch = {
        .gpio_num   = BOARD_PIN_LCD_BL,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

void display_set_brightness(uint8_t pct)
{
    if (pct > 100) pct = 100;

    uint32_t duty = (uint32_t)pct * pct * 255 / 10000;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

#define FADE_STEP_MS  20  // ~50 steps/sec — smooth without spamming LEDC updates

void display_fade(uint8_t from_pct, uint8_t to_pct, uint32_t duration_ms)
{
    if (from_pct > 100) from_pct = 100;
    if (to_pct   > 100) to_pct   = 100;
    if (duration_ms == 0) {
        display_set_brightness(to_pct);
        return;
    }

    uint32_t steps = duration_ms / FADE_STEP_MS;
    if (steps == 0) steps = 1;

    for (uint32_t i = 1; i <= steps; i++) {
        int32_t pct = (int32_t)from_pct + ((int32_t)to_pct - from_pct) * (int32_t)i / (int32_t)steps;
        display_set_brightness((uint8_t)pct);
        vTaskDelay(pdMS_TO_TICKS(FADE_STEP_MS));
    }
    display_set_brightness(to_pct); // land exactly on target regardless of rounding
}

// ---------------------------------------------------------------------------
// LVGL callbacks
// ---------------------------------------------------------------------------

static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                 esp_lcd_panel_io_event_data_t *edata,
                                 void *user_ctx)
{
    lv_display_flush_ready((lv_display_t *)user_ctx);
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = lv_display_get_user_data(disp);
    int x1 = area->x1, x2 = area->x2;
    int y1 = area->y1, y2 = area->y2;
    lv_draw_sw_rgb565_swap(px_map, (x2 + 1 - x1) * (y2 + 1 - y1));
    esp_lcd_panel_draw_bitmap(panel, x1, y1, x2 + 1, y2 + 1, px_map);
}

static void lvgl_tick_inc_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    uint32_t delay_ms;
    while (1) {
        _lock_acquire(&s_lvgl_lock);
        delay_ms = lv_timer_handler();
        _lock_release(&s_lvgl_lock);
        delay_ms = MAX(delay_ms, LVGL_TASK_MIN_DELAY_MS);
        delay_ms = MIN(delay_ms, LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * delay_ms);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t display_init(lv_display_t **out_disp)
{
    ledc_bl_init();   // backlight at 0% until ui_init applies the saved brightness

    esp_lcd_panel_handle_t    panel  = NULL;
    esp_lcd_panel_io_handle_t io     = NULL;
    ESP_ERROR_CHECK(lcd_st7789_init(&panel, &io));

    lv_init();

    lv_display_t *disp = lv_display_create(BOARD_LCD_H_RES, BOARD_LCD_V_RES);
    size_t buf_sz = BOARD_LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(BOARD_LCD_SPI_HOST, buf_sz, 0);
    void *buf2 = spi_bus_dma_memory_alloc(BOARD_LCD_SPI_HOST, buf_sz, 0);
    assert(buf1 && buf2);

    lv_display_set_buffers(disp, buf1, buf2, buf_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(disp, panel);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    const esp_lcd_panel_io_callbacks_t io_cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &io_cbs, disp));

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    esp_lcd_panel_swap_xy(panel, true);
    esp_lcd_panel_mirror(panel, false, true);
    esp_lcd_panel_set_gap(panel, 0, 35);
    esp_lcd_panel_invert_color(panel, true);

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_inc_cb,
        .name     = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    xTaskCreate(lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "ready");
    if (out_disp) *out_disp = disp;
    return ESP_OK;
}

void display_lock(void)
{
    _lock_acquire(&s_lvgl_lock);
}

void display_unlock(void)
{
    _lock_release(&s_lvgl_lock);
}
