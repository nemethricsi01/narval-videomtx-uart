#pragma once

#include "esp_err.h"
#include "lvgl.h"

/**
 * Initialise the ST7789 LCD, LVGL, draw buffers, tick timer, and LVGL task.
 * The display is rotated 90° (landscape 320×170) on return.
 *
 * @param[out] out_disp  Populated with the LVGL display handle for UI code.
 */
esp_err_t display_init(lv_display_t **out_disp);

/**
 * Acquire the LVGL API lock before creating or modifying widgets.
 * Always pair with display_unlock().
 */
void display_lock(void);

/** Release the LVGL API lock. */
void display_unlock(void);

/**
 * Set backlight brightness via LEDC PWM.
 * @param percent  0–100; applied on a quadratic (gamma-2) curve.
 */
void display_set_brightness(uint8_t percent);

/**
 * Ramp the backlight from from_pct to to_pct over duration_ms.
 * Steps `percent` linearly in time and lets display_set_brightness()'s
 * gamma curve turn that into a perceptually-linear fade. Blocking
 * (vTaskDelay-based) — call only while NOT holding the LVGL lock, so the
 * LVGL task can keep rendering while the backlight moves. Intended for
 * startup/splash dim sequences.
 */
void display_fade(uint8_t from_pct, uint8_t to_pct, uint32_t duration_ms);
