#include "services/ui.h"
#include "services/display.h"
#include "services/settings.h"
#include "services/can_monitor.h"
#include "services/videomtx.h"
#include "services/can_latest.h"
#include "services/bmd_status.h"
#include "drivers/encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "ui";

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef enum {
    SCREEN_SPLASH,
    SCREEN_MAIN,
    SCREEN_MENU,
    SCREEN_BRIGHTNESS,
    SCREEN_CAN_MONITOR,
    SCREEN_BMD_STATUS,
} ui_screen_t;

static ui_screen_t  s_current_screen = SCREEN_SPLASH;
static lv_indev_t  *s_enc_indev      = NULL;

// Groups
static lv_group_t *s_menu_group  = NULL;
static lv_group_t *s_empty_group = NULL;

// Main screen
static lv_obj_t *s_main_scr = NULL;

// Matrix state
#define MTX_SIZE 16
static lv_obj_t *s_mtx_cells[MTX_SIZE];   // cell containers
static lv_obj_t *s_mtx_labels[MTX_SIZE];  // labels inside cells
static lv_obj_t *s_mtx_overlay     = NULL; // zoom overlay shown while editing
static lv_obj_t *s_mtx_overlay_lbl = NULL; // text inside the overlay
static uint8_t   s_mtx_route[MTX_SIZE];   // route[out] = input (0..15)
static int       s_mtx_sel        = 0;    // selected output index 0..15
static bool      s_mtx_edit       = false;// in input-edit mode
static uint8_t   s_mtx_edit_saved = 0;   // saved input value on edit entry

// Menu screen
static lv_obj_t *s_menu_scr       = NULL;
static lv_obj_t *s_status_label   = NULL;
static lv_obj_t *s_first_menu_btn = NULL;

// Brightness screen
static lv_obj_t *s_brightness_scr   = NULL;
static lv_obj_t *s_brightness_label = NULL;
static lv_obj_t *s_brightness_bar   = NULL;
static int       s_brightness_pct   = 100;
static int       s_brightness_saved = 100;

// CAN Monitor screen
static lv_obj_t *s_can_scr  = NULL;
static lv_obj_t *s_can_cont = NULL;
static lv_obj_t *s_can_log  = NULL;
static char      s_can_log_text[CAN_MON_LOG_SIZE * CAN_MON_LINE_MAX];
// When true, can_monitor_refresh() updates the log and scrolls to the bottom.
// Set false by manual scrolls so the user can browse history undisturbed.
// Restored to true by encoder click (insert separator) or screen entry.
static bool      s_can_follow = true;

// BMD Status screen
static lv_obj_t *s_bmd_scr           = NULL;
static lv_obj_t *s_bmd_ip_label      = NULL;
static lv_obj_t *s_bmd_online_label  = NULL;

// BMD status hint on the main screen (below "nyomva: menu")
static lv_obj_t *s_bmd_main_label    = NULL;

// Splash screen (shown at boot; see build_splash_screen())
static lv_obj_t *s_splash_scr          = NULL;
static lv_obj_t *s_splash_status_label = NULL;
static lv_obj_t *s_splash_ip_label     = NULL;

// Single rotation accumulator: 2 raw CW/CCW events = 1 detent = 1 step.
// Reset on every screen transition so there is never leftover state.
static int s_enc_accum = 0;

// ---------------------------------------------------------------------------
// Encoder accumulator
// ---------------------------------------------------------------------------

static int accum_step(encoder_event_t event)
{
    if (event != ENCODER_CW && event != ENCODER_CCW) return 0;
    s_enc_accum += (event == ENCODER_CW) ? 1 : -1;
    if (s_enc_accum >=  2) { s_enc_accum = 0; return  1; }
    if (s_enc_accum <= -2) { s_enc_accum = 0; return -1; }
    return 0;
}

// ---------------------------------------------------------------------------
// Value-change helpers
// ---------------------------------------------------------------------------

static void save_settings(void)
{
    settings_t cfg = { .brightness_pct = (uint8_t)s_brightness_pct };
    settings_save(&cfg);
}

static void brightness_refresh(void)
{
    lv_bar_set_value(s_brightness_bar, s_brightness_pct, LV_ANIM_OFF);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", s_brightness_pct);
    lv_label_set_text(s_brightness_label, buf);
    display_set_brightness((uint8_t)s_brightness_pct);
}

static void brightness_step(int dir)
{
    s_brightness_pct += dir * 5;
    if (s_brightness_pct < 20)  s_brightness_pct = 20;
    if (s_brightness_pct > 100) s_brightness_pct = 100;
    brightness_refresh();
}

static void group_nav(lv_group_t *g, int step)
{
    if (step > 0) lv_group_focus_next(g);
    else          lv_group_focus_prev(g);
}

static void group_click(lv_group_t *g)
{
    lv_obj_t *focused = lv_group_get_focused(g);
    if (focused) lv_obj_send_event(focused, LV_EVENT_CLICKED, NULL);
}

// ---------------------------------------------------------------------------
// Forward declarations for screen transitions used by callbacks
// ---------------------------------------------------------------------------

static void ui_show_brightness(void);
static void ui_open_can_monitor(void);
static void ui_open_bmd_status(void);
static void can_monitor_refresh(void);
static void bmd_ui_refresh(void);

// ---------------------------------------------------------------------------
// Event callbacks
// ---------------------------------------------------------------------------

static void on_menu_exit(lv_event_t *e) { ui_show_main(); }

static void on_menu_select(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);

    switch (idx) {
    case 0: ui_show_brightness();  break;
    case 1: ui_open_can_monitor(); break;
    case 2: ui_open_bmd_status();  break;
    default: {
        const char *text = lv_list_get_button_text(lv_obj_get_parent(btn), btn);
        lv_label_set_text(s_status_label, text);
        break;
    }
    }
}

static const char *s_menu_items[] = {
    LV_SYMBOL_IMAGE    " Brightness",
    LV_SYMBOL_WARNING  " CAN Monitor",
    LV_SYMBOL_WIFI     " BMD Status",
};

// ---------------------------------------------------------------------------
// Screen builders
// ---------------------------------------------------------------------------

static void mtx_cell_refresh(int out)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "Ki %02d < Be %02d", out + 1, s_mtx_route[out] + 1);
    lv_label_set_text(s_mtx_labels[out], buf);

    bool sel     = (out == s_mtx_sel);
    bool editing = sel && s_mtx_edit;

    // Background: green when editing, black otherwise
    lv_color_t bg = editing ? lv_palette_lighten(LV_PALETTE_RED, 2)
                  :           lv_color_black();
    lv_obj_set_style_bg_color(s_mtx_cells[out], bg, 0);

    // Border: red 3px when editing, yellow 2px when selected, dim grey 1px otherwise
    lv_color_t border_col = editing          ? lv_palette_main(LV_PALETTE_RED)
                          : (sel && !editing) ? lv_palette_main(LV_PALETTE_YELLOW)
                          :                    lv_color_make(200, 200, 200);
    lv_coord_t border_w   = editing          ? 3
                          : (sel && !editing) ? 3
                          :                    1;
    lv_obj_set_style_border_color(s_mtx_cells[out], border_col, 0);
    lv_obj_set_style_border_width(s_mtx_cells[out], border_w, 0);

    // Zoom overlay: visible only while editing
    if (editing) {
        lv_label_set_text(s_mtx_overlay_lbl, buf);
        lv_obj_clear_flag(s_mtx_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_mtx_overlay);
    } else {
        lv_obj_add_flag(s_mtx_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

static void build_main_screen(void)
{
    const uint8_t *vmtx = videomtx_get_all();
    for (int i = 0; i < MTX_SIZE; i++)
        s_mtx_route[i] = vmtx[i];

    s_main_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_main_scr, lv_color_black(), 0);
    lv_obj_set_style_pad_all(s_main_scr, 0, 0);
    lv_obj_set_scrollbar_mode(s_main_scr, LV_SCROLLBAR_MODE_OFF);

    // 3 columns x 6 rows = 18 slots, 16 outputs fit; 320/3=106 col_w, (170-14)/6=26 row_h
    for (int i = 0; i < MTX_SIZE; i++) {
        int col = i / 6;
        int row = i % 6;

        lv_obj_t *cell = lv_obj_create(s_main_scr);
        lv_obj_set_size(cell, 106, 26);
        lv_obj_set_pos(cell, col * 106, 14 + row * 26);
        lv_obj_set_style_pad_all(cell, 0, 0);
        lv_obj_set_style_radius(cell, 0, 0);
        lv_obj_set_style_border_width(cell, 1, 0);
        lv_obj_set_style_border_color(cell, lv_color_make(200, 200, 200), 0);
        lv_obj_set_style_bg_color(cell, lv_color_black(), 0);
        lv_obj_set_scrollbar_mode(cell, LV_SCROLLBAR_MODE_OFF);
        s_mtx_cells[i] = cell;

        lv_obj_t *lbl = lv_label_create(cell);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
        char buf[24];
        snprintf(buf, sizeof(buf), "Ki %02d < Be%02d", i + 1, i + 1);
        lv_label_set_text(lbl, buf);
        s_mtx_labels[i] = lbl;
    }

    // Zoom overlay — grey box centered on screen, shown while editing a cell
    s_mtx_overlay = lv_obj_create(s_main_scr);
    lv_obj_set_size(s_mtx_overlay, 200, 54);
    lv_obj_align(s_mtx_overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(s_mtx_overlay, lv_color_make(0x50, 0x50, 0x50), 0);
    lv_obj_set_style_bg_opa(s_mtx_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_mtx_overlay, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(s_mtx_overlay, 3, 0);
    lv_obj_set_style_radius(s_mtx_overlay, 6, 0);
    lv_obj_set_style_pad_all(s_mtx_overlay, 4, 0);
    lv_obj_set_scrollbar_mode(s_mtx_overlay, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_mtx_overlay, LV_OBJ_FLAG_HIDDEN);

    s_mtx_overlay_lbl = lv_label_create(s_mtx_overlay);
    lv_obj_set_style_text_font(s_mtx_overlay_lbl, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(s_mtx_overlay_lbl, lv_color_white(), 0);
    lv_obj_align(s_mtx_overlay_lbl, LV_ALIGN_CENTER, 0, 0);

    // Hint + BMD status in col 2, rows 4-5 (unused slots)

    lv_obj_t *hint1 = lv_label_create(s_main_scr);
    lv_obj_set_style_text_font(hint1, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint1, lv_color_white(), 0);
    lv_label_set_text(hint1, "nyomva:\nmenu");
    lv_obj_set_pos(hint1, 2 * 106 + 3, 14 + 4 * 26);

    s_bmd_main_label = lv_label_create(s_main_scr);
    lv_obj_set_style_text_font(s_bmd_main_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_bmd_main_label, lv_color_white(), 0);
    lv_label_set_text(s_bmd_main_label, "BMD: --");
    lv_obj_set_pos(s_bmd_main_label, 2 * 106 + 3, 14 + 4 * 26 + 34);

    for (int i = 0; i < MTX_SIZE; i++)
        mtx_cell_refresh(i);
}

static void build_menu_screen(void)
{
    s_menu_scr = lv_obj_create(NULL);

    s_status_label = lv_label_create(s_menu_scr);
    lv_label_set_text(s_status_label, "Select an item");
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(s_status_label, LV_PCT(100));
    lv_obj_set_style_text_align(s_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -4);

    lv_obj_t *list = lv_list_create(s_menu_scr);
    lv_obj_set_size(list, LV_PCT(100), 145);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 0);

    for (int i = 0; i < (int)(sizeof(s_menu_items) / sizeof(s_menu_items[0])); i++) {
        lv_obj_t *btn = lv_list_add_button(list, NULL, s_menu_items[i]);
        lv_obj_set_height(btn, LV_PCT(30));
        lv_obj_set_user_data(btn, (void *)(intptr_t)i);
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_20, 0);
        lv_group_add_obj(s_menu_group, btn);
        lv_obj_add_event_cb(btn, on_menu_select, LV_EVENT_CLICKED, NULL);
        if (i == 0) s_first_menu_btn = btn;
    }

    lv_obj_t *exit_btn = lv_list_add_button(list, LV_SYMBOL_HOME " ", "Exit");
    lv_obj_set_height(exit_btn, LV_PCT(30));
    lv_obj_set_style_text_font(exit_btn, &lv_font_montserrat_20, 0);
    lv_group_add_obj(s_menu_group, exit_btn);
    lv_obj_add_event_cb(exit_btn, on_menu_exit, LV_EVENT_CLICKED, NULL);
}

static void build_brightness_screen(void)
{
    s_brightness_scr = lv_obj_create(NULL);

    lv_obj_t *label = lv_label_create(s_brightness_scr);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_label_set_text(label, "Brightness");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 8);

    s_brightness_label = lv_label_create(s_brightness_scr);
    lv_obj_set_style_text_font(s_brightness_label, &lv_font_montserrat_20, 0);
    {
        char _buf[8];
        snprintf(_buf, sizeof(_buf), "%d%%", s_brightness_pct);
        lv_label_set_text(s_brightness_label, _buf);
    }
    lv_obj_align(s_brightness_label, LV_ALIGN_CENTER, 0, -14);

    s_brightness_bar = lv_bar_create(s_brightness_scr);
    lv_bar_set_range(s_brightness_bar, 20, 100);
    lv_bar_set_value(s_brightness_bar, s_brightness_pct, LV_ANIM_OFF);
    lv_obj_set_size(s_brightness_bar, LV_PCT(80), 20);
    lv_obj_align(s_brightness_bar, LV_ALIGN_CENTER, 0, 18);

    lv_obj_t *hint = lv_label_create(s_brightness_scr);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_label_set_text(hint, "Rotate: adjust  Click: save  Hold: cancel");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -4);
}

static void can_monitor_refresh(void)
{
    if (s_current_screen != SCREEN_CAN_MONITOR) return;
    if (!s_can_follow) return;

    can_mon_snapshot(s_can_log_text, sizeof(s_can_log_text));
    lv_label_set_text_static(s_can_log,
                             s_can_log_text[0] ? s_can_log_text : "Waiting for CAN frames...");
    lv_obj_update_layout(s_can_cont);
    lv_obj_scroll_to_y(s_can_cont, 100000, LV_ANIM_OFF);
}

static void build_can_monitor_screen(void)
{
    s_can_scr = lv_obj_create(NULL);
    lv_obj_set_style_pad_all(s_can_scr, 0, 0);

    lv_obj_t *hdr = lv_label_create(s_can_scr);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_14, 0);
    lv_label_set_text(hdr, LV_SYMBOL_WARNING " CAN Monitor  (long-press: back)");
    lv_obj_align(hdr, LV_ALIGN_TOP_LEFT, 4, 2);

    int log_h = lv_display_get_vertical_resolution(NULL) - 18;
    s_can_cont = lv_obj_create(s_can_scr);
    lv_obj_set_size(s_can_cont, LV_PCT(100), log_h);
    lv_obj_align(s_can_cont, LV_ALIGN_TOP_MID, 0, 18);
    lv_obj_set_style_pad_all(s_can_cont, 2, 0);
    lv_obj_set_scroll_dir(s_can_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_can_cont, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_clear_flag(s_can_cont, LV_OBJ_FLAG_SCROLL_ELASTIC);

    s_can_log = lv_label_create(s_can_cont);
    lv_obj_set_width(s_can_log, LV_PCT(100));
    lv_label_set_long_mode(s_can_log, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(s_can_log, &lv_font_montserrat_14, 0);
    lv_obj_align(s_can_log, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text_static(s_can_log, "Waiting for CAN frames...");
}

static void build_bmd_status_screen(void)
{
    s_bmd_scr = lv_obj_create(NULL);

    lv_obj_t *title = lv_label_create(s_bmd_scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_label_set_text(title, LV_SYMBOL_WIFI " BMD Status");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    s_bmd_ip_label = lv_label_create(s_bmd_scr);
    lv_obj_set_style_text_font(s_bmd_ip_label, &lv_font_montserrat_20, 0);
    lv_label_set_text(s_bmd_ip_label, "IP: --");
    lv_obj_align(s_bmd_ip_label, LV_ALIGN_CENTER, 0, -10);

    s_bmd_online_label = lv_label_create(s_bmd_scr);
    lv_obj_set_style_text_font(s_bmd_online_label, &lv_font_montserrat_20, 0);
    lv_label_set_text(s_bmd_online_label, "--");
    lv_obj_align(s_bmd_online_label, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t *hint = lv_label_create(s_bmd_scr);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_label_set_text(hint, "Long-press: back");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -4);
}

// Updates the main-screen hint and the BMD Status submenu from the latest
// bmd_status service state. Registered as the service's notify callback, so
// it runs in the LVGL task and may touch LVGL objects freely.
static void bmd_ui_refresh(void)
{
    bool has_data = bmd_status_has_data();
    bool online   = bmd_status_is_online();

    lv_color_t color = !has_data ? lv_palette_main(LV_PALETTE_PURPLE)
                      : online   ? lv_palette_main(LV_PALETTE_LIME)
                      :            lv_palette_main(LV_PALETTE_AMBER);

    char main_buf[24];
    snprintf(main_buf, sizeof(main_buf), "BMD: %s",
             !has_data ? "--" : online ? "online" : "offline");
    lv_label_set_text(s_bmd_main_label, main_buf);
    lv_obj_set_style_text_color(s_bmd_main_label, color, 0);

    char ip_str[16];
    bmd_status_get_ip_str(ip_str, sizeof(ip_str));
    char ip_buf[24];
    snprintf(ip_buf, sizeof(ip_buf), "IP: %s", has_data ? ip_str : "--");
    lv_label_set_text(s_bmd_ip_label, ip_buf);

    lv_label_set_text(s_bmd_online_label, !has_data ? "--" : online ? "ONLINE" : "OFFLINE");
    lv_obj_set_style_text_color(s_bmd_online_label, color, 0);

    // Splash screen: black text on its white box regardless of status.
    lv_label_set_text(s_splash_status_label,
                       !has_data ? "---" : online ? "online" : "offline");
    char splash_ip_buf[24];
    snprintf(splash_ip_buf, sizeof(splash_ip_buf), "BMD ip: %s", has_data ? ip_str : "--");
    lv_label_set_text(s_splash_ip_label, splash_ip_buf);
}

// Boot splash: black screen, centered "NARVAL SYSTEMS" / "BMD controller" up
// top, then a full-width white box flush with the bottom of the screen
// holding (inverted) BMD status + IP, filled in by bmd_ui_refresh().
static void build_splash_screen(void)
{
    s_splash_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_splash_scr, lv_color_black(), 0);
    lv_obj_set_style_pad_all(s_splash_scr, 0, 0);
    lv_obj_set_scrollbar_mode(s_splash_scr, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *title = lv_label_create(s_splash_scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "NARVAL SYSTEMS");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *subtitle = lv_label_create(s_splash_scr);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_white(), 0);
    lv_label_set_text(subtitle, "BMD controller");
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 54);

    lv_obj_t *box = lv_obj_create(s_splash_scr);
    lv_obj_set_size(box, LV_PCT(100), 64);
    lv_obj_align(box, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(box, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(box, 0, 0);
    lv_obj_set_style_radius(box, 0, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_set_scrollbar_mode(box, LV_SCROLLBAR_MODE_OFF);

    s_splash_status_label = lv_label_create(box);
    lv_obj_set_style_text_font(s_splash_status_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_splash_status_label, lv_color_black(), 0);
    lv_label_set_text(s_splash_status_label, "---");
    lv_obj_align(s_splash_status_label, LV_ALIGN_TOP_MID, 0, 8);

    s_splash_ip_label = lv_label_create(box);
    lv_obj_set_style_text_font(s_splash_ip_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_splash_ip_label, lv_color_black(), 0);
    lv_label_set_text(s_splash_ip_label, "BMD ip: --");
    lv_obj_align(s_splash_ip_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ---------------------------------------------------------------------------
// Screen transitions — each resets the accumulator to avoid leftover state
// ---------------------------------------------------------------------------

void ui_show_main(void)
{
    s_enc_accum  = 0;
    s_mtx_edit   = false;
    s_current_screen = SCREEN_MAIN;
    lv_indev_set_group(s_enc_indev, s_empty_group);
    lv_screen_load(s_main_scr);
}

void ui_show_menu(void)
{
    s_enc_accum = 0;
    s_current_screen = SCREEN_MENU;
    lv_indev_set_group(s_enc_indev, s_menu_group);
    lv_screen_load(s_menu_scr);
    lv_obj_t *prev = lv_group_get_focused(s_menu_group);
    if (prev) lv_group_focus_obj(prev);
}

void ui_open_menu(void)
{
    s_enc_accum = 0;
    s_current_screen = SCREEN_MENU;
    lv_indev_set_group(s_enc_indev, s_menu_group);
    lv_screen_load(s_menu_scr);
    lv_group_focus_obj(s_first_menu_btn);
}

static void ui_show_brightness(void)
{
    s_brightness_saved = s_brightness_pct;
    s_enc_accum = 0;
    s_current_screen = SCREEN_BRIGHTNESS;
    lv_indev_set_group(s_enc_indev, s_empty_group);
    lv_screen_load(s_brightness_scr);
    brightness_refresh();
}

static void ui_open_can_monitor(void)
{
    s_enc_accum = 0;
    s_can_follow = true;
    s_current_screen = SCREEN_CAN_MONITOR;
    lv_indev_set_group(s_enc_indev, s_empty_group);
    can_monitor_refresh();
    lv_screen_load(s_can_scr);
}

static void ui_open_bmd_status(void)
{
    s_enc_accum = 0;
    s_current_screen = SCREEN_BMD_STATUS;
    lv_indev_set_group(s_enc_indev, s_empty_group);
    bmd_ui_refresh();
    lv_screen_load(s_bmd_scr);
}

// ---------------------------------------------------------------------------
// Central encoder dispatcher — runs in the LVGL task via lv_async_call
// ---------------------------------------------------------------------------

void ui_encoder_event(void *arg)
{
    encoder_event_t event = (encoder_event_t)(uintptr_t)arg;
    int step = accum_step(event);

    switch (s_current_screen) {

    case SCREEN_SPLASH:
        break; // inert during the boot splash — s_empty_group already blocks focus nav

    case SCREEN_MAIN:
        if (step != 0) {
            int old = s_mtx_sel;
            if (s_mtx_edit) {
                int in = (int)s_mtx_route[s_mtx_sel] + step;
                if (in < 0)         in = MTX_SIZE - 1;
                if (in >= MTX_SIZE) in = 0;
                s_mtx_route[s_mtx_sel] = (uint8_t)in;
                mtx_cell_refresh(s_mtx_sel);
            } else {
                s_mtx_sel = (s_mtx_sel + step + MTX_SIZE) % MTX_SIZE;
                mtx_cell_refresh(old);
                mtx_cell_refresh(s_mtx_sel);
            }
        } else if (event == ENCODER_CLICK) {
            if (s_mtx_edit) {
                s_mtx_edit = false;
                videomtx_set_silent((uint8_t)s_mtx_sel, s_mtx_route[s_mtx_sel]);
                can_latest_notify_route((uint8_t)s_mtx_sel);
                mtx_cell_refresh(s_mtx_sel);
            } else {
                s_mtx_edit_saved = s_mtx_route[s_mtx_sel];
                s_mtx_edit = true;
                mtx_cell_refresh(s_mtx_sel);
            }
        } else if (event == ENCODER_LONG) {
            if (s_mtx_edit) {
                s_mtx_route[s_mtx_sel] = s_mtx_edit_saved;
                s_mtx_edit = false;
                mtx_cell_refresh(s_mtx_sel);
            } else {
                ui_open_menu();
            }
        }
        break;

    case SCREEN_MENU:
        if      (step != 0)              group_nav(s_menu_group, step);
        else if (event == ENCODER_CLICK) group_click(s_menu_group);
        break;

    case SCREEN_BRIGHTNESS:
        if      (step != 0)               brightness_step(step);
        else if (event == ENCODER_CLICK) { save_settings(); ui_show_menu(); }
        else if (event == ENCODER_LONG) {
            s_brightness_pct = s_brightness_saved;
            display_set_brightness((uint8_t)s_brightness_pct);
            ui_show_menu();
        }
        break;

    case SCREEN_CAN_MONITOR:
        if (event == ENCODER_CW) {
            s_can_follow = false;
            lv_obj_scroll_by(s_can_cont, 0,  8, LV_ANIM_OFF);
        } else if (event == ENCODER_CCW) {
            s_can_follow = false;
            lv_obj_scroll_by(s_can_cont, 0, -8, LV_ANIM_OFF);
        } else if (event == ENCODER_CLICK) {
            can_mon_push_separator_from_ui();
            s_can_follow = true;
            can_monitor_refresh();
        } else if (event == ENCODER_LONG) {
            ui_show_menu();
        }
        break;

    case SCREEN_BMD_STATUS:
        if (event == ENCODER_LONG) ui_show_menu();
        break;
    }
}

// ---------------------------------------------------------------------------
// External matrix update (called from non-LVGL task via videomtx notify)
// ---------------------------------------------------------------------------

static void mtx_async_apply(void *arg)
{
    uint32_t v      = (uint32_t)(uintptr_t)arg;
    uint8_t  output = (v >> 8) & 0xFF;
    uint8_t  input  =  v       & 0xFF;
    if (s_mtx_route[output] == input) return;
    s_mtx_route[output] = input;
    if (!s_mtx_edit || s_mtx_sel != (int)output)
        mtx_cell_refresh(output);
}

static void mtx_external_notify(uint8_t output, uint8_t input)
{
    display_lock();
    lv_async_call(mtx_async_apply, (void *)(uintptr_t)((uint32_t)(output << 8) | input));
    display_unlock();
}

// ---------------------------------------------------------------------------
// UI initialization
// ---------------------------------------------------------------------------

static void brightness_init_cb(void *arg) { (void)arg; brightness_refresh(); }

esp_err_t ui_init(lv_indev_t *encoder_indev, const settings_t *s)
{
    s_enc_indev      = encoder_indev;
    s_brightness_pct = (s->brightness_pct >= 20 && s->brightness_pct <= 100)
                       ? s->brightness_pct : 100;

    s_menu_group  = lv_group_create();
    s_empty_group = lv_group_create();
    lv_group_set_wrap(s_menu_group, false);

    build_main_screen();
    build_menu_screen();
    build_brightness_screen();
    build_can_monitor_screen();
    build_bmd_status_screen();
    build_splash_screen();

    // Register notifications before the splash starts, so a BMD status frame
    // (or a CAN/route change) arriving during the splash sequence still
    // updates the already-built-but-not-yet-visible target screens live,
    // instead of only taking effect once the user navigates there later.
    can_mon_set_notify(can_monitor_refresh);
    videomtx_set_notify(mtx_external_notify);
    bmd_status_set_notify(bmd_ui_refresh);
    bmd_ui_refresh();

    s_current_screen = SCREEN_SPLASH;
    lv_indev_set_group(s_enc_indev, s_empty_group);
    lv_screen_load(s_splash_scr);

    // Splash choreography: 2 s dim up, 3 s hold, 1 s dim down, swap to the
    // matrix screen while dark, then 1 s dim up over it. Each fade/delay
    // blocks, so release the LVGL lock around them — otherwise the LVGL
    // task can't render anything (including the splash we just loaded)
    // until the whole sequence finishes.
    display_unlock();
    display_fade(0, (uint8_t)s_brightness_pct, 2000);
    vTaskDelay(pdMS_TO_TICKS(3000));
    display_fade((uint8_t)s_brightness_pct, 0, 1000);
    display_lock();

    s_current_screen = SCREEN_MAIN;
    lv_screen_load(s_main_scr);

    display_unlock();
    display_fade(0, (uint8_t)s_brightness_pct, 1000);
    display_lock();
    ESP_LOGI(TAG, "splash done, brightness at %d%%", s_brightness_pct);

    // Re-apply brightness from the LVGL task on the first rendered frame.
    // Guards against any timing issue that prevents the fade above from
    // taking effect before the first lv_timer_handler() run.
    lv_async_call(brightness_init_cb, NULL);

    ESP_LOGI(TAG, "UI initialized");
    return ESP_OK;
}
