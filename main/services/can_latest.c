#include "services/can_latest.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include "videomtx.h"
#include "services/column_properties.h"
#include "services/prog.h"
#include "services/matrix_config.h"
#include "services/can.h"
#include "services/can_monitor.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "CAN_LATEST"

#define MAX_SEQ_LEN 16

typedef enum
{
    BUTTON_TYPE_HH = 1,
    BUTTON_TYPE_CSEND = 8,
} button_type_t;

typedef struct
{
    uint8_t button_id;
    button_type_t type;
    uint8_t dev_addr;
} button_event_t;

// LED sequence: led_order holds the ordered LED indices for one mode,
// ptr cycles through them as button presses arrive.
typedef struct
{
    uint8_t led_order[MAX_SEQ_LEN];  // LED indices to cycle through
    uint8_t len;               // valid entries: 4=Normal, 6=6Step, up to 16=Radio
    uint8_t ptr;               // current position in led_order
    uint8_t sel_idx;           // last-sent index into led_order (used for broadcast)
    uint8_t addrToSendTo[4];   // device addresses to send updates to for this column
    uint8_t mode;              // MODE_NORMAL / MODE_6STEP / MODE_RADIO_GREEN / MODE_RADIO_RED
    uint8_t base_addr;         // first LED layer address (PROP_LED_BASE_ADDR)
    uint8_t rows[MAX_SEQ_LEN]; // matrix row index for each led_order entry
} led_seq_t;

static led_seq_t s_cols[16];             // one per column
static uint16_t s_addr_to_col_mask[256]; // bit N = column N has this address

uint8_t LedBuff[4] = {0}; // for sending led updates, holds the 4 bytes of the CAN data field
can_frame_t tx = {0};

// Stores the single most-recently-received CAN frame.
// Protected by a mutex so it can be read safely from any task (e.g. the UI).
static can_frame_t s_frame;
static bool s_valid = false; // false until the first frame arrives
static SemaphoreHandle_t s_mutex = NULL;

// Called once at startup (before can_service_init).
void can_latest_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    assert(s_mutex);
}

// Finds which of col's configured states currently matches videomtx's route.
// Returns 0xFF if the routed input isn't one of this column's known states —
// the UI can route to any of the 16 inputs, but a column only knows about
// the rows its matrix config actually gave it.
static uint8_t find_col_state_for_route(uint8_t col)
{
    uint8_t target_row = videomtx_get(col);
    for (uint8_t i = 0; i < s_cols[col].len; i++) {
        if (s_cols[col].rows[i] == target_row) return i;
    }
    return 0xFF;
}

// Applies `found` as col's sel_idx and recomputes ptr so the next button
// press resumes cycling from the right spot.
static void apply_col_state(uint8_t col, uint8_t found)
{
    s_cols[col].sel_idx = found;

    uint8_t mode = s_cols[col].mode;
    if (mode == MODE_NORMAL || mode == MODE_NORMAL_SECONDARY) {
        s_cols[col].ptr = (found + 1 >= s_cols[col].len) ? 0 : found + 1;
    } else if (mode == MODE_6STEP || mode == MODE_6STEP_SECONDARY) {
        uint8_t half = s_cols[col].len / 2;
        if (found < half) {
            s_cols[col].ptr = (found + 1 >= half) ? 0 : found + 1;
        } else {
            s_cols[col].ptr = (found + 1 >= s_cols[col].len) ? half : found + 1;
        }
    }
    // RADIO: ptr unused for cycling, leave as-is
}

// Reads the saved vmtx crosspoints and resolves sel_idx / ptr for every column
// so that can_latest_broadcast() will send the correct initial LED states.
// Called at end of can_latest_configure().
static void vmtx_to_cols(void)
{
    for (uint8_t col = 0; col < 16; col++)
    {
        if (s_cols[col].len == 0)
            continue;

        uint8_t found = find_col_state_for_route(col);
        if (found == 0xFF) {
            ESP_LOGW(TAG, "col %d: vmtx row %d not in states, defaulting to 0", col, videomtx_get(col));
            found = 0;
        }
        apply_col_state(col, found);

        ESP_LOGI(TAG, "vmtx_to_cols: col %d vmtx_row=%d -> idx=%d ptr=%d",
                 col, videomtx_get(col), found, s_cols[col].ptr);
    }
}

// Called once after prog_init() — reads the committed prog settings to build
// local state (ledOrder, per-column config) used during runtime frame handling.
void can_latest_configure(void)
{
    const uint8_t *matrix = prog_get_matrix();
    const uint8_t *col_props = prog_get_col_props();

    for (int col = 0; col < 16; col++)
    {
        uint8_t mode = column_props_get(col_props, col, PROP_MODE);
        s_cols[col].addrToSendTo[0] = column_props_get(col_props, col, PROP_DEV_ADDR_1);
        s_cols[col].addrToSendTo[1] = column_props_get(col_props, col, PROP_DEV_ADDR_2);
        s_cols[col].addrToSendTo[2] = column_props_get(col_props, col, PROP_DEV_ADDR_3);
        s_cols[col].addrToSendTo[3] = column_props_get(col_props, col, PROP_DEV_ADDR_4);
        s_cols[col].mode = mode;
        s_cols[col].base_addr = column_props_get(col_props, col, PROP_LED_BASE_ADDR);
        s_cols[col].len = 0;
        s_cols[col].ptr = 0;
        s_cols[col].sel_idx = 0;
        if (mode == MODE_NORMAL || mode == MODE_NORMAL_SECONDARY)
        {
            for (uint8_t row = 0; row < 16 && s_cols[col].len < 4; row++) // loop rows until we have 4 states for this column
            {
                uint8_t color = matrix_get_cell(matrix, row, col);
                if (color > 0)
                {
                    s_cols[col].rows[s_cols[col].len] = row;
                    s_cols[col].led_order[s_cols[col].len++] = color - 1;
                }
            }
        }
        else if (mode == MODE_6STEP || mode == MODE_6STEP_SECONDARY)
        {
            // Six-step cells encode button group AND color in one value (5..10):
            //   5,6,7  -> button A, color (v-5)%3 = Green/Red/Yellow
            //   8,9,10 -> button B, color (v-5)%3 = Green/Red/Yellow
            // (v-5) already lands in 0..5, matching the fixed slot layout this
            // struct expects ([0..2]=button A G/R/Y, [3..5]=button B G/R/Y), so
            // the raw value maps straight to its slot. This replaces inferring
            // the button group from row-scan position (first 3 hits = A, next
            // 3 = B), which broke as soon as a column's rows weren't laid out
            // with all of button A's cells scanned before button B's.
            //
            // led_order is the actual on-wire LED code, and this codebase
            // reserves 0 for OFF — real colors are 1=Green, 2=Red, 3=Yellow
            // (see color_names below and MODE_NORMAL's `color - 1`). (v-5)%3
            // only yields 0..2, so it needs +1 to land on a real color code;
            // without it, "green" silently sent the OFF code and "yellow"
            // was never reachable at all.
            for (uint8_t i = 0; i < 6; i++) {
                s_cols[col].rows[i]      = 0xFF; // sentinel: slot not configured
                s_cols[col].led_order[i] = 0;
            }
            bool any = false;
            for (uint8_t row = 0; row < 16; row++)
            {
                uint8_t v = matrix_get_cell(matrix, row, col);
                if (v == 0) continue;
                if (v < 5 || v > 10) {
                    ESP_LOGW(TAG, "col %d row %d: value %d out of 6step range, skipped", col, row, v);
                    continue;
                }
                uint8_t slot = v - 5; // 0..5
                s_cols[col].rows[slot]      = row;
                s_cols[col].led_order[slot] = (slot % 3) + 1; // 1=Green 2=Red 3=Yellow
                any = true;
            }
            // len stays 0 (set at the top of this loop) for an unused column —
            // only claim the fixed 6-slot layout once it actually has cells.
            if (any) s_cols[col].len = 6;
        }
        else if (mode == MODE_RADIO_GREEN || mode == MODE_RADIO_RED)
        {
            for (uint8_t row = 0; row < 16 && s_cols[col].len < MAX_SEQ_LEN; row++) // loop rows until we fill the buf for this column
            {
                uint8_t color = matrix_get_cell(matrix, row, col);
                if (color > 0)
                {
                    s_cols[col].rows[s_cols[col].len] = row;
                    s_cols[col].led_order[s_cols[col].len++] = color - 1;
                }
            }
        }
    }

    memset(s_addr_to_col_mask, 0, sizeof(s_addr_to_col_mask));
    for (uint8_t col = 0; col < 16; col++)
    {
        for (uint8_t addresses = 0; addresses < 4; addresses++)
        {
            uint8_t addr = s_cols[col].addrToSendTo[addresses];
            if (addr != 0xFF)
            {
                s_addr_to_col_mask[addr] |= (1u << col);
            }
        }
    }
    vmtx_to_cols();
}


void can_latest_dump(void)
{
    static const char *mode_names[] = {
        [MODE_NORMAL]           = "NORMAL",
        [MODE_6STEP]            = "6STEP",
        [MODE_RADIO_GREEN]      = "RADIO_GRN",
        [MODE_RADIO_RED]        = "RADIO_RED",
        [MODE_NORMAL_SECONDARY] = "NORMAL_SEC",
        [MODE_6STEP_SECONDARY]  = "6STEP_SEC",
    };
    static const char *color_names[] = {"OFF", "GRN", "RED", "YEL"};

    for (uint8_t col = 0; col < 16; col++)
    {
        if (s_cols[col].len == 0)
            continue;
        uint8_t m = s_cols[col].mode;
        const char *mname = (m < 6) ? mode_names[m] : "?";
        ESP_LOGI(TAG, "col %2d | %-10s base=0x%02x len=%d ptr=%d sel=%d",
                 col, mname, s_cols[col].base_addr,
                 s_cols[col].len, s_cols[col].ptr, s_cols[col].sel_idx);

        ESP_LOGI(TAG, "       addrs: %02x %02x %02x %02x",
                 s_cols[col].addrToSendTo[0], s_cols[col].addrToSendTo[1],
                 s_cols[col].addrToSendTo[2], s_cols[col].addrToSendTo[3]);

        for (uint8_t i = 0; i < s_cols[col].len; i++)
        {
            uint8_t c = s_cols[col].led_order[i];
            ESP_LOGI(TAG, "       [%2d] row=%2d  color=%s",
                     i, s_cols[col].rows[i], (c < 4) ? color_names[c] : "?");
        }
    }
}

uint8_t baseAddressToLedLayer(uint8_t base_addr)
{
    if(base_addr == 0xFF || base_addr < 128)
        return 0xFF; // invalid base address
    base_addr -= 128;
    base_addr = 48 + (base_addr / 8); // convert from LED index to layer index
    return base_addr;
}

// Resolve which column index handles dev_addr.
// prefer_red=true  → pick the RADIO_RED column (CSEND)
// prefer_red=false → pick the non-RADIO_RED column (HH)
// Returns 0xFF on failure.
static uint8_t resolve_col(button_event_t event, bool prefer_red)
{
    uint16_t mask = s_addr_to_col_mask[event.dev_addr];
    uint8_t count = __builtin_popcount(mask);
    if (count == 0)
    {
        ESP_LOGE(TAG, "unknown addr 0x%02x", event.dev_addr);
        return 0xFF;
    }
    if (count > 2)
    {
        ESP_LOGE(TAG, "addr 0x%02x maps to %d cols", event.dev_addr, count);
        return 0xFF;
    }
    uint8_t col_a = __builtin_ctz(mask);
    if (count == 1)
        return col_a;
    uint8_t col_b = __builtin_ctz(mask & ~(1u << col_a));

    uint8_t pos = event.button_id % 8;//first or second button pressed
    uint8_t c[2] = {col_a, col_b};
    for (int i = 0; i < 2; i++) {
        uint8_t m = s_cols[c[i]].mode;
        if ((m == MODE_NORMAL && pos == 1) || (m == MODE_NORMAL_SECONDARY && pos == 2))
            return c[i];
        if ((m == MODE_6STEP && ((pos == 1) || (pos == 2))) || (m == MODE_6STEP_SECONDARY && ((pos == 3) || (pos == 4))))
            return c[i];
    }
    return ((s_cols[col_a].mode == MODE_RADIO_RED) == prefer_red) ? col_a : col_b;
}

// Send LedBuff to all device addresses registered for column col.
// Sends one frame for ≤8 LEDs, two frames for >8.
// slow=true adds a 15 ms delay after each frame — use during startup to avoid
// exhausting the TX pool at the low CAN bitrate (~9785 bps, ~10 ms/frame).
// from_ui=true must be set when calling from the LVGL task (e.g. via
// can_latest_notify_route()) — it routes the monitor push through
// can_mon_push_from_ui() to avoid a display_lock() self-deadlock.
static void send_to_col_addrs(uint8_t col, bool slow, bool from_ui)
{
    uint8_t base_addr = s_cols[col].base_addr;

    tx.header.id = 0x000;
    tx.header.dlc = 4;
    for (uint8_t addresses = 0; addresses < 4; addresses++)
    {
        uint8_t addr = s_cols[col].addrToSendTo[addresses];
        if (addr == 0xFF)
            continue;
        tx.data[0] = addr - 1;
        tx.data[1] = LedBuff[1];
        tx.data[2] = LedBuff[0];
        tx.data[3] = baseAddressToLedLayer(base_addr);
        esp_err_t ret = can_service_send(&tx, 100);
        if (ret == ESP_OK)
            from_ui ? can_mon_push_from_ui(&tx, true, false) : can_mon_push(&tx, true, false);
        else
            ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(ret));
        if (slow) vTaskDelay(pdMS_TO_TICKS(15));
        if (s_cols[col].len > 8)
        {
            tx.data[0] = addr - 1;
            tx.data[1] = LedBuff[3];
            tx.data[2] = LedBuff[2];
            tx.data[3] = baseAddressToLedLayer(base_addr) + 1;
            ret = can_service_send(&tx, 100);
            ESP_LOG_BUFFER_HEX(TAG, tx.data, 4);
            if (ret == ESP_OK)
                from_ui ? can_mon_push_from_ui(&tx, true, false) : can_mon_push(&tx, true, false);
            else
                ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(ret));
            if (slow) vTaskDelay(pdMS_TO_TICKS(15));
        }

    }
}

// Reconstruct LedBuff for column col from its stored sel_idx, without advancing state.
static void fill_ledbuff(uint8_t col)
{
    memset(LedBuff, 0, sizeof(LedBuff));
    uint8_t mode = s_cols[col].mode;
    uint8_t idx  = s_cols[col].sel_idx;
    if (s_cols[col].len == 0 || idx >= s_cols[col].len)
        return;

    if (mode == MODE_RADIO_GREEN || mode == MODE_RADIO_RED)
    {
        bool is_green = (mode == MODE_RADIO_GREEN);
        uint8_t byte_idx = idx / 4;
        uint8_t bit = is_green ? (idx % 4) * 2 : (idx % 4) * 2 + 1;
        LedBuff[byte_idx] |= (1u << bit);
        // merge companion (same base_addr, opposite RADIO mode)
        uint8_t comp = is_green ? MODE_RADIO_RED : MODE_RADIO_GREEN;
        for (uint8_t c = 0; c < 16; c++) {
            if (c == col || s_cols[c].mode != comp || baseAddressToLedLayer(s_cols[c].base_addr) != baseAddressToLedLayer(s_cols[col].base_addr)) continue;
            uint8_t ci = s_cols[c].sel_idx;
            if (ci < s_cols[c].len) {
                uint8_t cb = ci / 4;
                uint8_t cbit = is_green ? (ci % 4) * 2 + 1 : (ci % 4) * 2;
                LedBuff[cb] |= (1u << cbit);
            }
            break;
        }
    }
    else if (mode == MODE_NORMAL)
    {
        LedBuff[0] |= s_cols[col].led_order[idx] & 0x03;
        for (uint8_t c = 0; c < 16; c++) {
            if (c == col || s_cols[c].mode != MODE_NORMAL_SECONDARY || baseAddressToLedLayer(s_cols[c].base_addr) != baseAddressToLedLayer(s_cols[col].base_addr)) continue;
            uint8_t ci = s_cols[c].sel_idx;
            if (ci < s_cols[c].len) LedBuff[0] |= (s_cols[c].led_order[ci] << 2) & 0x0C;
            break;
        }
    }
    else if (mode == MODE_NORMAL_SECONDARY)
    {
        LedBuff[0] |= (s_cols[col].led_order[idx] << 2) & 0x0C;
        for (uint8_t c = 0; c < 16; c++) {
            if (c == col || s_cols[c].mode != MODE_NORMAL || baseAddressToLedLayer(s_cols[c].base_addr) != baseAddressToLedLayer(s_cols[col].base_addr)) continue;
            uint8_t ci = s_cols[c].sel_idx;
            if (ci < s_cols[c].len) LedBuff[0] |= s_cols[c].led_order[ci] & 0x03;
            break;
        }
    }
    else if (mode == MODE_6STEP)
    {
        uint8_t half = s_cols[col].len / 2;
        if (idx < half)
            LedBuff[0] |= s_cols[col].led_order[idx] & 0x03;
        else
            LedBuff[0] |= (s_cols[col].led_order[idx] << 2) & 0x0C;
        for (uint8_t c = 0; c < 16; c++) {
            if (c == col || s_cols[c].mode != MODE_6STEP_SECONDARY || baseAddressToLedLayer(s_cols[c].base_addr) != baseAddressToLedLayer(s_cols[col].base_addr)) continue;
            uint8_t ci = s_cols[c].sel_idx;
            uint8_t ch = s_cols[c].len / 2;
            if (ci < s_cols[c].len) {
                if (ci < ch)
                    LedBuff[0] |= (s_cols[c].led_order[ci] << 4) & 0x30;
                else
                    LedBuff[0] |= (s_cols[c].led_order[ci] << 6) & 0xC0;
            }
            break;
        }
    }
    else if (mode == MODE_6STEP_SECONDARY)
    {
        uint8_t half = s_cols[col].len / 2;
        if (idx < half)
            LedBuff[0] |= (s_cols[col].led_order[idx] << 4) & 0x30;
        else
            LedBuff[0] |= (s_cols[col].led_order[idx] << 6) & 0xC0;
        for (uint8_t c = 0; c < 16; c++) {
            if (c == col || s_cols[c].mode != MODE_6STEP || baseAddressToLedLayer(s_cols[c].base_addr) != baseAddressToLedLayer(s_cols[col].base_addr)) continue;
            uint8_t ci = s_cols[c].sel_idx;
            uint8_t ch = s_cols[c].len / 2;
            if (ci < s_cols[c].len) {
                if (ci < ch)
                    LedBuff[0] |= s_cols[c].led_order[ci] & 0x03;
                else
                    LedBuff[0] |= (s_cols[c].led_order[ci] << 2) & 0x0C;
            }
            break;
        }
    }
}

// Send the sel_idx state of every configured column to its device addresses.
void can_latest_broadcast(bool slow)
{
    for (uint8_t col = 0; col < 16; col++)
    {
        if (s_cols[col].len == 0)
            continue;
        fill_ledbuff(col);
        send_to_col_addrs(col, slow, false);
    }
    memset(LedBuff, 0, sizeof(LedBuff));
}

// Re-derives all column state from the currently committed prog data and
// pushes fresh LED states to the bus. Equivalent to what boot does — call
// after a live config upload (prog's commit notify) so new matrix/column
// properties apply without a reboot.
//
// TODO(concurrency): this rewrites s_cols[]/s_addr_to_col_mask with no lock,
// while can_task can concurrently be inside can_latest_update() reading the
// same arrays on a real button press. Worst case: one button press near an
// upload gets resolved against a half-rewritten config. Low-probability
// (config upload is a deliberate, infrequent action) but real.
//
// A naive fix (wrap s_mutex around this + the state-decision part of
// can_latest_update()) is NOT safe as a drop-in: can_latest_update() calls
// videomtx_set() -> the UI's notify callback -> display_lock(), while the
// LVGL task's ui_encoder_event -> can_latest_notify_route() takes s_mutex
// while ALREADY holding display_lock(). That's a lock-order inversion
// (s_mutex->display_lock on one path, display_lock->s_mutex on the other) =
// deadlock. Fixing this needs either a recursive mutex, or restructuring so
// s_mutex is never held while calling into anything that can reach
// display_lock() (i.e. release it before fill_ledbuff()/send_to_col_addrs()
// and before videomtx_set()). Revisit before shipping if uploads can
// realistically happen while the bus is live.
void can_latest_reconfigure(void)
{
    can_latest_configure();
    can_latest_dump();
    can_latest_broadcast(true);
}

// Called when a column's route changes from somewhere other than a real
// button press (currently: the on-device UI matrix editor). Syncs sel_idx/ptr
// to match the new route and pushes the update to the column's devices.
//
// The UI lets the user pick any of the 16 inputs, but a column only knows
// about the states its matrix config actually gave it (up to 4/6/16 rows
// depending on mode). If the picked input isn't one of them, there's no sane
// LED state to send, so this logs a warning and leaves the bus alone — the
// routing change itself already happened in videomtx regardless.
void can_latest_notify_route(uint8_t output)
{
    if (output >= 16 || s_cols[output].len == 0)
        return;

    uint8_t found = find_col_state_for_route(output);
    if (found == 0xFF) {
        ESP_LOGW(TAG, "col %d: UI route (row %d) not in this column's states — CAN not updated",
                 output, videomtx_get(output));
        return;
    }

    apply_col_state(output, found);
    fill_ledbuff(output);
    send_to_col_addrs(output, false, true);
}

// Called by can_task for every received CAN frame.
// Runs in the CAN task — safe to call videomtx_set() here (not the LVGL task).
void can_latest_update(const can_frame_t *frame)
{
    if (!s_mutex)
        return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_frame = *frame;
    s_valid = true;
    xSemaphoreGive(s_mutex);

    uint8_t sel_idx = 0xFF;

    if ((frame->header.dlc != 4 || frame->data[3] != 0x18))
        return;

    if(frame->data[1] == 0x4 && frame->data[3] == 0x18)
       {
        can_latest_broadcast(true);
        return;
       }

    button_event_t event = {
        .button_id = frame->data[2] + 1,
        .type = frame->data[1],
        .dev_addr = frame->data[0] + 1,
    };

    if (event.type != BUTTON_TYPE_HH && event.type != BUTTON_TYPE_CSEND)
        return;

    bool prefer_red = (event.type == BUTTON_TYPE_CSEND);
    uint8_t col = resolve_col(event, prefer_red);
    if (col == 0xFF)
        return;

    if (!prefer_red && s_cols[col].mode == MODE_RADIO_RED)
    {
        ESP_LOGE(TAG, "HH addr 0x%02x col %d is RADIO_RED", event.dev_addr, col);
        return;
    }
    if (prefer_red && s_cols[col].mode != MODE_RADIO_RED)
    {
        ESP_LOGE(TAG, "CSEND addr 0x%02x col %d not RADIO_RED", event.dev_addr, col);
        return;
    }

    int16_t whichLed = event.button_id - s_cols[col].base_addr;
    if (whichLed < 0 || (uint8_t)whichLed >= s_cols[col].len
        || ((s_cols[col].mode == MODE_6STEP        || s_cols[col].mode == MODE_6STEP_SECONDARY)   && whichLed >= 4)
        || ((s_cols[col].mode == MODE_NORMAL        || s_cols[col].mode == MODE_NORMAL_SECONDARY)  && whichLed >= 2))
    {
        ESP_LOGE(TAG, "button_id %d OOR col %d (base=%d len=%d)",
                 event.button_id, col, s_cols[col].base_addr, s_cols[col].len);
        return;
    }


    if (s_cols[col].mode == MODE_RADIO_GREEN || s_cols[col].mode == MODE_RADIO_RED)
    {
        sel_idx = (uint8_t)whichLed;
    }
    else if (s_cols[col].mode == MODE_NORMAL || s_cols[col].mode == MODE_NORMAL_SECONDARY)
    {
        sel_idx = s_cols[col].ptr;
        if (++s_cols[col].ptr >= s_cols[col].len) s_cols[col].ptr = 0;
    }
    else if (s_cols[col].mode == MODE_6STEP)
    {
        uint8_t half = s_cols[col].len / 2;
        if (whichLed == 0)
        {
            if (s_cols[col].ptr >= half) s_cols[col].ptr = 0;
            sel_idx = s_cols[col].ptr;
            if (++s_cols[col].ptr >= half) s_cols[col].ptr = 0;
        }
        else
        {
            if (s_cols[col].ptr < half) s_cols[col].ptr = half;
            sel_idx = s_cols[col].ptr;
            if (++s_cols[col].ptr >= s_cols[col].len) s_cols[col].ptr = half;
        }
    }
    else if (s_cols[col].mode == MODE_6STEP_SECONDARY)
    {
        ESP_LOGI(TAG,"whichLed: %d",whichLed);
        uint8_t half = s_cols[col].len / 2;
        if (whichLed == 0)
        {
            if (s_cols[col].ptr >= half) s_cols[col].ptr = 0;
            sel_idx = s_cols[col].ptr;
            if (++s_cols[col].ptr >= half) s_cols[col].ptr = 0;
        }
        else  // whichLed == 1
        {
            if (s_cols[col].ptr < half) s_cols[col].ptr = half;
            sel_idx = s_cols[col].ptr;
            if (++s_cols[col].ptr >= s_cols[col].len) s_cols[col].ptr = half;
        }
    }
    if (sel_idx != 0xFF) {
        s_cols[col].sel_idx = sel_idx;
        videomtx_set(col, s_cols[col].rows[sel_idx]);
    }
    fill_ledbuff(col);
    send_to_col_addrs(col, false, false);
}

// Returns the latest received frame into *out.
// Returns false (and leaves *out untouched) if no frame has arrived yet.
// Safe to call from any task.
bool can_latest_get(can_frame_t *out)
{
    if (!s_mutex || !out)
        return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_valid;
    if (v)
        *out = s_frame;
    xSemaphoreGive(s_mutex);
    return v;
}
