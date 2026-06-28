/**
 * @file tuya_main.c
 * @brief Water Tank Controller v1.1.0
 *
 * Changes vs v1.2.5:
 * - Changed cloud reporting threshold for water level from 2% to 5% to further
 * reduce Tuya API calls. Local LCD still updates in real-time.
 */

#include "cJSON.h"
#include "netmgr.h"
#include "tal_api.h"
#include "tkl_output.h"
#include "tuya_config.h"
#include "tuya_iot.h"
#include "tuya_iot_dp.h"
#include "tal_kv.h"
#include "tal_uart.h"
#include "tkl_gpio.h"
#include "tuya_authorize.h"
#include "ili9341.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#if defined(ENABLE_WIFI) && (ENABLE_WIFI == 1)
#include "netconn_wifi.h"
#endif

/* ========== Forward declarations ========== */
static void report_dps_smart(void);
static void request_report_now(void);

/* ========== Tuya Globals ========== */
static tuya_iot_client_t client;
static tuya_iot_license_t license;

/* ========== KV Store Keys ========== */
#define KV_TANK_HEIGHT      "wt_tank_h"
#define KV_SENSOR_OFFSET    "wt_sens_off"
#define KV_ON_THRESHOLD     "wt_thr_on"
#define KV_OFF_THRESHOLD    "wt_thr_off"
#define KV_PUMP_MODE        "wt_mode"
#define KV_PUMP_STATE       "wt_pump_st"
#define KV_SETTINGS_VALID   "wt_valid"
#define KV_MAGIC_VALUE      0xA5

/* ========== Application State ========== */
typedef struct {
    int16_t tank_height_cm;
    int16_t sensor_offset_cm;
    uint8_t on_threshold_pct;
    uint8_t off_threshold_pct;
    uint8_t pump_mode;
    int water_level_pct;
    int raw_distance_cm;
    BOOL_T pump_running;
    BOOL_T power_recovery;
    int sensor_fail_count;
    BOOL_T manual_override;
    BOOL_T display_ready;
    uint32_t fault_bits;
} app_state_t;

static app_state_t g_app = {
    .tank_height_cm    = DEFAULT_TANK_HEIGHT,
    .sensor_offset_cm  = DEFAULT_SENSOR_OFFSET,
    .on_threshold_pct  = DEFAULT_ON_THRESHOLD,
    .off_threshold_pct = DEFAULT_OFF_THRESHOLD,
    .pump_mode         = 0,
    .water_level_pct   = 0,
    .raw_distance_cm   = 0,
    .pump_running      = FALSE,
    .power_recovery    = FALSE,
    .sensor_fail_count = 0,
    .manual_override   = FALSE,
    .display_ready     = FALSE,
    .fault_bits        = 0,
};

/* ========== UI State ========== */
typedef enum { SCREEN_HOME, SCREEN_MENU, SCREEN_EDIT } screen_t;
static screen_t g_screen = SCREEN_HOME;
static int g_menu_sel = 0;
static int g_prev_menu_sel = -1;
static int g_edit_val = 0;
static int g_edit_item = -1;
static BOOL_T g_ui_dirty = TRUE;
static BOOL_T g_full_redraw = TRUE;
static BOOL_T g_menu_full = TRUE;
static BOOL_T g_edit_full = TRUE;

/* Cache of last-displayed home values — used to skip redraw when nothing changed */
static int    g_disp_level   = -1;
static int    g_disp_pump    = -1;
static int    g_disp_mode    = -1;
static int    g_disp_fault   = -1;

#define MENU_ITEMS 7
static const char *menu_labels[MENU_ITEMS] = {
    "Tank Height", "Sensor Offset", "ON Threshold",
    "OFF Threshold", "Pump Mode", "Raw Distance", "Back"
};

/* ========================================================================
 * SMART CLOUD REPORTING — PER-DP CHANGE TRACKING
 * ======================================================================== */

#define LEVEL_REPORT_THRESHOLD_PCT   5              /* report level if changed by >=5% */
#define DISTANCE_REPORT_THRESHOLD_CM 5              /* report distance if changed by >=5 cm */
#define HEARTBEAT_MS                 (10 * 60 * 1000) /* 10 minutes — only for live values */

/* Sentinel values that can never equal a real reading,
 * so the first call after boot reports each DP exactly once. */
static int      g_rep_level    = -999;
static int      g_rep_distance = -999;
static int      g_rep_pump     = -1;
static int      g_rep_mode     = -1;
static int      g_rep_on_thr   = -1;
static int      g_rep_off_thr  = -1;
static int      g_rep_tank_h   = -1;
static int      g_rep_sens_off = -1;
static int      g_rep_fault    = -1;
static uint32_t g_rep_last_time = 0;

/* Force-report flag — set by relay_set, button handlers, MQTT_CONNECTED, etc. */
static volatile BOOL_T g_force_report = FALSE;

static void request_report_now(void)
{
    g_force_report = TRUE;
}

/* Build a dp_obj_t array containing only DPs that changed (or all of them
 * if forced / heartbeat). Sends nothing if everything is already in sync. */
static void report_dps_smart(void)
{
    /* Don't even build the array if we have no link — saves CPU. */
    netmgr_status_e nstat = NETMGR_LINK_DOWN;
    netmgr_conn_get(NETCONN_AUTO, NETCONN_CMD_STATUS, &nstat);
    if (nstat == NETMGR_LINK_DOWN) return;

    dp_obj_t dps[9];
    int cnt = 0;
    uint32_t now = tal_system_get_millisecond();
    int cur_fault = (g_app.fault_bits & 0x01) ? 1 : 0;
    BOOL_T force = g_force_report;
    g_force_report = FALSE;

    /* Heartbeat applies ONLY to live values (level / distance / pump status)
     * so the app's home screen stays fresh even if nothing changed.
     * Settings don't need heartbeats — the cloud already has them. */
    BOOL_T heartbeat = (g_rep_last_time == 0) || (now - g_rep_last_time >= HEARTBEAT_MS);

    /* ---- Pump switch + pump-running status (live, change-driven) ---- */
    if (force || heartbeat || g_app.pump_running != g_rep_pump) {
        dps[cnt].id   = DPID_SWITCH;
        dps[cnt].type = PROP_BOOL;
        dps[cnt].value.dp_bool = g_app.pump_running;
        cnt++;

        dps[cnt].id   = DPID_PUMP_RUNNING;
        dps[cnt].type = PROP_BOOL;
        dps[cnt].value.dp_bool = g_app.pump_running;
        cnt++;

        g_rep_pump = g_app.pump_running;
    }

    /* ---- Water level (live, threshold-gated) ---- */
    if (force || heartbeat ||
        abs(g_app.water_level_pct - g_rep_level) >= LEVEL_REPORT_THRESHOLD_PCT) {
        dps[cnt].id   = DPID_WATER_LEVEL;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.water_level_pct;
        cnt++;
        g_rep_level = g_app.water_level_pct;
    }

    /* ---- Raw distance (live, threshold-gated to filter sensor noise) ---- */
    if (force || heartbeat ||
        abs(g_app.raw_distance_cm - g_rep_distance) >= DISTANCE_REPORT_THRESHOLD_CM) {
        dps[cnt].id   = DPID_RAW_DISTANCE;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.raw_distance_cm;
        cnt++;
        g_rep_distance = g_app.raw_distance_cm;
    }

    /* ---- Settings (change-only; NO heartbeat — they're already in cloud) ---- */
    if (force || g_app.pump_mode != g_rep_mode) {
        dps[cnt].id   = DPID_PUMP_MODE;
        dps[cnt].type = PROP_ENUM;
        dps[cnt].value.dp_enum = g_app.pump_mode;
        cnt++;
        g_rep_mode = g_app.pump_mode;
    }
    if (force || g_app.on_threshold_pct != g_rep_on_thr) {
        dps[cnt].id   = DPID_ON_THRESHOLD;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.on_threshold_pct;
        cnt++;
        g_rep_on_thr = g_app.on_threshold_pct;
    }
    if (force || g_app.off_threshold_pct != g_rep_off_thr) {
        dps[cnt].id   = DPID_OFF_THRESHOLD;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.off_threshold_pct;
        cnt++;
        g_rep_off_thr = g_app.off_threshold_pct;
    }
    if (force || g_app.tank_height_cm != g_rep_tank_h) {
        dps[cnt].id   = DPID_TANK_HEIGHT;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.tank_height_cm;
        cnt++;
        g_rep_tank_h = g_app.tank_height_cm;
    }
    if (force || g_app.sensor_offset_cm != g_rep_sens_off) {
        dps[cnt].id   = DPID_SENSOR_OFFSET;
        dps[cnt].type = PROP_VALUE;
        dps[cnt].value.dp_value = g_app.sensor_offset_cm;
        cnt++;
        g_rep_sens_off = g_app.sensor_offset_cm;
    }

    if (cnt > 0) {
        tuya_iot_dp_obj_report(&client, NULL, dps, cnt, 0);
        g_rep_last_time = now;
        g_rep_fault = cur_fault;
    }
}

/* ========== Flash Persistence ========== */

static void kv_save_int(const char *key, int value)
{
    tal_kv_set(key, (const uint8_t *)&value, sizeof(int));
}

static int kv_read_int(const char *key, int default_val)
{
    int value = default_val;
    uint8_t *buf = NULL;
    size_t len = 0;
    if (OPRT_OK == tal_kv_get(key, &buf, &len)) {
        if (buf != NULL && len == sizeof(int)) memcpy(&value, buf, sizeof(int));
        if (buf) tal_kv_free(buf);
    }
    return value;
}

static void settings_save(void)
{
    kv_save_int(KV_TANK_HEIGHT,    g_app.tank_height_cm);
    kv_save_int(KV_SENSOR_OFFSET,  g_app.sensor_offset_cm);
    kv_save_int(KV_ON_THRESHOLD,   g_app.on_threshold_pct);
    kv_save_int(KV_OFF_THRESHOLD,  g_app.off_threshold_pct);
    kv_save_int(KV_PUMP_MODE,      g_app.pump_mode);
    kv_save_int(KV_SETTINGS_VALID, KV_MAGIC_VALUE);
}

static void pump_state_save(void)
{
    kv_save_int(KV_PUMP_STATE, (int)g_app.pump_running);
}

static void settings_load(void)
{
    if (kv_read_int(KV_SETTINGS_VALID, 0) != KV_MAGIC_VALUE) { settings_save(); return; }
    g_app.tank_height_cm   = kv_read_int(KV_TANK_HEIGHT,   DEFAULT_TANK_HEIGHT);
    g_app.sensor_offset_cm = kv_read_int(KV_SENSOR_OFFSET, DEFAULT_SENSOR_OFFSET);
    g_app.on_threshold_pct = kv_read_int(KV_ON_THRESHOLD,  DEFAULT_ON_THRESHOLD);
    g_app.off_threshold_pct= kv_read_int(KV_OFF_THRESHOLD, DEFAULT_OFF_THRESHOLD);
    g_app.pump_mode        = kv_read_int(KV_PUMP_MODE,     0);
}

/* ========== GPIO Control ========== */

static void relay_set(BOOL_T on)
{
    tkl_gpio_write(PIN_RELAY, on ? TUYA_GPIO_LEVEL_LOW : TUYA_GPIO_LEVEL_HIGH);
    tkl_gpio_write(PIN_LED_PUMP, on ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW);
    g_app.pump_running = on;
    pump_state_save();
    g_ui_dirty = TRUE;
    /* Just flag — sensor_task will push it to the cloud within ~3s,
     * sending ONLY the pump-state DPs (2 records, not 9). */
    request_report_now();
}

static void gpio_init(void)
{
    TUYA_GPIO_BASE_CFG_T relay_cfg = { .mode = TUYA_GPIO_PUSH_PULL, .direct = TUYA_GPIO_OUTPUT, .level = TUYA_GPIO_LEVEL_HIGH };
    tkl_gpio_init(PIN_RELAY, &relay_cfg);
    TUYA_GPIO_BASE_CFG_T led_cfg = { .mode = TUYA_GPIO_PUSH_PULL, .direct = TUYA_GPIO_OUTPUT, .level = TUYA_GPIO_LEVEL_LOW };
    tkl_gpio_init(PIN_LED_PUMP, &led_cfg);
    TUYA_GPIO_BASE_CFG_T btn_cfg = { .mode = TUYA_GPIO_PULLUP, .direct = TUYA_GPIO_INPUT, .level = TUYA_GPIO_LEVEL_HIGH };
    tkl_gpio_init(PIN_BTN_PUMP, &btn_cfg);
    tkl_gpio_init(PIN_BTN_UP, &btn_cfg);
    tkl_gpio_init(PIN_BTN_DOWN, &btn_cfg);
    tkl_gpio_init(PIN_BTN_SELECT, &btn_cfg);
}

/* ========== Power Recovery ========== */

static void power_recovery_check(void)
{
    int saved = kv_read_int(KV_PUMP_STATE, 0);
    if (saved) {
        g_app.power_recovery = TRUE;
        for (int i = POWER_RECOVERY_DELAY_SEC; i > 0; i--) {
            tkl_gpio_write(PIN_LED_PUMP, TUYA_GPIO_LEVEL_HIGH); tal_system_sleep(500);
            tkl_gpio_write(PIN_LED_PUMP, TUYA_GPIO_LEVEL_LOW);  tal_system_sleep(500);
        }
        g_app.power_recovery = FALSE;
        relay_set(TRUE);
    }
}

/* ========== A02YYUW Sensor ========== */

static TUYA_UART_NUM_E g_uart_num = TUYA_UART_NUM_1;

static void sensor_uart_init(void)
{
    extern void __tkl_uart1_set_txd_pin(TUYA_PIN_NAME_E pin);
    extern void __tkl_uart1_set_rxd_pin(TUYA_PIN_NAME_E pin);
    __tkl_uart1_set_txd_pin(17);
    __tkl_uart1_set_rxd_pin(16);
    TAL_UART_CFG_T uart_cfg = {
        .rx_buffer_size = 256, .open_mode = 0,
        .base_cfg = { .baudrate = 9600, .parity = TUYA_UART_PARITY_TYPE_NONE,
                      .databits = TUYA_UART_DATA_LEN_8BIT, .stopbits = TUYA_UART_STOP_LEN_1BIT,
                      .flowctrl = TUYA_UART_FLOWCTRL_NONE },
    };
    tal_uart_init(g_uart_num, &uart_cfg);
}

/* Read one A02YYUW frame: 0xFF, H, L, checksum. Returns distance in cm or -1. */
static int sensor_read_distance(void)
{
    uint8_t discard[64], buf[4];
    int idx = 0, attempts = 0, n;

    /* Drain anything stale in the buffer before fishing for a fresh frame. */
    do { n = tal_uart_read(g_uart_num, discard, sizeof(discard)); } while (n > 0);
    tal_system_sleep(150);   /* let a fresh frame arrive (sensor period ~100 ms) */

    /* Bounded loop — attempts now increments on EVERY iteration, so a stream
     * of non-0xFF garbage can no longer spin forever. */
    while (attempts < 100) {
        uint8_t byte;
        int len = tal_uart_read(g_uart_num, &byte, 1);
        attempts++;
        if (len <= 0) { tal_system_sleep(10); continue; }
        if (idx == 0 && byte != 0xFF) continue;       /* hunt for header */
        buf[idx++] = byte;
        if (idx == 4) {
            if (((buf[0] + buf[1] + buf[2]) & 0xFF) == buf[3])
                return ((buf[1] << 8) | buf[2]) / 10;  /* mm -> cm */
            idx = 0;   /* checksum bad — resync */
        }
    }
    return -1;
}

/* ========================================================================
 * KALMAN FILTER FOR SENSOR NOISE
 * ======================================================================== */

#define KALMAN_R        4.0f    /* measurement noise covariance (variance) */
#define KALMAN_Q        0.5f    /* process noise covariance */
#define KALMAN_DT_SEC   3.0f    /* time step aligned with 3s sleep in sensor_task */

static float g_xhat = -1.0f;    /* Kalman state estimate (cm), -1 means uninitialized */
static float g_p = 500.0f;      /* Kalman error covariance (cm^2) */

/* Predict the state ahead based on the process model. */
static void kalman_predict(void) {
    g_p += KALMAN_Q * KALMAN_DT_SEC;
}

/* Update the Kalman state based on a new sensor reading (cm). */
static void kalman_update(float z) {
    /* 1. Initialize immediately on first valid read to prevent 0-to-target crawl lag */
    if (g_xhat < 0.0f) {
        g_xhat = z;
        g_p = KALMAN_R;
        return;
    }

    /* 2. Fast-track large changes (>15cm jump) to eliminate stabilization delay 
     * when a sudden, real physical change happens in the tank. */
    if (fabsf(z - g_xhat) > 15.0f) {
        g_xhat = z;
        g_p = KALMAN_R;
        return;
    }

    /* 3. Standard Kalman smoothing for minor steady-state jitter */
    float k = g_p / (g_p + KALMAN_R);   /* Kalman gain */
    g_xhat += k * (z - g_xhat);         /* state update */
    g_p *= (1.0f - k);                  /* covariance update */
}

/* Read the A02YYUW sensor, update the Kalman filter, return the new
 * estimated distance in cm. */
static int sensor_filtered_distance(void) {
    int dist = sensor_read_distance();

    /* Advance filter time by 1 loop iteration (3 seconds) */
    kalman_predict();

    if (dist > 0) {
        kalman_update((float)dist);
    }

    return g_xhat > 0.0f ? (int)roundf(g_xhat) : -1;
}

/* ========== Water Level & Pump Control ========== */

static int calculate_water_level(int distance_cm)
{
    int wh = g_app.tank_height_cm + g_app.sensor_offset_cm - distance_cm;
    if (wh < 0) wh = 0;
    if (wh > g_app.tank_height_cm) wh = g_app.tank_height_cm;
    int pct = (int)((long)wh * 100 / g_app.tank_height_cm);
    return (pct < 0) ? 0 : (pct > 100) ? 100 : pct;
}

static void pump_auto_control(void)
{
    if (g_app.pump_mode != 0 || g_app.power_recovery) return;
    if (g_app.water_level_pct <= g_app.on_threshold_pct) {
        if (g_app.manual_override) g_app.manual_override = FALSE;
        if (!g_app.pump_running) relay_set(TRUE);
    } else if (g_app.water_level_pct >= g_app.off_threshold_pct) {
        if (g_app.manual_override) g_app.manual_override = FALSE;
        if (g_app.pump_running) relay_set(FALSE);
    }
}

/* ========== Handle DPs from App ========== */

static void handle_dp_receive(tuya_iot_client_t *c, dp_obj_recv_t *dpobj)
{
    BOOL_T changed = FALSE;
    for (uint32_t i = 0; i < dpobj->dpscnt; i++) {
        dp_obj_t *dp = &dpobj->dps[i];
        switch (dp->id) {
        case DPID_SWITCH:
            if (g_app.pump_mode == 1) { relay_set(dp->value.dp_bool); }
            else { int lvl = g_app.water_level_pct;
                   if (lvl > g_app.on_threshold_pct && lvl < g_app.off_threshold_pct)
                   { g_app.manual_override = TRUE; relay_set(dp->value.dp_bool); } }
            break;
        case DPID_PUMP_MODE:     g_app.pump_mode = dp->value.dp_enum;     g_app.manual_override = FALSE; changed = TRUE; if (!g_app.pump_mode) pump_auto_control(); break;
        case DPID_ON_THRESHOLD:  g_app.on_threshold_pct  = dp->value.dp_value; changed = TRUE; break;
        case DPID_OFF_THRESHOLD: g_app.off_threshold_pct = dp->value.dp_value; changed = TRUE; break;
        case DPID_TANK_HEIGHT:   g_app.tank_height_cm    = dp->value.dp_value; changed = TRUE; break;
        case DPID_SENSOR_OFFSET: g_app.sensor_offset_cm  = dp->value.dp_value; changed = TRUE; break;
        default: break;
        }
    }
    if (changed) {
        settings_save();
        if (g_app.raw_distance_cm > 0) g_app.water_level_pct = calculate_water_level(g_app.raw_distance_cm);
        g_ui_dirty = TRUE; g_full_redraw = TRUE;
    }
    /* Echo received DPs back to confirm receipt. Our smart reporter will
     * see the values now match the cache and NOT re-send them. */
    tuya_iot_dp_obj_report(c, dpobj->devid, dpobj->dps, dpobj->dpscnt, 0);
}

/* ========== Tuya Event Handler ========== */

static void user_event_handler_on(tuya_iot_client_t *c, tuya_event_msg_t *event)
{
    switch (event->id) {
    case TUYA_EVENT_MQTT_CONNECTED:
        /* Force a full snapshot to cloud on (re)connect. */
        request_report_now();
        report_dps_smart();
        break;
    case TUYA_EVENT_DP_RECEIVE_OBJ: handle_dp_receive(c, event->value.dpobj); break;
    case TUYA_EVENT_RESET_COMPLETE: tal_system_reset(); break;
    default: break;
    }
}

static bool user_network_check(void)
{
    netmgr_status_e status = NETMGR_LINK_DOWN;
    netmgr_conn_get(NETCONN_AUTO, NETCONN_CMD_STATUS, &status);
    return status != NETMGR_LINK_DOWN;
}

/* ========== Tank Drawing ========== */

#define TANK_BX  70
#define TANK_BW  100
#define TANK_CX  (TANK_BX + TANK_BW / 2)
#define TANK_BY  60
#define TANK_BH  160
#define TANK_IX  (TANK_BX + 2)
#define TANK_IW  (TANK_BW - 4)
#define TANK_IY  (TANK_BY + 2)
#define TANK_IH  (TANK_BH - 4)
#define TANK_COLOR ILI_CYAN

static void draw_tank_dome(void)
{
    static const struct { int dy; int h; int w; } steps[] = {
        { 6,6,TANK_BW-10}, {12,6,TANK_BW-28}, {18,6,TANK_BW-48}, {25,7,TANK_BW-72},
    };
    for (int i=0;i<4;i++) {
        int sx=TANK_CX-steps[i].w/2, sy=TANK_BY-steps[i].dy;
        ili9341_fill_rect(sx,sy,steps[i].w,steps[i].h,ILI_NAVY);
        ili9341_draw_rect(sx,sy,steps[i].w,steps[i].h,TANK_COLOR);
    }
    ili9341_fill_rect(TANK_CX-4, TANK_BY-29, 8, 5, ILI_ORANGE);
}

static void draw_tank_body_outline(void)
{
    ili9341_draw_rect(TANK_BX, TANK_BY, TANK_BW, TANK_BH, TANK_COLOR);
    ili9341_draw_rect(TANK_BX+1, TANK_BY+1, TANK_BW-2, TANK_BH-2, TANK_COLOR);
    ili9341_hline(TANK_BX, TANK_IY, TANK_BW, TANK_COLOR);
}

static void draw_tank_fill(void)
{
    int fill_h = (g_app.water_level_pct * TANK_IH) / 100;
    int empty_h = TANK_IH - fill_h;
    uint16_t wc = g_app.water_level_pct <= g_app.on_threshold_pct ? ILI_ORANGE : ILI_WATER;

    if (empty_h > 0) ili9341_fill_rect(TANK_IX, TANK_IY, TANK_IW, empty_h, ILI_BLACK);
    if (fill_h > 0)  ili9341_fill_rect(TANK_IX, TANK_IY+empty_h, TANK_IW, fill_h, wc);

    ili9341_hline(TANK_IX, TANK_IY, TANK_IW, TANK_COLOR);

    char pct_buf[5];
    snprintf(pct_buf, sizeof(pct_buf), "%d%%", g_app.water_level_pct);
    int pct_x = TANK_CX - (strlen(pct_buf)*12)/2;
    int pct_y = TANK_BY + TANK_BH/2 - 8;

    ili9341_fill_rect(TANK_IX+2, pct_y-2, TANK_IW-4, 18, ILI_NAVY);
    ili9341_draw_string(pct_x, pct_y, pct_buf, ILI_WHITE, ILI_NAVY, 2);
}

/* ========== Home Screen ========== */

static void display_home_full(void)
{
    if (!g_app.display_ready) return;
    ili9341_fill_screen(ILI_BLACK);
    ili9341_fill_rect(0,0,240,24,ILI_NAVY);
    ili9341_draw_string(8,5,"WATER TANK",ILI_WHITE,ILI_NAVY,2);
    ili9341_draw_string(180,5,g_app.pump_mode?"MAN ":"AUTO",ILI_CYAN,ILI_NAVY,2);
    draw_tank_dome();
    draw_tank_body_outline();
    draw_tank_fill();
    ili9341_draw_string(60,240,"PUMP:",ILI_GRAY,ILI_BLACK,2);
    ili9341_draw_string(140,240,g_app.pump_running?"ON ":"OFF",
                       g_app.pump_running?ILI_GREEN:ILI_RED,ILI_BLACK,2);
    if (g_app.fault_bits & 0x01) {
        ili9341_fill_rect(0,275,240,16,ILI_RED);
        ili9341_draw_string(45,278,"! SENSOR FAULT !",ILI_WHITE,ILI_RED,1);
    }
    ili9341_hline(0,295,240,ILI_DARKGRAY);
    ili9341_draw_string(10,305,"[SEL] Menu",ILI_DARKGRAY,ILI_BLACK,1);
    ili9341_draw_string(130,305,"[PUMP] Toggle",ILI_DARKGRAY,ILI_BLACK,1);
}

static void display_home_update(void)
{
    if (!g_app.display_ready) return;
    ili9341_draw_string(180,5,g_app.pump_mode?"MAN ":"AUTO",ILI_CYAN,ILI_NAVY,2);
    draw_tank_fill();
    ili9341_draw_string(140,240,g_app.pump_running?"ON ":"OFF",
                       g_app.pump_running?ILI_GREEN:ILI_RED,ILI_BLACK,2);
    if (g_app.fault_bits & 0x01) {
        ili9341_fill_rect(0,275,240,16,ILI_RED);
        ili9341_draw_string(45,278,"! SENSOR FAULT !",ILI_WHITE,ILI_RED,1);
    } else {
        ili9341_fill_rect(0,275,240,16,ILI_BLACK);
    }
}

/* ========== Settings Menu ========== */

#define MENU_ROW_Y(i) (30 + (i) * 38)
#define MENU_ROW_H    36

static void display_menu_item(int i)
{
    int y = MENU_ROW_Y(i);
    BOOL_T sel = (i == g_menu_sel);
    ili9341_fill_rect(0, y, 240, MENU_ROW_H, sel ? ILI_DARKGRAY : ILI_BLACK);
    uint16_t fg = sel ? ILI_WHITE : ILI_GRAY;
    uint16_t bg = sel ? ILI_DARKGRAY : ILI_BLACK;
    ili9341_draw_string(6, y+6, sel ? ">" : " ", ILI_CYAN, bg, 2);
    ili9341_draw_string(24, y+6, menu_labels[i], fg, bg, 2);
    char vb[20]; vb[0]='\0';
    switch (i) {
    case 0: snprintf(vb,sizeof(vb),"%d cm",g_app.tank_height_cm); break;
    case 1: snprintf(vb,sizeof(vb),"%d cm",g_app.sensor_offset_cm); break;
    case 2: snprintf(vb,sizeof(vb),"%d %%",g_app.on_threshold_pct); break;
    case 3: snprintf(vb,sizeof(vb),"%d %%",g_app.off_threshold_pct); break;
    case 4: snprintf(vb,sizeof(vb),"%s",g_app.pump_mode?"MANUAL":"AUTO"); break;
    case 5: snprintf(vb,sizeof(vb),"%d cm",g_app.raw_distance_cm); break;
    case 6: break;
    }
    if (vb[0]) ili9341_draw_string(24, y+24, vb, ILI_YELLOW, bg, 1);
}

static void display_menu_full(void)
{
    if (!g_app.display_ready) return;
    ili9341_fill_screen(ILI_BLACK);
    ili9341_fill_rect(0,0,240,24,ILI_NAVY);
    ili9341_draw_string(8,5,"SETTINGS",ILI_CYAN,ILI_NAVY,2);
    for (int i=0; i<MENU_ITEMS; i++) display_menu_item(i);
    ili9341_draw_string(8,305,"UP/DN nav  SEL edit  PUMP back",ILI_DARKGRAY,ILI_BLACK,1);
    g_prev_menu_sel = g_menu_sel;
}

static void display_menu_nav(void)
{
    if (!g_app.display_ready) return;
    if (g_prev_menu_sel >= 0 && g_prev_menu_sel < MENU_ITEMS && g_prev_menu_sel != g_menu_sel) {
        display_menu_item(g_prev_menu_sel);
    }
    display_menu_item(g_menu_sel);
    g_prev_menu_sel = g_menu_sel;
}

/* ========== Edit Screen ========== */

static void display_edit_value(void);

static void display_edit_full(void)
{
    if (!g_app.display_ready) return;
    ili9341_fill_screen(ILI_BLACK);
    ili9341_fill_rect(0,0,240,24,ILI_NAVY);
    ili9341_draw_string(8,5,"EDIT VALUE",ILI_CYAN,ILI_NAVY,2);
    if (g_edit_item >= 0 && g_edit_item < MENU_ITEMS)
        ili9341_draw_string(20, 50, menu_labels[g_edit_item], ILI_WHITE, ILI_BLACK, 2);
    display_edit_value();
    ili9341_hline(0,220,240,ILI_DARKGRAY);
    ili9341_draw_string(20,240,"UP    = increase",ILI_GRAY,ILI_BLACK,2);
    ili9341_draw_string(20,265,"DOWN  = decrease",ILI_GRAY,ILI_BLACK,2);
    ili9341_draw_string(20,290,"SEL   = save",ILI_GREEN,ILI_BLACK,2);
    ili9341_draw_string(20,308,"PUMP  = cancel",ILI_RED,ILI_BLACK,1);
}

static void display_edit_value(void)
{
    if (!g_app.display_ready) return;
    ili9341_fill_rect(40, 100, 180, 35, ILI_BLACK);
    ili9341_draw_number(40, 100, g_edit_val, ILI_GREEN, ILI_BLACK, 5);
    ili9341_fill_rect(40, 145, 180, 21, ILI_BLACK);
    if (g_edit_item == 4) {
        ili9341_draw_string(40,145, g_edit_val ? "MANUAL" : "AUTO  ", ILI_YELLOW, ILI_BLACK, 3);
    } else {
        const char *unit = "";
        if (g_edit_item <= 1) unit = "cm";
        else if (g_edit_item <= 3) unit = "%";
        ili9341_draw_string(40, 145, unit, ILI_GRAY, ILI_BLACK, 3);
    }
}

/* ========== Button Helper ========== */

static BOOL_T btn_read(int pin)
{
    TUYA_GPIO_LEVEL_E level;
    tkl_gpio_read(pin, &level);
    return (level == TUYA_GPIO_LEVEL_LOW);
}

/* ========== Sensor Task ========== */

static void sensor_task(void *arg)
{
    while (g_app.power_recovery) tal_system_sleep(500);

    while (1) {
        int dist = sensor_filtered_distance();   /* Kalman filtered */
        if (dist > 0) {
            g_app.raw_distance_cm = dist;
            g_app.water_level_pct = calculate_water_level(dist);
            g_app.sensor_fail_count = 0;
            if (g_app.fault_bits & 0x01) {
                g_app.fault_bits &= ~0x01;
                g_ui_dirty = TRUE;
            }
            pump_auto_control();
            g_ui_dirty = TRUE;
        } else {
            g_app.sensor_fail_count++;
            if (g_app.sensor_fail_count > 5 && !(g_app.fault_bits & 0x01)) {
                g_app.fault_bits |= 0x01;
                g_ui_dirty = TRUE;
            }
            if (g_app.pump_running && g_app.pump_mode == 0 && g_app.sensor_fail_count > 15) {
                relay_set(FALSE);
            }
        }

        /* Smart reporter — decides on its own whether anything is worth sending. */
        report_dps_smart();

        tal_system_sleep(3000);
    }
}

/* ========== UI Task ========== */

static void ui_task(void *arg)
{
    while (!g_app.display_ready) tal_system_sleep(100);
    display_home_full();

    BOOL_T pump_last=TRUE, up_last=TRUE, down_last=TRUE, sel_last=TRUE;
    uint32_t pump_press_time=0, sel_press_time=0;

    while (1) {
        BOOL_T pump_now = btn_read(PIN_BTN_PUMP);
        BOOL_T up_now   = btn_read(PIN_BTN_UP);
        BOOL_T down_now = btn_read(PIN_BTN_DOWN);
        BOOL_T sel_now  = btn_read(PIN_BTN_SELECT);

        /* PUMP BUTTON */
        if (pump_now && !pump_last) pump_press_time = tal_system_get_millisecond();
        if (!pump_now && pump_last && pump_press_time > 0) {
            uint32_t dur = tal_system_get_millisecond() - pump_press_time;
            if (g_screen == SCREEN_EDIT) {
                g_screen = SCREEN_MENU; g_menu_full = TRUE; g_ui_dirty = TRUE;
            } else if (g_screen == SCREEN_MENU) {
                g_screen = SCREEN_HOME; g_full_redraw = TRUE; g_ui_dirty = TRUE;
            } else if (dur > 50 && dur < 3000) {
                if (g_app.pump_mode == 1) { relay_set(!g_app.pump_running); }
                else { int lvl=g_app.water_level_pct;
                       if (lvl>g_app.on_threshold_pct && lvl<g_app.off_threshold_pct)
                       { g_app.manual_override=TRUE; relay_set(!g_app.pump_running); } }
            } else if (dur >= 3000) {
                g_app.pump_mode = g_app.pump_mode ? 0 : 1;
                g_app.manual_override = FALSE;
                settings_save();
                request_report_now();   /* mode change — push within 3s */
                g_ui_dirty = TRUE;
            }
            pump_press_time = 0;
        }

        /* SELECT BUTTON */
        if (sel_now && !sel_last) sel_press_time = tal_system_get_millisecond();
        if (!sel_now && sel_last && sel_press_time > 0) {
            uint32_t dur = tal_system_get_millisecond() - sel_press_time;
            if (dur > 50) {
                switch (g_screen) {
                case SCREEN_HOME:
                    g_screen = SCREEN_MENU; g_menu_sel = 0; g_menu_full = TRUE; g_ui_dirty = TRUE;
                    break;
                case SCREEN_MENU:
                    if (g_menu_sel == 6) { g_screen = SCREEN_HOME; g_full_redraw = TRUE; }
                    else if (g_menu_sel == 5) { /* read-only */ }
                    else if (g_menu_sel == 4) {
                        g_app.pump_mode = g_app.pump_mode ? 0 : 1;
                        g_app.manual_override = FALSE;
                        settings_save();
                        request_report_now();
                        if (!g_app.pump_mode) pump_auto_control();
                        display_menu_item(4); g_prev_menu_sel = g_menu_sel;
                    } else {
                        g_edit_item = g_menu_sel;
                        switch (g_edit_item) {
                        case 0: g_edit_val=g_app.tank_height_cm; break;
                        case 1: g_edit_val=g_app.sensor_offset_cm; break;
                        case 2: g_edit_val=g_app.on_threshold_pct; break;
                        case 3: g_edit_val=g_app.off_threshold_pct; break;
                        }
                        g_screen = SCREEN_EDIT; g_edit_full = TRUE;
                    }
                    g_ui_dirty = TRUE;
                    break;
                case SCREEN_EDIT:
                    switch (g_edit_item) {
                    case 0: g_app.tank_height_cm    = g_edit_val; break;
                    case 1: g_app.sensor_offset_cm  = g_edit_val; break;
                    case 2: g_app.on_threshold_pct  = g_edit_val; break;
                    case 3: g_app.off_threshold_pct = g_edit_val; break;
                    }
                    settings_save();
                    if (g_app.raw_distance_cm > 0)
                        g_app.water_level_pct = calculate_water_level(g_app.raw_distance_cm);
                    request_report_now();
                    g_screen = SCREEN_MENU; g_menu_full = TRUE; g_ui_dirty = TRUE;
                    break;
                }
            }
            sel_press_time = 0;
        }

        /* UP BUTTON */
        if (up_now && !up_last) {
            switch (g_screen) {
            case SCREEN_MENU:
                if (g_menu_sel > 0) g_menu_sel--;
                g_ui_dirty = TRUE;
                break;
            case SCREEN_EDIT:
                if (g_edit_item==4) g_edit_val = g_edit_val ? 0 : 1;
                else g_edit_val++;
                g_ui_dirty = TRUE;
                break;
            default: break;
            }
        }

        /* DOWN BUTTON */
        if (down_now && !down_last) {
            switch (g_screen) {
            case SCREEN_MENU:
                if (g_menu_sel < MENU_ITEMS-1) g_menu_sel++;
                g_ui_dirty = TRUE;
                break;
            case SCREEN_EDIT:
                if (g_edit_item==4) g_edit_val = g_edit_val ? 0 : 1;
                else g_edit_val--;
                if (g_edit_item==0 && g_edit_val<10) g_edit_val=10;
                if (g_edit_item==1 && g_edit_val<0)  g_edit_val=0;
                if (g_edit_item==2 && g_edit_val<5)  g_edit_val=5;
                if (g_edit_item==3 && g_edit_val<10) g_edit_val=10;
                g_ui_dirty = TRUE;
                break;
            default: break;
            }
        }

        pump_last=pump_now; up_last=up_now; down_last=down_now; sel_last=sel_now;

        /* ===== REDRAW ===== */
        if (g_ui_dirty) {
            switch (g_screen) {
            case SCREEN_HOME:
                if (g_full_redraw) {
                    display_home_full();
                    g_full_redraw = FALSE;
                    g_disp_level = g_app.water_level_pct;
                    g_disp_pump  = g_app.pump_running;
                    g_disp_mode  = g_app.pump_mode;
                    g_disp_fault = (g_app.fault_bits & 0x01) ? 1 : 0;
                } else {
                    int cur_fault = (g_app.fault_bits & 0x01) ? 1 : 0;
                    if (g_app.water_level_pct != g_disp_level ||
                        g_app.pump_running     != g_disp_pump  ||
                        g_app.pump_mode        != g_disp_mode  ||
                        cur_fault              != g_disp_fault) {

                        display_home_update();
                        g_disp_level = g_app.water_level_pct;
                        g_disp_pump  = g_app.pump_running;
                        g_disp_mode  = g_app.pump_mode;
                        g_disp_fault = cur_fault;
                    }
                }
                break;
            case SCREEN_MENU:
                if (g_menu_full) { display_menu_full(); g_menu_full=FALSE; }
                else display_menu_nav();
                break;
            case SCREEN_EDIT:
                if (g_edit_full) { display_edit_full(); g_edit_full=FALSE; }
                else display_edit_value();
                break;
            }
            g_ui_dirty = FALSE;
        }

        tal_system_sleep(50);
    }
}

/* ========== user_main ========== */

static void user_main(void)
{
    cJSON_InitHooks(&(cJSON_Hooks){.malloc_fn = tal_malloc, .free_fn = tal_free});
    tal_log_init(TAL_LOG_LEVEL_DEBUG, 1024, (TAL_LOG_OUTPUT_CB)tkl_log_output);
    tal_kv_init(&(tal_kv_cfg_t){ .seed = "vmlkasdh93dlvlcy", .key = "dflfuap134ddlduq" });
    tal_sw_timer_init();
    tal_workq_init();

    gpio_init();

    ili9341_init(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST, PIN_TFT_MOSI, PIN_TFT_SCK, PIN_TFT_BL);
    ili9341_fill_screen(ILI_BLACK);
    ili9341_draw_string(20,80,"Water Tank",ILI_CYAN,ILI_BLACK,3);
    ili9341_draw_string(20,115,"Controller",ILI_CYAN,ILI_BLACK,3);
    ili9341_draw_string(20,160,"v1.2.6",ILI_GRAY,ILI_BLACK,2);
    ili9341_draw_string(20,200,"Booting...",ILI_YELLOW,ILI_BLACK,2);
    g_app.display_ready = TRUE;

    sensor_uart_init();
    settings_load();
    power_recovery_check();

    if (OPRT_OK != tuya_authorize_read(&license)) {
        license.uuid = TUYA_DEVICE_UUID;
        license.authkey = TUYA_DEVICE_AUTHKEY;
    }

    int rt = tuya_iot_init(&client, &(const tuya_iot_config_t){
        .software_ver = "1.2.6", .productkey = TUYA_PRODUCT_ID,
        .uuid = license.uuid, .authkey = license.authkey,
        .event_handler = user_event_handler_on, .network_check = user_network_check,
    });
    if (rt != OPRT_OK) { ili9341_draw_string(20,240,"TUYA INIT FAIL!",ILI_RED,ILI_BLACK,2); return; }

    netmgr_type_e type = 0;
#if defined(ENABLE_WIFI) && (ENABLE_WIFI == 1)
    type |= NETCONN_WIFI;
    netmgr_init(type);
    netmgr_conn_set(NETCONN_WIFI, NETCONN_CMD_NETCFG,
                     &(netcfg_args_t){.type = NETCFG_TUYA_BLE | NETCFG_TUYA_WIFI_AP});
#else
    netmgr_init(type);
#endif

    tuya_iot_start(&client);

    THREAD_CFG_T s_cfg = { .stackDepth=4096, .priority=THREAD_PRIO_3, .thrdname="sensor" };
    THREAD_HANDLE s_thrd=NULL;
    tal_thread_create_and_start(&s_thrd, NULL, NULL, sensor_task, NULL, &s_cfg);

    THREAD_CFG_T u_cfg = { .stackDepth=4096, .priority=THREAD_PRIO_3, .thrdname="ui" };
    THREAD_HANDLE u_thrd=NULL;
    tal_thread_create_and_start(&u_thrd, NULL, NULL, ui_task, NULL, &u_cfg);

    for (;;) tuya_iot_yield(&client);
}

/* ========== Entry point ========== */

static THREAD_HANDLE ty_app_thread = NULL;
static void tuya_app_thread_fn(void *arg) { user_main(); tal_thread_delete(ty_app_thread); ty_app_thread=NULL; }

void tuya_app_main(void)
{
    THREAD_CFG_T p = { .stackDepth=4096, .priority=THREAD_PRIO_1, .thrdname="tuya_app_main" };
    tal_thread_create_and_start(&ty_app_thread, NULL, NULL, tuya_app_thread_fn, NULL, &p);
}