#ifndef TUYA_CONFIG_H
#define TUYA_CONFIG_H

/* ========== Tuya Product Info ========== */
#ifndef TUYA_PRODUCT_ID
#define TUYA_PRODUCT_ID     "xxx"
#endif

#define TUYA_DEVICE_UUID    "xxx"
#define TUYA_DEVICE_AUTHKEY "xxx"

/* ========== Tuya DP IDs ========== */
#define DPID_SWITCH         1
#define DPID_WATER_LEVEL    101
#define DPID_PUMP_RUNNING   102
#define DPID_PUMP_MODE      103
#define DPID_ON_THRESHOLD   104
#define DPID_OFF_THRESHOLD  105
#define DPID_TANK_HEIGHT    106
#define DPID_SENSOR_OFFSET  107
#define DPID_RAW_DISTANCE   108
#define DPID_FAULT          109

/* ========== Pin Assignments (ESP32 30-pin) ========== */

/* ST7735 1.8" Display (SPI) */
#define PIN_TFT_CS          5
#define PIN_TFT_DC          2
#define PIN_TFT_RST         4
#define PIN_TFT_MOSI        23
#define PIN_TFT_SCK         18
#define PIN_TFT_BL          15

/* Relay (active LOW) */
#define PIN_RELAY           27

/* LEDs */
#define PIN_LED_PUMP        13

/* Buttons (all active LOW with internal pull-up) */
#define PIN_BTN_PUMP        14     /* Pump toggle (short) / mode toggle (long) */
#define PIN_BTN_UP          25     /* Menu: navigate up / increase value */
#define PIN_BTN_DOWN        26     /* Menu: navigate down / decrease value */
#define PIN_BTN_SELECT      32     /* Menu: enter / confirm / back (long press) */

/* RS485 Sensor UART (remapped UART1) */
#define PIN_SENSOR_RX       16
#define PIN_SENSOR_TX       17

/* ========== Default Settings ========== */
#define DEFAULT_TANK_HEIGHT     150   /* cm */
#define DEFAULT_SENSOR_OFFSET     5   /* cm */
#define DEFAULT_ON_THRESHOLD     20   /* % */
#define DEFAULT_OFF_THRESHOLD    95   /* % */

/* ========== Power Recovery ========== */
#define POWER_RECOVERY_DELAY_SEC 10

#endif /* TUYA_CONFIG_H */