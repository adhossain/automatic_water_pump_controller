/**
 * @file ili9341.c
 * @brief ILI9341 display driver — Optimized Software SPI
 *
 *  Optimizations vs original:
 *   1. draw_char renders entire character cell into a buffer and sends it
 *      as ONE windowed transfer, not 35+ separate fill_rect calls.
 *      This eliminates per-pixel window-set overhead (~15x faster text).
 *   2. fill_rect watchdog sleep only fires on large fills (h >= 24, row > 0).
 *      Original slept 2ms on row 0 of EVERY fill — each char pixel cost 2ms.
 *
 * 240x320 pixels, 16-bit color (RGB565), portrait orientation.
 */

#include "ili9341.h"
#include "tkl_gpio.h"
#include "tal_system.h"
#include <string.h>

static int s_dc_pin = -1;
static int s_mosi_pin = -1;
static int s_sclk_pin = -1;

/* Scratch buffer for batched character rendering.
 * Max size=6 → (6×6)×(7×6) = 36×42 = 1512 pixels → 3024 bytes.
 * Static BSS, does not use task stack. */
#define CHAR_CELL_MAX_BYTES (36 * 42 * 2)
static uint8_t s_char_buf[CHAR_CELL_MAX_BYTES];

/* ========== 5x7 ASCII Font (chars 32-126) ========== */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 space
    {0x00,0x00,0x5F,0x00,0x00}, // 33 !
    {0x00,0x07,0x00,0x07,0x00}, // 34 "
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
    {0x23,0x13,0x08,0x64,0x62}, // 37 %
    {0x36,0x49,0x55,0x22,0x50}, // 38 &
    {0x00,0x05,0x03,0x00,0x00}, // 39 '
    {0x00,0x1C,0x22,0x41,0x00}, // 40 (
    {0x00,0x41,0x22,0x1C,0x00}, // 41 )
    {0x08,0x2A,0x1C,0x2A,0x08}, // 42 *
    {0x08,0x08,0x3E,0x08,0x08}, // 43 +
    {0x00,0x50,0x30,0x00,0x00}, // 44 ,
    {0x08,0x08,0x08,0x08,0x08}, // 45 -
    {0x00,0x60,0x60,0x00,0x00}, // 46 .
    {0x20,0x10,0x08,0x04,0x02}, // 47 /
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
    {0x00,0x42,0x7F,0x40,0x00}, // 49 1
    {0x42,0x61,0x51,0x49,0x46}, // 50 2
    {0x21,0x41,0x45,0x4B,0x31}, // 51 3
    {0x18,0x14,0x12,0x7F,0x10}, // 52 4
    {0x27,0x45,0x45,0x45,0x39}, // 53 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
    {0x01,0x71,0x09,0x05,0x03}, // 55 7
    {0x36,0x49,0x49,0x49,0x36}, // 56 8
    {0x06,0x49,0x49,0x29,0x1E}, // 57 9
    {0x00,0x36,0x36,0x00,0x00}, // 58 :
    {0x00,0x56,0x36,0x00,0x00}, // 59 ;
    {0x00,0x08,0x14,0x22,0x41}, // 60 <
    {0x14,0x14,0x14,0x14,0x14}, // 61 =
    {0x41,0x22,0x14,0x08,0x00}, // 62 >
    {0x02,0x01,0x51,0x09,0x06}, // 63 ?
    {0x32,0x49,0x79,0x41,0x3E}, // 64 @
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
    {0x7F,0x49,0x49,0x49,0x36}, // 66 B
    {0x3E,0x41,0x41,0x41,0x22}, // 67 C
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
    {0x7F,0x49,0x49,0x49,0x41}, // 69 E
    {0x7F,0x09,0x09,0x01,0x01}, // 70 F
    {0x3E,0x41,0x41,0x51,0x32}, // 71 G
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
    {0x00,0x41,0x7F,0x41,0x00}, // 73 I
    {0x20,0x40,0x41,0x3F,0x01}, // 74 J
    {0x7F,0x08,0x14,0x22,0x41}, // 75 K
    {0x7F,0x40,0x40,0x40,0x40}, // 76 L
    {0x7F,0x02,0x04,0x02,0x7F}, // 77 M
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
    {0x7F,0x09,0x09,0x09,0x06}, // 80 P
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
    {0x7F,0x09,0x19,0x29,0x46}, // 82 R
    {0x46,0x49,0x49,0x49,0x31}, // 83 S
    {0x01,0x01,0x7F,0x01,0x01}, // 84 T
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
    {0x7F,0x20,0x18,0x20,0x7F}, // 87 W
    {0x63,0x14,0x08,0x14,0x63}, // 88 X
    {0x03,0x04,0x78,0x04,0x03}, // 89 Y
    {0x61,0x51,0x49,0x45,0x43}, // 90 Z
    {0x00,0x00,0x7F,0x41,0x41}, // 91 [
    {0x02,0x04,0x08,0x10,0x20}, // 92 backslash
    {0x41,0x41,0x7F,0x00,0x00}, // 93 ]
    {0x04,0x02,0x01,0x02,0x04}, // 94 ^
    {0x40,0x40,0x40,0x40,0x40}, // 95 _
    {0x00,0x01,0x02,0x04,0x00}, // 96 `
    {0x20,0x54,0x54,0x54,0x78}, // 97 a
    {0x7F,0x48,0x44,0x44,0x38}, // 98 b
    {0x38,0x44,0x44,0x44,0x20}, // 99 c
    {0x38,0x44,0x44,0x48,0x7F}, // 100 d
    {0x38,0x54,0x54,0x54,0x18}, // 101 e
    {0x08,0x7E,0x09,0x01,0x02}, // 102 f
    {0x08,0x14,0x54,0x54,0x3C}, // 103 g
    {0x7F,0x08,0x04,0x04,0x78}, // 104 h
    {0x00,0x44,0x7D,0x40,0x00}, // 105 i
    {0x20,0x40,0x44,0x3D,0x00}, // 106 j
    {0x00,0x7F,0x10,0x28,0x44}, // 107 k
    {0x00,0x41,0x7F,0x40,0x00}, // 108 l
    {0x7C,0x04,0x18,0x04,0x78}, // 109 m
    {0x7C,0x08,0x04,0x04,0x78}, // 110 n
    {0x38,0x44,0x44,0x44,0x38}, // 111 o
    {0x7C,0x14,0x14,0x14,0x08}, // 112 p
    {0x08,0x14,0x14,0x18,0x7C}, // 113 q
    {0x7C,0x08,0x04,0x04,0x08}, // 114 r
    {0x48,0x54,0x54,0x54,0x20}, // 115 s
    {0x04,0x3F,0x44,0x40,0x20}, // 116 t
    {0x3C,0x40,0x40,0x20,0x7C}, // 117 u
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 v
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 w
    {0x44,0x28,0x10,0x28,0x44}, // 120 x
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 y
    {0x44,0x64,0x54,0x4C,0x44}, // 122 z
    {0x00,0x08,0x36,0x41,0x00}, // 123 {
    {0x00,0x00,0x7F,0x00,0x00}, // 124 |
    {0x00,0x41,0x36,0x08,0x00}, // 125 }
    {0x08,0x08,0x2A,0x1C,0x08}, // 126 ~
};

/* ========== Low-level Software SPI ========== */

static void soft_spi_transfer(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        uint8_t d = data[i];
        for (int b = 7; b >= 0; b--) {
            tkl_gpio_write(s_sclk_pin, TUYA_GPIO_LEVEL_LOW);
            tkl_gpio_write(s_mosi_pin, (d & (1 << b)) ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW);
            tkl_gpio_write(s_sclk_pin, TUYA_GPIO_LEVEL_HIGH);
        }
    }
}

static void ili_cmd(uint8_t cmd)
{
    tkl_gpio_write(s_dc_pin, TUYA_GPIO_LEVEL_LOW);
    soft_spi_transfer(&cmd, 1);
}

static void ili_data(const uint8_t *data, int len)
{
    if (len <= 0) return;
    tkl_gpio_write(s_dc_pin, TUYA_GPIO_LEVEL_HIGH);
    soft_spi_transfer(data, len);
}

static void ili_data8(uint8_t val)
{
    ili_data(&val, 1);
}

/* ========== Public API ========== */

void ili9341_init(int cs, int dc, int rst, int mosi, int sclk, int bl)
{
    s_dc_pin = dc;
    s_mosi_pin = mosi;
    s_sclk_pin = sclk;

    TUYA_GPIO_BASE_CFG_T out_cfg = {
        .mode = TUYA_GPIO_PUSH_PULL,
        .direct = TUYA_GPIO_OUTPUT,
        .level = TUYA_GPIO_LEVEL_HIGH
    };

    tkl_gpio_init(dc, &out_cfg);
    tkl_gpio_init(rst, &out_cfg);
    tkl_gpio_init(cs, &out_cfg);

    out_cfg.level = TUYA_GPIO_LEVEL_LOW;
    tkl_gpio_init(sclk, &out_cfg);
    tkl_gpio_init(mosi, &out_cfg);

    if (bl >= 0) {
        out_cfg.level = TUYA_GPIO_LEVEL_HIGH;
        tkl_gpio_init(bl, &out_cfg);
        tkl_gpio_write(bl, TUYA_GPIO_LEVEL_HIGH);
    }

    /* Hardware reset */
    tkl_gpio_write(rst, TUYA_GPIO_LEVEL_HIGH);
    tal_system_sleep(10);
    tkl_gpio_write(rst, TUYA_GPIO_LEVEL_LOW);
    tal_system_sleep(10);
    tkl_gpio_write(rst, TUYA_GPIO_LEVEL_HIGH);
    tal_system_sleep(120);

    tkl_gpio_write(cs, TUYA_GPIO_LEVEL_LOW);

    /* ILI9341 Init sequence */
    ili_cmd(0x01);
    tal_system_sleep(150);

    ili_cmd(0xCB); { uint8_t d[]={0x39,0x2C,0x00,0x34,0x02}; ili_data(d,5); }
    ili_cmd(0xCF); { uint8_t d[]={0x00,0xC1,0x30}; ili_data(d,3); }
    ili_cmd(0xE8); { uint8_t d[]={0x85,0x00,0x78}; ili_data(d,3); }
    ili_cmd(0xEA); { uint8_t d[]={0x00,0x00}; ili_data(d,2); }
    ili_cmd(0xED); { uint8_t d[]={0x64,0x03,0x12,0x81}; ili_data(d,4); }
    ili_cmd(0xF7); ili_data8(0x20);
    ili_cmd(0xC0); ili_data8(0x23);
    ili_cmd(0xC1); ili_data8(0x10);
    ili_cmd(0xC5); { uint8_t d[]={0x3E,0x28}; ili_data(d,2); }
    ili_cmd(0xC7); ili_data8(0x86);
    ili_cmd(0x36); ili_data8(0x48);
    ili_cmd(0x3A); ili_data8(0x55);
    ili_cmd(0xB1); { uint8_t d[]={0x00,0x18}; ili_data(d,2); }
    ili_cmd(0xB6); { uint8_t d[]={0x08,0x82,0x27}; ili_data(d,3); }
    ili_cmd(0xF2); ili_data8(0x00);
    ili_cmd(0x26); ili_data8(0x01);
    ili_cmd(0xE0); { uint8_t d[]={0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00}; ili_data(d,15); }
    ili_cmd(0xE1); { uint8_t d[]={0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F}; ili_data(d,15); }

    ili_cmd(0x11);
    tal_system_sleep(120);
    ili_cmd(0x29);
    tal_system_sleep(50);

    ili9341_fill_screen(ILI_BLACK);
}

static void ili_set_window(int x0, int y0, int x1, int y1)
{
    ili_cmd(0x2A);
    { uint8_t d[4]={x0>>8,x0,x1>>8,x1}; ili_data(d,4); }
    ili_cmd(0x2B);
    { uint8_t d[4]={y0>>8,y0,y1>>8,y1}; ili_data(d,4); }
    ili_cmd(0x2C);
}

void ili9341_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > ILI_WIDTH) w = ILI_WIDTH - x;
    if (y + h > ILI_HEIGHT) h = ILI_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    ili_set_window(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    uint8_t line[240 * 2];
    int row_bytes = w * 2;
    for (int i = 0; i < row_bytes; i += 2) {
        line[i] = hi;
        line[i + 1] = lo;
    }

    tkl_gpio_write(s_dc_pin, TUYA_GPIO_LEVEL_HIGH);
    for (int row = 0; row < h; row++) {
        soft_spi_transfer(line, row_bytes);

        /* Feed watchdog ONLY during large fills (full screen, bars, menu
         * highlights). Small fills must never sleep or text crawls. */
        if (h >= 24 && row > 0 && (row & 15) == 0) {
            tal_system_sleep(1);
        }
    }
}

void ili9341_fill_screen(uint16_t color)
{
    ili9341_fill_rect(0, 0, ILI_WIDTH, ILI_HEIGHT, color);
}

void ili9341_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int size)
{
    if (c < 32 || c > 126) c = '?';
    if (size < 1) size = 1;
    const uint8_t *glyph = font5x7[c - 32];

    int W = 6 * size;
    int H = 7 * size;

    /* FAST PATH: render entire character cell into buffer, send as one
     * windowed transfer. Replaces 35+ tiny fill_rect calls. */
    if (size <= 6 &&
        x >= 0 && y >= 0 &&
        (x + W) <= ILI_WIDTH && (y + H) <= ILI_HEIGHT &&
        (W * H * 2) <= (int)sizeof(s_char_buf))
    {
        uint8_t fg_hi = fg >> 8, fg_lo = fg & 0xFF;
        uint8_t bg_hi = bg >> 8, bg_lo = bg & 0xFF;
        int idx = 0;

        for (int sr = 0; sr < H; sr++) {
            int grow = sr / size;
            for (int sc = 0; sc < W; sc++) {
                int on = 0;
                if (sc < 5 * size) {
                    int gcol = sc / size;
                    on = (glyph[gcol] >> grow) & 0x01;
                }
                if (on) { s_char_buf[idx++] = fg_hi; s_char_buf[idx++] = fg_lo; }
                else    { s_char_buf[idx++] = bg_hi; s_char_buf[idx++] = bg_lo; }
            }
        }

        ili_set_window(x, y, x + W - 1, y + H - 1);
        tkl_gpio_write(s_dc_pin, TUYA_GPIO_LEVEL_HIGH);
        soft_spi_transfer(s_char_buf, W * H * 2);
        return;
    }

    /* Fallback */
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            uint16_t clr = (line & (1 << row)) ? fg : bg;
            if (size == 1) {
                ili9341_fill_rect(x + col, y + row, 1, 1, clr);
            } else {
                ili9341_fill_rect(x + col * size, y + row * size, size, size, clr);
            }
        }
    }
    ili9341_fill_rect(x + 5 * size, y, size, 7 * size, bg);
}

void ili9341_draw_string(int x, int y, const char *str, uint16_t fg, uint16_t bg, int size)
{
    while (*str) {
        ili9341_draw_char(x, y, *str++, fg, bg, size);
        x += 6 * size;
    }
}

void ili9341_draw_number(int x, int y, int num, uint16_t fg, uint16_t bg, int size)
{
    char buf[12];
    int len = 0, n = num;
    if (n < 0) { buf[len++] = '-'; n = -n; }
    if (n == 0) { buf[len++] = '0'; }
    else {
        char tmp[10]; int tl = 0;
        while (n > 0) { tmp[tl++] = '0' + (n % 10); n /= 10; }
        for (int i = tl - 1; i >= 0; i--) buf[len++] = tmp[i];
    }
    buf[len] = '\0';
    ili9341_draw_string(x, y, buf, fg, bg, size);
}

void ili9341_hline(int x, int y, int w, uint16_t color)
{
    ili9341_fill_rect(x, y, w, 1, color);
}

void ili9341_draw_rect(int x, int y, int w, int h, uint16_t color)
{
    ili9341_hline(x, y, w, color);
    ili9341_hline(x, y + h - 1, w, color);
    ili9341_fill_rect(x, y, 1, h, color);
    ili9341_fill_rect(x + w - 1, y, 1, h, color);
}