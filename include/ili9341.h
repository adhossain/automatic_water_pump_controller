#ifndef ILI9341_H
#define ILI9341_H

#include <stdint.h>

/* Colors (RGB565) */
#define ILI_BLACK    0x0000
#define ILI_WHITE    0xFFFF
#define ILI_RED      0xF800
#define ILI_GREEN    0x07E0
#define ILI_BLUE     0x001F
#define ILI_CYAN     0x07FF
#define ILI_YELLOW   0xFFE0
#define ILI_ORANGE   0xFD20
#define ILI_GRAY     0x7BEF
#define ILI_DARKGRAY 0x3186
#define ILI_NAVY     0x000F
#define ILI_WATER    0x2D7F

/* Display dimensions (portrait) */
#define ILI_WIDTH    240
#define ILI_HEIGHT   320

void ili9341_init(int cs, int dc, int rst, int mosi, int sclk, int bl);
void ili9341_fill_screen(uint16_t color);
void ili9341_fill_rect(int x, int y, int w, int h, uint16_t color);
void ili9341_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int size);
void ili9341_draw_string(int x, int y, const char *str, uint16_t fg, uint16_t bg, int size);
void ili9341_draw_number(int x, int y, int num, uint16_t fg, uint16_t bg, int size);
void ili9341_hline(int x, int y, int w, uint16_t color);
void ili9341_draw_rect(int x, int y, int w, int h, uint16_t color);

#endif
