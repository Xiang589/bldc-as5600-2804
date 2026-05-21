#ifndef LCD_ILI9341_H
#define LCD_ILI9341_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_WIDTH  240U
#define LCD_HEIGHT 320U

void LCD_Init(void);
void LCD_SetBacklight(uint8_t on);
void LCD_FillScreen(uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawText(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg_color);
uint8_t LCD_IsBusy(void);
uint8_t LCD_WaitReady(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
