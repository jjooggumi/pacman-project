/*
 * Ili9341.h
 *
 *  Created on: Sep 17, 2025
 *      Author: Administrator
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Control pin definitions
#define LCD_RD_PORT     GPIOA
#define LCD_RD_PIN      GPIO_PIN_0
#define LCD_WR_PORT     GPIOA
#define LCD_WR_PIN      GPIO_PIN_1
#define LCD_RS_PORT     GPIOA
#define LCD_RS_PIN      GPIO_PIN_4
#define LCD_CS_PORT     GPIOB
#define LCD_CS_PIN      GPIO_PIN_0
#define LCD_RST_PORT    GPIOC
#define LCD_RST_PIN     GPIO_PIN_1
#define F_CS_PORT       GPIOC
#define F_CS_PIN        GPIO_PIN_0

// Data pin definitions (8-bit bus)
#define LCD_D0_PORT     GPIOA
#define LCD_D0_PIN      GPIO_PIN_9
#define LCD_D1_PORT     GPIOC
#define LCD_D1_PIN      GPIO_PIN_7
#define LCD_D2_PORT     GPIOA
#define LCD_D2_PIN      GPIO_PIN_10
#define LCD_D3_PORT     GPIOB
#define LCD_D3_PIN      GPIO_PIN_3
#define LCD_D4_PORT     GPIOB
#define LCD_D4_PIN      GPIO_PIN_5
#define LCD_D5_PORT     GPIOB
#define LCD_D5_PIN      GPIO_PIN_4
#define LCD_D6_PORT     GPIOB
#define LCD_D6_PIN      GPIO_PIN_10
#define LCD_D7_PORT     GPIOA
#define LCD_D7_PIN      GPIO_PIN_8

// Screen dimensions
#define ILI9341_WIDTH       240
#define ILI9341_HEIGHT      320

// Colors (RGB565 format)
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// ILI9341 commands
#define ILI9341_SWRESET     0x01
#define ILI9341_RDDID       0x04
#define ILI9341_RDDST       0x09
#define ILI9341_SLPIN       0x10
#define ILI9341_SLPOUT      0x11
#define ILI9341_PTLON       0x12
#define ILI9341_NORON       0x13
#define ILI9341_RDMODE      0x0A
#define ILI9341_RDMADCTL    0x0B
#define ILI9341_RDPIXFMT    0x0C
#define ILI9341_RDIMGFMT    0x0D
#define ILI9341_RDSELFDIAG  0x0F
#define ILI9341_INVOFF      0x20
#define ILI9341_INVON       0x21
#define ILI9341_GAMMASET    0x26
#define ILI9341_DISPOFF     0x28
#define ILI9341_DISPON      0x29
#define ILI9341_CASET       0x2A
#define ILI9341_PASET       0x2B
#define ILI9341_RAMWR       0x2C
#define ILI9341_RAMRD       0x2E
#define ILI9341_PTLAR       0x30
#define ILI9341_MADCTL      0x36
#define ILI9341_VSCRSADD    0x37
#define ILI9341_PIXFMT      0x3A
#define ILI9341_FRMCTR1     0xB1
#define ILI9341_FRMCTR2     0xB2
#define ILI9341_FRMCTR3     0xB3
#define ILI9341_INVCTR      0xB4
#define ILI9341_DFUNCTR     0xB6
#define ILI9341_PWCTR1      0xC0
#define ILI9341_PWCTR2      0xC1
#define ILI9341_PWCTR3      0xC2
#define ILI9341_PWCTR4      0xC3
#define ILI9341_PWCTR5      0xC4
#define ILI9341_VMCTR1      0xC5
#define ILI9341_VMCTR2      0xC7
#define ILI9341_RDID1       0xDA
#define ILI9341_RDID2       0xDB
#define ILI9341_RDID3       0xDC
#define ILI9341_RDID4       0xDD
#define ILI9341_GMCTRP1     0xE0
#define ILI9341_GMCTRN1     0xE1

// Function prototypes
void ILI9341_Init(void);
void ILI9341_WriteCommand(uint8_t cmd);
void ILI9341_WriteData(uint8_t data);
void ILI9341_WriteData16(uint16_t data);
uint8_t ILI9341_ReadData(void);
void ILI9341_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_Fill(uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9341_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ILI9341_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bgcolor);
void ILI9341_DrawString(uint16_t x, uint16_t y, char* str, uint16_t color, uint16_t bgcolor);

//#endif

#endif /* INC_ILI9341_H_ */
