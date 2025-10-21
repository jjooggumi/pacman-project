/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 팩맨과 고스트의 위치 및 상태 구조체
typedef struct {
	int x;
	int y;
	int dir; // 0: Right, 1: Up, 2: Left, 3: Down
	uint16_t color;
} Entity_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* LCD size */
#define LCD_WIDTH   240
#define LCD_HEIGHT  320

/* RGB565 */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_YELLOW    0xFFE0
#define COLOR_BLUE      0x001F
#define COLOR_RED       0xF800
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_ORANGE    0xFD20
#define COLOR_PINK 		0xFBEB

/* Button helper */
#define BTN_PRESSED(gpio, pin)   (HAL_GPIO_ReadPin((gpio),(pin)) == GPIO_PIN_RESET)

/* Buzzer timers are set to PSC=63 -> 1 MHz timer tick (64 MHz / 64) */
#define TIMER_TICK_HZ   1000000UL

/* Game Constants */
#define GRID_SIZE   16      // 움직임 단위
#define DOT_SIZE    4       // 도트 크기
#define MAP_WIDTH   (LCD_WIDTH / GRID_SIZE)
#define MAP_HEIGHT  (LCD_HEIGHT / GRID_SIZE)

#define ADDRESS   0x27 << 1
#define RS1_EN1   0x05
#define RS1_EN0   0x01
#define RS0_EN1   0x04
#define RS0_EN0   0x00
#define BackLight 0x08

#define delay_ms HAL_Delay

//#define ADDRESS   0x3F << 1
#define ADDRESS   0x27 << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t g_mirror_xy = 0; // 0: 일반, 1: 상하+좌우 동시 반전

// 팩맨과 고스트 위치 (그리드 기준 좌표)
Entity_t pacman = { .x = 1, .y = 1, .dir = 0, .color = COLOR_YELLOW };
Entity_t ghost = { .x = 13, .y = 10, .dir = 2, .color = COLOR_RED };

// ✨ 이전 위치 저장을 위한 변수 추가
static int pacman_old_x = 1, pacman_old_y = 1;
static int ghost_old_x = 13, ghost_old_y = 10;
static uint32_t ghost_last_move_time = 0; // ✨ 고스트 움직임 타이머

// 간단한 미로 맵 (1: 벽, 0: 통로, 2: 도트)
// 15x20 그리드 (240/16=15, 320/16=20)
uint8_t game_map[20][15] = { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, {
		1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 1, 1, 2, 1, 0,
		1, 2, 1, 1, 1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1 },
		{ 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1 }, { 1, 2, 1, 2, 2, 2, 2,
				2, 2, 2, 2, 2, 1, 2, 1 }, { 1, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
				1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 1 }, { 1,
				1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 2,
				1, 0, 1, 2, 0, 0, 0, 0, 0 }, // 고스트 집 입구(0)
		{ 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1 }, { 1, 2, 2, 2, 2, 2, 2,
				2, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1,
				1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 1 }, { 1,
				1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1 }, { 1, 2, 2, 2, 2, 2,
				2, 2, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1,
				2, 1, 2, 1 }, { 1, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1 }, {
				1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1 }, { 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } };

int delay = 0;
int value = 0;
int score = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* ILI9341 */
void ILI9341_Init(void);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/* UI */
static void StartScreen_Draw(void);
void Next_Step_Screen(void);

/* Input */
void wait_for_any_button_press(void);

/* Buzzer */
void BUZZ_Start(void);
void BUZZ_Stop(void);
void BUZZ_SetFreq_TIM1(uint32_t hz); /* PA11 TIM1_CH4 */
void BUZZ_SetFreq_TIM3(uint32_t hz); /* PB1  TIM3_CH4 */
void PLAY_PacmanIntro_Blocking(void); /* 2성부 */

/* GAME */
void GameScreen_DrawMap(void);
void GameScreen_DrawEntities(void);
void Update_Game_Logic(void);
void Update_Pacman_Logic(void);          // ✨ 새로 추가/수정된 함수
void Update_Ghost_Logic(void);           // ✨ 새로 추가/수정된 함수
void GameScreen_Update_Screen(void);     // ✨ 새로 추가된 화면 업데이트 함수
int get_next_pacman_dir(void);
uint32_t get_next_move_delay(void); // 팩맨의 움직임 딜레이

/* Score */
void delay_us(int us);
void LCD_DATA(uint8_t data);
void LCD_CMD(uint8_t cmd);
void LCD_CMD_4bit(uint8_t cmd);
void LCD_INIT(void);
void LCD_XY(char x, char y);
void LCD_CLEAR(void);
void LCD_PUTS(char *str);

/* I2C */
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Write_Byte(uint8_t data);
uint8_t I2C_Read_Byte(uint8_t ack);

// --- Bit-Banging 핀 제어 매크로 (PC6: SDA, PC8: SCL 가정) ---
// I2C1_SDA_Pin, I2C1_SCL_Pin이 각각 PC6, PC8이라고 가정합니다.
#define SDA_HIGH()      HAL_GPIO_WritePin(GPIOC, I2C1_SDA_Pin, GPIO_PIN_SET)
#define SDA_LOW()       HAL_GPIO_WritePin(GPIOC, I2C1_SDA_Pin, GPIO_PIN_RESET)
#define SCL_HIGH()      HAL_GPIO_WritePin(GPIOC, I2C1_SCL_Pin, GPIO_PIN_SET)
#define SCL_LOW()       HAL_GPIO_WritePin(GPIOC, I2C1_SCL_Pin, GPIO_PIN_RESET)
#define SDA_READ()      HAL_GPIO_ReadPin(GPIOC, I2C1_SDA_Pin)

// 실제 마이크로초 딜레이 함수가 없으므로 임시로 HAL_Delay(1)을 사용하거나
// 정확한 타이밍을 위해 별도 구현이 필요합니다.
#define I2C_DELAY()     delay_us(5) // 5us 딜레이 가정

#define SCORE_MAX_DIGITS 6 // 최대 6자리 점수까지 허용

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

/* ----------------------------- LCD low-level ----------------------------- */
static inline void LCD_CS_LOW(void) {
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}
static inline void LCD_CS_HIGH(void) {
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
static inline void LCD_RS_CMD(void) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
}
static inline void LCD_RS_DATA(void) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
}
static inline void LCD_WR_LOW(void) {
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
}
static inline void LCD_WR_HIGH(void) {
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
}
static inline void LCD_RD_HIGH(void) {
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_SET);
}
static inline void LCD_RST_LOW(void) {
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
}
static inline void LCD_RST_HIGH(void) {
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
}

/* 8-bit data bus */
static inline void LCD_SET_DATA(uint8_t d) {
	HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin,
			(d & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin,
			(d & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin,
			(d & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin,
			(d & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin,
			(d & (1 << 4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin,
			(d & (1 << 5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin,
			(d & (1 << 6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin,
			(d & (1 << 7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void LCD_WRITE_STROBE(void) {
	LCD_WR_LOW();
	__NOP();
	__NOP();
	__NOP();
	LCD_WR_HIGH();
}

static void ILI9341_WriteCmd(uint8_t cmd) {
	LCD_RS_CMD();
	LCD_SET_DATA(cmd);
	LCD_WRITE_STROBE();
}
static void ILI9341_WriteData8(uint8_t d) {
	LCD_RS_DATA();
	LCD_SET_DATA(d);
	LCD_WRITE_STROBE();
}
static void ILI9341_WriteData16(uint16_t d) {
	ILI9341_WriteData8(d >> 8);
	ILI9341_WriteData8(d & 0xFF);
}

static void ILI9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1,
		uint16_t y1) {
	ILI9341_WriteCmd(0x2A);
	ILI9341_WriteData16(x0);
	ILI9341_WriteData16(x1);
	ILI9341_WriteCmd(0x2B);
	ILI9341_WriteData16(y0);
	ILI9341_WriteData16(y1);
	ILI9341_WriteCmd(0x2C);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t c) {
	if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
		return;
	ILI9341_SetWindow(x, y, x, y);
	ILI9341_WriteData16(c);
}
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t c) {
	if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
		return;
	uint16_t x1 = (x + w - 1 < LCD_WIDTH) ? (x + w - 1) : (LCD_WIDTH - 1);
	uint16_t y1 = (y + h - 1 < LCD_HEIGHT) ? (y + h - 1) : (LCD_HEIGHT - 1);
	ILI9341_SetWindow(x, y, x1, y1);
	uint32_t n = (uint32_t) (x1 - x + 1) * (uint32_t) (y1 - y + 1);
	while (n--)
		ILI9341_WriteData16(c);
}
void ILI9341_FillScreen(uint16_t c) {
	ILI9341_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, c);
}

/* tiny 5x7 font subset */
static const uint8_t font5x7[][5] = { { 0, 0, 0, 0, 0 }, /* ' ' */
{ 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* P */
{ 0x7C, 0x12, 0x11, 0x12, 0x7C }, /* A */
{ 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* C */
{ 0x7F, 0x02, 0x0C, 0x02, 0x7F }, /* M */
{ 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* N */
};

static int fidx(char c) {
	switch (c) {
	case ' ':
		return 0;
	case 'N':
		return 1;
	case 'A':
		return 2;
	case 'M':
		return 3;
	case 'C':
		return 4;
	case 'P':
		return 5;
	default:
		return 0;
	}
}

/* ----- LCG 난수 (컨페티/각도 지터용) ----- */
static uint32_t fw_seed = 0x31415927u;
static inline uint32_t fw_rand(void) {
	fw_seed = fw_seed * 1664525u + 1013904223u;
	return fw_seed;
}

/* 안전한 사각 점 찍기(2x2 기본) */
static inline void PutDot2(int x, int y, uint16_t c) {
	if ((unsigned) x < LCD_WIDTH && (unsigned) y < LCD_HEIGHT) {
		int x0 = (x > 0) ? (x - 1) : x, y0 = (y > 0) ? (y - 1) : y;
		int w = (x0 + 2 <= LCD_WIDTH) ? 2 : (LCD_WIDTH - x0);
		int h = (y0 + 2 <= LCD_HEIGHT) ? 2 : (LCD_HEIGHT - y0);
		ILI9341_FillRect(x0, y0, w, h, c);
	}
}

/* 화면 안쪽으로 위치 보정(여유 margin) */
static void clamp_inside(int *x, int *y, int margin) {
	if (*x < margin)
		*x = margin;
	if (*x > (LCD_WIDTH - 1 - margin))
		*x = LCD_WIDTH - 1 - margin;
	if (*y < margin)
		*y = margin;
	if (*y > (LCD_HEIGHT - 1 - margin))
		*y = LCD_HEIGHT - 1 - margin;
}

/* ===== 좌우+상하 동시 미러 (글자용) ===== */
static void DrawChar5x7_FlipXY(int x, int y, char c, uint16_t fg, uint16_t bg,
		int s) {
	int i = fidx(c);
	for (int col = 0; col < 5; col++) {
		/* 좌우 반전을 위해 오른쪽 열부터 읽고, */
		uint8_t bits = font5x7[i][4 - col];
		for (int row = 0; row < 7; row++) {
			/* 상하 반전을 위해 아래쪽 행부터 찍는다 */
			uint16_t color = (bits & (1 << (row))) ? fg : bg; // row를 그대로 쓰면 위아래 뒤집힘
			ILI9341_FillRect(x + col * s, y + row * s, s, s, color);
		}
	}
	/* 글자 간 공백 */
	ILI9341_FillRect(x + 5 * s, y, s, 7 * s, bg);
}

static void DrawText_FlipXY(int x, int y, const char *s, uint16_t fg,
		uint16_t bg, int scale) {
	int cx = x;
	while (*s) {
		DrawChar5x7_FlipXY(cx, y, *s++, fg, bg, scale);
		cx += 6 * scale;
	}
}

/* circles + pacman */
static void DrawFilledCircle(int xc, int yc, int r, uint16_t color) {
	for (int y = -r; y <= r; y++) {
		int dx = (int) (sqrtf((float) r * r - (float) y * y) + 0.5f);
		ILI9341_FillRect(xc - dx, yc + y, 2 * dx + 1, 1, color);
	}
}

static void DrawLineH(int x, int y, int w, int t, uint16_t c) {
	ILI9341_FillRect(x, y - t / 2, w, t, c);
}
static void DrawLineV(int x, int y, int h, int t, uint16_t c) {
	ILI9341_FillRect(x - t / 2, y, t, h, c);
}

static void DrawRoundRect(int x, int y, int w, int h, int r, int t, uint16_t c) {
	/* 외곽선을 두께 t로 근사 */
	for (int i = 0; i < t; i++) {
		int ri = r - i;
		if (ri < 0)
			ri = 0;
		/* 네 귀퉁이 호 */
		for (int yy = -ri; yy <= ri; yy++) {
			int dx = (int) (sqrtf((float) ri * ri - (float) yy * yy) + 0.5f);
			/* 상단 좌/우 */
			ILI9341_FillRect(x + r - dx, y + i, 2 * dx, 1, c);
			ILI9341_FillRect(x + w - r - dx, y + i, 2 * dx, 1, c);
			/* 하단 좌/우 */
			ILI9341_FillRect(x + r - dx, y + h - 1 - i, 2 * dx, 1, c);
			ILI9341_FillRect(x + w - r - dx, y + h - 1 - i, 2 * dx, 1, c);
		}
		/* 직선 부분 */
		DrawLineH(x + r, y + i, w - 2 * r, 1, c);
		DrawLineH(x + r, y + h - 1 - i, w - 2 * r, 1, c);
		DrawLineV(x + i, y + r, h - 2 * r, 1, c);
		DrawLineV(x + w - 1 - i, y + r, h - 2 * r, 1, c);
	}
}

static void DrawPacman(int xc, int yc, int r, float mouth_deg, uint16_t color,
		uint16_t bg) {
	DrawFilledCircle(xc, yc, r, color);
	float a = mouth_deg * 3.1415926f / 180.f;
	int x1 = xc, y1 = yc;
	int x2 = xc + (int) (r * cosf(a / 2)), y2 = yc - (int) (r * sinf(a / 2));
	int x3 = xc + (int) (r * cosf(-a / 2)), y3 = yc - (int) (r * sinf(-a / 2));
	int miny = y1;
	if (y2 < miny)
		miny = y2;
	if (y3 < miny)
		miny = y3;
	int maxy = y1;
	if (y2 > maxy)
		maxy = y2;
	if (y3 > maxy)
		maxy = y3;
	for (int y = miny; y <= maxy; y++) {
		int xs[3], n = 0;
#define EDGE(xa,ya,xb,yb) if(!((ya<y&&yb<y)||(ya>y&&yb>y)||(ya==yb))){ float t=(ya==yb)?0.f:((float)(y-ya)/(float)(yb-ya)); xs[n++]=xa+(int)((xb-xa)*t); }
		EDGE(x1, y1, x2, y2);
		EDGE(x2, y2, x3, y3);
		EDGE(x3, y3, x1, y1);
#undef EDGE
		if (n >= 2) {
			if (xs[0] > xs[1]) {
				int t = xs[0];
				xs[0] = xs[1];
				xs[1] = t;
			}
			ILI9341_FillRect(xs[0], y, xs[1] - xs[0] + 1, 1, COLOR_BLACK);
		}
	}
	/* 눈 */
	ILI9341_FillRect(xc + r / 5, yc - r / 2, r / 6, r / 6, COLOR_BLACK);
}

static void DrawPacman_FlipX(int xc, int yc, int r, float mouth_deg,
		uint16_t color, uint16_t bg) {
	/* 몸통 채우기 */
	DrawFilledCircle(xc, yc, r, color);

	/* 입(왼쪽 방향으로 벌어지게): 중심각을 π(180°) 기준으로 ±a/2 */
	float a = mouth_deg * 3.1415926f / 180.f;
	int x1 = xc, y1 = yc;
	int x2 = xc - (int) (r * cosf(a / 2)), y2 = yc - (int) (r * sinf(a / 2));
	int x3 = xc - (int) (r * cosf(a / 2)), y3 = yc + (int) (r * sinf(a / 2));

	int miny = y1;
	if (y2 < miny)
		miny = y2;
	if (y3 < miny)
		miny = y3;
	int maxy = y1;
	if (y2 > maxy)
		maxy = y2;
	if (y3 > maxy)
		maxy = y3;

	for (int y = miny; y <= maxy; y++) {
		int xs[3], n = 0;
#define EDGE(xa,ya,xb,yb) \
            if (!((ya<y && yb<y) || (ya>y && yb>y) || (ya==yb))) { \
                float t = (ya==yb) ? 0.f : ((float)(y - ya) / (float)(yb - ya)); \
                xs[n++] = xa + (int)((xb - xa) * t); \
            }
		EDGE(x1, y1, x2, y2);
		EDGE(x2, y2, x3, y3);
		EDGE(x3, y3, x1, y1);
#undef EDGE

		if (n >= 2) {
			if (xs[0] > xs[1]) {
				int t = xs[0];
				xs[0] = xs[1];
				xs[1] = t;
			}
			ILI9341_FillRect(xs[0], y, xs[1] - xs[0] + 1, 1, COLOR_BLACK);
		}
	}

	/* 눈: 왼쪽에 위치하도록 이동 */
	ILI9341_FillRect(xc - r / 5 - r / 6, yc - r / 2, r / 6, r / 6, COLOR_BLACK);
}

static void DrawGhost(int x, int y, int w, int h, uint16_t body, uint16_t eye) {
	ILI9341_FillRect(x, y + h / 4, w, 3 * h / 4, body);
	int r = w / 2;
	for (int yy = 0; yy < h / 2; yy++) {
		int dx = (int) sqrtf((float) r * r - (float) (r - yy) * (r - yy));
		ILI9341_FillRect(x + r - dx, y + yy, 2 * dx, 1, body);
	}
	int tooth = w / 5;
	for (int i = 0; i < 5; i++)
		if (i % 2 == 0)
			ILI9341_FillRect(x + i * tooth, y + h - (h / 8), tooth, h / 8,
			COLOR_BLACK);
	int ex = x + w / 4, ey = y + h / 3;
	ILI9341_FillRect(ex, ey, w / 6, h / 6, COLOR_WHITE);
	ILI9341_FillRect(ex + w / 2, ey, w / 6, h / 6, COLOR_WHITE);
	ILI9341_FillRect(ex + w / 12, ey + h / 12, w / 12, w / 12, eye);
	ILI9341_FillRect(ex + w / 2 + w / 12, ey + h / 12, w / 12, w / 12, eye);
}

/* --------------------------- Start Screen ------------------------------ */
/* 원본 참고 이미지를 가상의 기준 캔버스(가로 360, 세로 210)로 설계한 뒤
 LCD(240x320)에 비율 유지로 축소하여 중앙 배치. 위/아래 레터박스는 자동으로 검정. */
static void StartScreen_Draw(void) {
	/* 반전 ON */
	g_mirror_xy = 1;

	ILI9341_FillScreen(COLOR_BLACK);

	const float W_REF = 360.0f;
	const float H_REF = 210.0f;

	float s = (float) LCD_WIDTH / W_REF;
	int scene_w = (int) (W_REF * s + 0.5f);
	int scene_h = (int) (H_REF * s + 0.5f);
	int x0 = (LCD_WIDTH - scene_w) / 2;
	int y0 = (LCD_HEIGHT - scene_h) / 2;

	int stroke = (int) (3 * s);
	if (stroke < 2)
		stroke = 2;

	/* 미로 프레임 */
	DrawRoundRect(x0 + (20 * s), y0 + (15 * s), (70 * s), (70 * s), (18 * s),
			stroke, COLOR_BLUE);
	DrawRoundRect(x0 + (270 * s), y0 + (135 * s), (70 * s), (70 * s), (18 * s),
			stroke, COLOR_BLUE);
	DrawRoundRect(x0 + (328 * s), y0 + (20 * s), (28 * s), (110 * s), (14 * s),
			stroke, COLOR_BLUE);
	DrawRoundRect(x0 + (20 * s), y0 + (120 * s), (28 * s), (110 * s), (14 * s),
			stroke, COLOR_BLUE);
	DrawRoundRect(x0 + (120 * s), y0 + (38 * s), (85 * s), (22 * s), (11 * s),
			stroke, COLOR_BLUE);
	DrawRoundRect(x0 + (215 * s), y0 + (38 * s), (85 * s), (22 * s), (11 * s),
			stroke, COLOR_BLUE);

	/* 과일 아이콘 */
	DrawFilledCircle(x0 + (210 * s), y0 + (30 * s), (5 * s), COLOR_RED);
	DrawFilledCircle(x0 + (165 * s), y0 + (30 * s), (5 * s), COLOR_RED);
	DrawFilledCircle(x0 + (58 * s), y0 + (172 * s), (6 * s), COLOR_RED);

	/* 중앙 로고 */
	int logo_scale = (int) (5 * s);
	if (logo_scale < 2)
		logo_scale = 2;
	int text_y = y0 + (80 * s);
	DrawText_FlipXY(x0 + (110 * s), text_y, "PAC", COLOR_YELLOW, COLOR_BLACK,
			logo_scale);
	DrawText_FlipXY(x0 + (110 * s) + 3 * (6 * logo_scale) + (8 * s), text_y,
			"MAN",
			COLOR_YELLOW, COLOR_BLACK, logo_scale);

	/* 하단 도트 & 팩맨 */
	int dot_r = (int) (3 * s);
	if (dot_r < 2)
		dot_r = 2;
	int big_r = (int) (7 * s);
	int base_y = y0 + (140 * s);
	int base_x = x0 + (100 * s);

	for (int i = 0; i < 7; i++)
		DrawFilledCircle(base_x + i * (12 * s), base_y, dot_r, COLOR_ORANGE);
	for (int i = 1; i < 8; i++)
		DrawFilledCircle(base_x, base_y + i * (12 * s), dot_r, COLOR_ORANGE);

	DrawFilledCircle(base_x + (8 * 12 * s), base_y, big_r, COLOR_ORANGE);
	DrawFilledCircle(base_x + (4 * 12 * s), base_y - (12 * s), (int) (5 * s),
	COLOR_ORANGE);

	for (int i = 1; i <= 10; i++)
		DrawFilledCircle(base_x + (8 * 12 * s) + i * (11 * s),
				base_y + (12 * s), dot_r, COLOR_ORANGE);

	/* 원본은 오른쪽 끝 팩맨(왼쪽 바라보는 형태로 그렸음) — 전체 반전이 적용되어 최종적으로 상하/좌우 뒤집혀 보임 */
	DrawPacman_FlipX(base_x + (8 * 12 * s) + (11 * 11 * s), base_y + (12 * s),
			(int) (10 * s), 45.f, COLOR_YELLOW, COLOR_BLACK);

	/* 유령들 */
	DrawGhost(x0 + (75 * s), y0 + (120 * s), (24 * s), (18 * s), COLOR_CYAN,
	COLOR_BLACK);
	DrawGhost(x0 + (180 * s), y0 + (185 * s), (24 * s), (18 * s), COLOR_RED,
	COLOR_BLACK);
	DrawGhost(x0 + (315 * s), y0 + (115 * s), (24 * s), (18 * s), COLOR_PINK,
	COLOR_BLACK);

	/* 반전 OFF (이후 화면에는 영향 없음) */
	g_mirror_xy = 0;
}

//StartScreen_Draw 함수 뒤에 넣기

/* --- DeadScreen 전용 LCG 난수 --- */
static uint32_t ds_seed = 0x13572468u;
static inline uint32_t ds_lcg(void) {
	ds_seed = ds_seed * 1664525u + 1013904223u;
	return ds_seed;
}

/* --- 작은 가로 대시(‘-’) 찍기: 중심(xc,yc), 길이 len, 두께 th --- */
static inline void PutDashH(int xc, int yc, int len, int th, uint16_t col) {
	if (len < 1)
		len = 1;
	if (th < 1)
		th = 1;
	int x0 = xc - len / 2;
	int y0 = yc - th / 2;
	if (x0 < 0) {
		len -= -x0;
		x0 = 0;
	}
	if (y0 < 0) {
		th -= -y0;
		y0 = 0;
	}
	if (x0 + len > LCD_WIDTH)
		len = LCD_WIDTH - x0;
	if (y0 + th > LCD_HEIGHT)
		th = LCD_HEIGHT - y0;
	if (len > 0 && th > 0)
		ILI9341_FillRect(x0, y0, len, th, col);
}

/* ===== 사망 폭발: '-' 대시가 링처럼 퍼지고, 중심부터 서서히 사라짐 ===== */
static void DeadScreen_Draw(void) {

	/* 폭발 중심: 팩맨 현재 픽셀 좌표 */
	const int cx = pacman.x * GRID_SIZE + GRID_SIZE / 2;
	const int cy = pacman.y * GRID_SIZE + GRID_SIZE / 2;
	/* 반경: 화면 전체의 약 90%까지 허용 */
	const int full = (LCD_WIDTH < LCD_HEIGHT ? LCD_WIDTH : LCD_HEIGHT) * 0.9f;

	int roomL = cx, roomR = LCD_WIDTH - 1 - cx;
	int roomT = cy, roomB = LCD_HEIGHT - 1 - cy;
	int edgeLimited = roomL;
	if (roomR < edgeLimited)
		edgeLimited = roomR;
	if (roomT < edgeLimited)
		edgeLimited = roomT;
	if (roomB < edgeLimited)
		edgeLimited = roomB;

	/* 두 배 정도 넓힌 폭발 반경 */
	const int maxr = (edgeLimited < full ? edgeLimited : (int) full);

	/* 파라미터 */
	const int step_r = 3;          // 반경 증가 속도
	const int frame_delay_ms = 10; // 프레임 간 딜레이

	/* 화면 어둡게 시작 */
	ILI9341_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);

	/* 1) 중심에서 바깥으로: '-' 대시 링을 뿌린다 */
	for (int r = 0; r <= maxr; r += step_r) {
		/* 색상 그라데이션: 노랑 → 오렌지 → 빨강 */
		uint16_t col = (r < (int) (maxr * 0.35f)) ? COLOR_YELLOW :
						(r < (int) (maxr * 0.70f)) ? COLOR_ORANGE : COLOR_RED;

		/* 링 위 대시 개수(둘레 비례, 하한 10) */
		int count = 8 + (int) ((2.f * 3.1415926f * r) / 12.f);
		if (count < 10)
			count = 10;

		for (int i = 0; i < count; ++i) {
			float base = (2.f * 3.1415926f * i) / (float) count;
			/* 각도/반경에 약간 지터(±1~2° 수준) */
			float jitter = ((int) (ds_lcg() % 61) - 30) * (3.1415926f / 1800.f);
			float a = base + jitter;

			int x = cx + (int) (r * cosf(a));
			int y = cy + (int) (r * sinf(a));

			/* 대시 길이/두께를 살짝 랜덤화 (길이 4~8, 두께 1~2) */
			int len = 4 + (int) (ds_lcg() % 5);       // 4..8
			int th = 1 + (int) (ds_lcg() & 1);       // 1..2

			/* 가로 대시로 찍기(‘-’ 느낌). 방향은 모두 수평로 통일 */
			if ((unsigned) x < LCD_WIDTH && (unsigned) y < LCD_HEIGHT)
				PutDashH(x, y, len, th, col);
		}

		/* 중심을 조금씩 비워 ‘산산이’ 느낌 */
		if ((r % (step_r * 2)) == 0) {
			int holes = count / 4;
			for (int k = 0; k < holes; ++k) {
				int rx = cx + ((int) ds_lcg() % (r ? r : 1)) - r / 2;
				int ry = cy + ((int) ds_lcg() % (r ? r : 1)) - r / 2;
				if ((unsigned) rx < LCD_WIDTH && (unsigned) ry < LCD_HEIGHT)
					ILI9341_DrawPixel(rx, ry, COLOR_BLACK);
			}
		}

		HAL_Delay(frame_delay_ms);
	}

	/* 2) 페이드아웃: 중심에서부터 검정 원을 키워가며 지운다 */
	for (int er = 2; er <= maxr; er += 3) {
		DrawFilledCircle(cx, cy, er, COLOR_BLACK);   // 가운데부터 서서히 사라짐
		HAL_Delay(12);
	}

	/* 3) 잔여 점수 정리(조금의 랜덤 블랙 스플랫) */
	for (int t = 0; t < 3; ++t) {
		int n = 60 + (ds_lcg() % 40);
		for (int j = 0; j < n; ++j) {
			int rx = (int) (ds_lcg() % LCD_WIDTH);
			int ry = (int) (ds_lcg() % LCD_HEIGHT);
			ILI9341_DrawPixel(rx, ry, COLOR_BLACK);
		}
		HAL_Delay(8);
	}
}

/* ----- 축포: 폭발 → 중심부터 페이드아웃 ----- */
static void Firework_BurstFade(int xc, int yc, int rmax) {
	if (rmax < 10)
		rmax = 10;

	/* 1) 폭발: 링을 늘려가며 점 뿌리기 */
	const int step_r = 3;
	for (int r = 3; r <= rmax; r += step_r) {
		/* 색상: 안쪽 노랑 → 중간 오렌지 → 바깥 흰색 */
		uint16_t col = (r < (int) (rmax * 0.33f)) ? COLOR_YELLOW :
						(r < (int) (rmax * 0.66f)) ? COLOR_ORANGE : COLOR_WHITE;

		/* 링 위 점 개수(둘레에 비례, 상한 50) */
		int count = 10 + (int) ((2.f * 3.1415926f * r) / 12.f);
		if (count > 50)
			count = 50;

		for (int i = 0; i < count; ++i) {
			float base = (2.f * 3.1415926f * i) / (float) count;
			float jitter = ((int) (fw_rand() % 61) - 30)
					* (3.1415926f / 1800.f); // ±~1.7°
			float a = base + jitter;

			int x = xc + (int) (r * cosf(a));
			int y = yc + (int) (r * sinf(a));
			PutDot2(x, y, col);
		}
		HAL_Delay(10);
	}

	/* 2) 페이드: 중심에서부터 검정 원을 키워가며 지우기 */
	for (int er = 2; er <= rmax; er += 3) {
		DrawFilledCircle(xc, yc, er, COLOR_BLACK); /* 중심부터 서서히 사라짐 */
		HAL_Delay(12);
	}

	/* 3) 가장자리 남은 점 약간 정리(블랙 스플랫) */
	for (int t = 0; t < 3; ++t) {
		int n = 60 + (fw_rand() % 40);
		for (int j = 0; j < n; ++j) {
			int rx = (int) (fw_rand() % LCD_WIDTH);
			int ry = (int) (fw_rand() % LCD_HEIGHT);
			ILI9341_DrawPixel(rx, ry, COLOR_BLACK);
		}
		HAL_Delay(8);
	}
}

/* ====== 문자 없이: 팩맨 + 주변 축포(2~3개) ====== */
static void ClearScreen_Draw(void) {
	/* 배경 초기화 */
	ILI9341_FillScreen(COLOR_BLACK);
	fw_seed ^= (HAL_GetTick() | 0xA5A55A5Au);

	/* 중앙 팩맨 유지(살짝 입 애니메 2프레임) */
	int pcx = LCD_WIDTH / 2;
	int pcy = LCD_HEIGHT / 2 + 8;
	int pr = (LCD_WIDTH < LCD_HEIGHT ? LCD_WIDTH : LCD_HEIGHT) / 6;
	DrawPacman(pcx, pcy, pr, 28.f, COLOR_YELLOW, COLOR_BLACK);
	HAL_Delay(120);
	DrawPacman(pcx, pcy, pr, 8.f, COLOR_YELLOW, COLOR_BLACK);

	/* 축포 위치 계산: 팩맨 주변 원 위에 2~3개 */
	int fireworks = 2 + (fw_rand() % 2);         // 2 또는 3개
	float base_ang = (float) (fw_rand() % 360) * 3.1415926f / 180.f;
	int ring_r = (int) (pr * 1.8f);             // 팩맨에서 약간 떨어진 반경
	int fr_max = (LCD_WIDTH < LCD_HEIGHT ? LCD_WIDTH : LCD_HEIGHT) / 2;
	int fw_rmax = (int) (fr_max * 0.45f);       // 축포 자체의 최대 반경(화면 절반 미만)

	for (int k = 0; k < fireworks; ++k) {
		float a = base_ang + k * (2.f * 3.1415926f / fireworks);
		/* 위치 */
		int fx = pcx + (int) (ring_r * cosf(a));
		int fy = pcy + (int) (ring_r * sinf(a));
		clamp_inside(&fx, &fy, fw_rmax + 6);   // 화면 밖으로 나가지 않게 마진 확보

		/* 축포 1개 연출 */
		Firework_BurstFade(fx, fy, fw_rmax);

		/* 팩맨이 가려졌다면 다시 한 번 그려 안정감 부여 */
		DrawPacman(pcx, pcy, pr, 12.f, COLOR_YELLOW, COLOR_BLACK);
	}
}

/* 50ms debounce for any of UP/LEFT/RIGHT/DOWN */
static int any_button_now(void) {
	return BTN_PRESSED(UP_GPIO_Port, UP_Pin)
			|| BTN_PRESSED(LEFT_GPIO_Port, LEFT_Pin)
			|| BTN_PRESSED(RIGHT_GPIO_Port, RIGHT_Pin)
			|| BTN_PRESSED(DOWN_GPIO_Port, DOWN_Pin);
}

void wait_for_any_button_press(void) {
	while (!any_button_now()) {
		HAL_Delay(1);
	}
	HAL_Delay(50);
	if (!any_button_now())
		return;
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	while (any_button_now()) {
		HAL_Delay(1);
	}
	HAL_Delay(20);
}

/* ----------------------------- ILI9341 init ------------------------------ */
void ILI9341_Init(void) {
	LCD_CS_HIGH();
	LCD_WR_HIGH();
	LCD_RD_HIGH();
	LCD_RST_LOW();
	HAL_Delay(20);
	LCD_RST_HIGH();
	HAL_Delay(120);

	LCD_CS_LOW();

	ILI9341_WriteCmd(0x01);
	HAL_Delay(5);
	ILI9341_WriteCmd(0x28);

	ILI9341_WriteCmd(0xCF);
	ILI9341_WriteData8(0x00);
	ILI9341_WriteData8(0x83);
	ILI9341_WriteData8(0x30);
	ILI9341_WriteCmd(0xED);
	ILI9341_WriteData8(0x64);
	ILI9341_WriteData8(0x03);
	ILI9341_WriteData8(0x12);
	ILI9341_WriteData8(0x81);
	ILI9341_WriteCmd(0xE8);
	ILI9341_WriteData8(0x85);
	ILI9341_WriteData8(0x01);
	ILI9341_WriteData8(0x79);
	ILI9341_WriteCmd(0xCB);
	ILI9341_WriteData8(0x39);
	ILI9341_WriteData8(0x2C);
	ILI9341_WriteData8(0x00);
	ILI9341_WriteData8(0x34);
	ILI9341_WriteData8(0x02);
	ILI9341_WriteCmd(0xF7);
	ILI9341_WriteData8(0x20);
	ILI9341_WriteCmd(0xEA);
	ILI9341_WriteData8(0x00);
	ILI9341_WriteData8(0x00);

	ILI9341_WriteCmd(0xC0);
	ILI9341_WriteData8(0x26);
	ILI9341_WriteCmd(0xC1);
	ILI9341_WriteData8(0x11);
	ILI9341_WriteCmd(0xC5);
	ILI9341_WriteData8(0x35);
	ILI9341_WriteData8(0x3E);
	ILI9341_WriteCmd(0xC7);
	ILI9341_WriteData8(0xBE);

	ILI9341_WriteCmd(0x36);
	ILI9341_WriteData8(0x48); /* MY=1, BGR=1 */
	ILI9341_WriteCmd(0x3A);
	ILI9341_WriteData8(0x55); /* RGB565 */

	ILI9341_WriteCmd(0xB1);
	ILI9341_WriteData8(0x00);
	ILI9341_WriteData8(0x1B);
	ILI9341_WriteCmd(0xB6);
	ILI9341_WriteData8(0x0A);
	ILI9341_WriteData8(0xA2);

	ILI9341_WriteCmd(0xF2);
	ILI9341_WriteData8(0x00);
	ILI9341_WriteCmd(0x26);
	ILI9341_WriteData8(0x01);

	ILI9341_WriteCmd(0x11);
	HAL_Delay(120);
	ILI9341_WriteCmd(0x29);
}

/* ----------------------------- Buzzer (PWM) ------------------------------ */
/* duty 50% square: ARR = TIMER_TICK_HZ/freq - 1 , CCR = (ARR+1)/2 */
static void set_pwm_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t hz) {
	if (hz == 0) {
		__HAL_TIM_SET_COMPARE(htim, channel, 0); /* mute */
		return;
	}
	uint32_t arr = (TIMER_TICK_HZ / hz);
	if (arr < 2)
		arr = 2;
	arr -= 1;
	__HAL_TIM_SET_AUTORELOAD(htim, arr);
	__HAL_TIM_SET_COMPARE(htim, channel, (arr + 1) / 2);
	__HAL_TIM_SET_COUNTER(htim, 0);
}

void BUZZ_Start(void) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* PA11 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); /* PB1  */
}
void BUZZ_Stop(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}
void BUZZ_SetFreq_TIM1(uint32_t hz) {
	set_pwm_freq(&htim1, TIM_CHANNEL_4, hz);
}
void BUZZ_SetFreq_TIM3(uint32_t hz) {
	set_pwm_freq(&htim3, TIM_CHANNEL_4, hz);
}

/* Pac-Man intro (간략 2성부 편곡) — 템포 140bpm 기준 근사 */
#define QN(ms)  (ms)          /* quarter  = 1beat */
#define EN(ms)  ((ms)/2)      /* eighth   = 1/2   */
#define SN(ms)  ((ms)/4)      /* sixteenth= 1/4   */

static const uint16_t TEMPO_MS = 430; /* quarter ≈430ms (@~140bpm) */

/* 음계 주파수(Hz) – 4/5/6옥타브 일부 */
enum {

	G2 = 98,
	/* 3옥타브(추가) */
	C3 = 131,
	Cs3 = 139,
	D3 = 147,
	Ds3 = 156,
	E3 = 165,
	F3 = 175,
	Fs3 = 185,
	G3 = 196,
	Gs3 = 208,
	A3 = 220,
	As3 = 233,
	B3 = 247,

	/* 4옥타브 */
	C4 = 262,
	Cs4 = 277,
	D4 = 294,
	Ds4 = 311,
	E4 = 330,
	F4 = 349,
	Fs4 = 370,
	G4 = 392,
	Gs4 = 415,
	A4 = 440,
	As4 = 466,
	B4 = 494,

	/* 5옥타브 */
	C5 = 523,
	Cs5 = 554,
	D5 = 587,
	Ds5 = 622,
	E5 = 659,
	F5 = 698,
	Fs5 = 740,
	G5 = 784,
	Gs5 = 831,
	A5 = 880,
	As5 = 932,
	B5 = 988,

	/* 6옥타브 */
	C6 = 1047,
	Cs6 = 1109,
	D6 = 1175,
	Ds6 = 1245,
	E6 = 1319,
	F6 = 1397,
	Fs6 = 1480,
	G6 = 1568,
	Gs6 = 1661,
	A6 = 1760,
	As6 = 1865,
	B6 = 1976
};

/* 멜로디/하모니 (클래식 팩맨 인트로의 느낌을 살린 근사치) */
//시작 배경음
static const uint16_t voice1_freq[] = { B5, B6, B5, G5, E5, C5, D5, G4, 0, E5,
		C5, D5, G4 };
static const uint16_t voice2_freq[] = { G4, D5, G4, E4, C4, G4, B4, D4, G3, C4,
		G4, B4, D4 };
static const uint16_t voice_dur[] = { EN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS),
		EN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS), QN(TEMPO_MS),
		EN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS), QN(TEMPO_MS) };
static const size_t VOICE_LEN = (sizeof(voice1_freq) / sizeof(voice1_freq[0]));

//팩맨 기본 브금
static const uint16_t voice3_freq[] = { B4, B5, Fs5, Ds5, B5, Fs5, A5, B4, B5,
		Fs5, Ds5, B5, Fs5, A5 };
static const uint16_t voice4_freq[] = { G4, G5, D5, B4, G5, D5, F5, G4, G5, D5,
		B4, G5, D5, F5 };
static const uint16_t voice_dur_1[] = { SN(TEMPO_MS), SN(TEMPO_MS), SN(
		TEMPO_MS), SN(TEMPO_MS), EN(TEMPO_MS), EN(TEMPO_MS), QN(TEMPO_MS), SN(
		TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), EN(TEMPO_MS), EN(
		TEMPO_MS), QN(TEMPO_MS) };
static const size_t VOICE_LEN_1 = (sizeof(voice3_freq) / sizeof(voice3_freq[0]));

//죽는 효과음
static const uint16_t voice5_freq[] = { D5, C5, As4, G4, B4, Fs4, Gs4, F4, Ds4,
		E4 };
static const uint16_t voice6_freq[] = { Cs5, B4, A4, Fs4, A4, E4, Fs4, Ds4, Cs4,
		F4 };
static const uint16_t voice_dur_death[] = { SN(TEMPO_MS), SN(TEMPO_MS), SN(
		TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), SN(
		TEMPO_MS), SN(TEMPO_MS), SN(TEMPO_MS), };
static const size_t VOICE_LEN_2 = (sizeof(voice5_freq) / sizeof(voice6_freq[0]));

//성공 효과음
static const uint16_t voice7_freq[] = { E5, G5, C6, E6, F6, E6, C6, D6, E6 };
static const uint16_t voice8_freq[] = { C5, E5, G5, C6, D6, C6, A5, B5, C6 };
static const uint16_t voice_dur_clear[] = { EN(TEMPO_MS), EN(TEMPO_MS), EN(
		TEMPO_MS), EN(TEMPO_MS), QN(TEMPO_MS), QN(TEMPO_MS), EN(TEMPO_MS), EN(
		TEMPO_MS), SN(TEMPO_MS) };
static const size_t VOICE_LEN_3 = (sizeof(voice7_freq) / sizeof(voice8_freq[0]));

//픽맨 도트 섭취
//static const uint16_t voice9_freq[] = {C5, E5, G5, B5, G5, E5, C5};
//static const uint16_t voice10_freq[] = {C5, E5, G5, B5, G5, E5, C5};
static const uint16_t voice9_freq[] = { E5, G5, C6, E6 };
static const uint16_t voice10_freq[] = { E5, G5, C6, E6 };
static const uint16_t voice_dur_eat[] = { 40, 40, 40, 60 };
//static const uint16_t voice_dur_eat[] = { 30, 30, 40, 50, 40, 30, 30 };
static const size_t VOICE_LEN_4 =
		(sizeof(voice9_freq) / sizeof(voice10_freq[0]));

void PLAY_Pacman_eating(void) {
	BUZZ_Start();
	for (size_t i = 0; i < VOICE_LEN_4; ++i) {
		BUZZ_SetFreq_TIM1(voice9_freq[i]); /* 멜로디 */
		BUZZ_SetFreq_TIM3(voice10_freq[i]); /* 하모니 */
		HAL_Delay(voice_dur_eat[i] - 10);
		/* 짧은 스택카토 */
		BUZZ_SetFreq_TIM1(0);
		BUZZ_SetFreq_TIM3(0);
		HAL_Delay(10);
	}
	BUZZ_Stop();
}

void PLAY_PacmanIntro_Blocking(void) {
	BUZZ_Start();
	for (size_t i = 0; i < VOICE_LEN; ++i) {
		BUZZ_SetFreq_TIM1(voice1_freq[i]); /* 멜로디 */
		BUZZ_SetFreq_TIM3(voice2_freq[i]); /* 하모니 */
		HAL_Delay(voice_dur[i] - 10);
		/* 짧은 스택카토 */
		BUZZ_SetFreq_TIM1(0);
		BUZZ_SetFreq_TIM3(0);
		HAL_Delay(10);
	}
	BUZZ_Stop();
}

void PLAY_Pacman_map(void) {
	BUZZ_Start();
	for (size_t i = 0; i < VOICE_LEN_1; ++i) {
		BUZZ_SetFreq_TIM1(voice3_freq[i]); /* 멜로디 */
		BUZZ_SetFreq_TIM3(voice4_freq[i]); /* 하모니 */
		HAL_Delay(voice_dur_1[i] - 10);
		/* 짧은 스택카토 */
		BUZZ_SetFreq_TIM1(0);
		BUZZ_SetFreq_TIM3(0);
		HAL_Delay(10);
	}
	BUZZ_Stop();
}

void PLAY_Pacman_dead(void) {
	BUZZ_Start();
	for (size_t i = 0; i < VOICE_LEN_2; ++i) {
		BUZZ_SetFreq_TIM1(voice5_freq[i]); /* 멜로디 */
		BUZZ_SetFreq_TIM3(voice6_freq[i]); /* 하모니 */
		HAL_Delay(voice_dur_death[i] - 10);
		/* 짧은 스택카토 */
		BUZZ_SetFreq_TIM1(0);
		BUZZ_SetFreq_TIM3(0);
		HAL_Delay(10);
	}
	BUZZ_Stop();
}

void PLAY_Pacman_clear(void) {
	BUZZ_Start();
	for (size_t i = 0; i < VOICE_LEN_3; ++i) {
		BUZZ_SetFreq_TIM1(voice7_freq[i]); /* 멜로디 */
		BUZZ_SetFreq_TIM3(voice8_freq[i]); /* 하모니 */
		HAL_Delay(voice_dur_clear[i] - 10);
		/* 짧은 스택카토 */
		BUZZ_SetFreq_TIM1(0);
		BUZZ_SetFreq_TIM3(0);
		HAL_Delay(10);
	}
	BUZZ_Stop();
}

/* ----------------------------- Game UI ----------------------------- */

// 미로와 도트 그리기
void GameScreen_DrawMap(void) {
	ILI9341_FillScreen(COLOR_BLACK);
	for (int y = 0; y < 20; y++) {
		for (int x = 0; x < 15; x++) {
			int px = x * GRID_SIZE;
			int py = y * GRID_SIZE;
			if (game_map[y][x] == 1) {
				// 벽 (파란색)
				ILI9341_FillRect(px, py, GRID_SIZE, GRID_SIZE, COLOR_BLUE);
			} else if (game_map[y][x] == 2) {
				// 도트 (흰색)
				ILI9341_FillRect(px + GRID_SIZE / 2 - DOT_SIZE / 2,
						py + GRID_SIZE / 2 - DOT_SIZE / 2,
						DOT_SIZE, DOT_SIZE, COLOR_WHITE);
			}
		}
	}
}

void Game_ResetMap(void) {
	uint8_t initial_map[20][15] = { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1 }, { 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 1,
			1, 2, 1, 0, 1, 2, 1, 1, 1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 1 }, { 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1 }, {
			1, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1 }, { 1, 2, 1, 1, 1, 1,
			1, 0, 1, 1, 1, 1, 1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2,
			2, 2, 1 }, { 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1 }, { 0, 0,
			0, 0, 0, 2, 1, 0, 1, 2, 0, 0, 0, 0, 0 }, // 고스트 집 입구(0)
			{ 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1 }, { 1, 2, 2, 2, 2, 2,
					2, 2, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 1, 1, 2, 1, 1, 1, 2,
					1, 1, 1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2,
					1 }, { 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1 }, { 1,
					2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1 }, { 1, 2, 1, 2, 1,
					1, 1, 1, 1, 1, 1, 2, 1, 2, 1 }, { 1, 2, 1, 2, 2, 2, 2, 2, 2,
					2, 2, 2, 1, 2, 1 }, { 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
					2, 1 }, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } };
	memcpy(game_map, initial_map, sizeof(game_map));

	// 2. 화면 다시 그리기
	GameScreen_DrawMap();
	score = 0;
}

// 팩맨과 고스트 그리기
void GameScreen_DrawEntities(void) {
	// 팩맨 그리기
	DrawPacman(pacman.x * GRID_SIZE + GRID_SIZE / 2,
			pacman.y * GRID_SIZE + GRID_SIZE / 2,
			GRID_SIZE / 2 - 2, 40.f, pacman.color, COLOR_BLACK);

	// 고스트 그리기 (단순화)
	DrawGhost(ghost.x * GRID_SIZE + 2, ghost.y * GRID_SIZE + 2,
	GRID_SIZE - 4, GRID_SIZE - 4, ghost.color, COLOR_BLUE);
}

/* ----------------------------- Game Logic ----------------------------- */

// 버튼 입력에 따라 팩맨의 다음 방향 결정
int get_next_pacman_dir(void) {
	int direction;

	// 각 버튼의 현재 상태를 변수로 저장
	GPIO_PinState up_state = HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin);
	GPIO_PinState down_state = HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin);
	GPIO_PinState left_state = HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin);
	GPIO_PinState right_state = HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin);

	// 디버깅용: watch 창에서 이 변수들을 보면 됨
	if (up_state == GPIO_PIN_RESET)
		direction = 1;
	else if (left_state == GPIO_PIN_RESET)
		direction = 2;
	else if (right_state == GPIO_PIN_RESET)
		direction = 0;
	else if (down_state == GPIO_PIN_RESET)
		direction = 3;
	else
		direction = -1;

	return direction;
}

// 팩맨 이동 및 도트 처리 로직
void Update_Pacman_Logic(void) {
	// 1. ✨ 화면 업데이트를 위해 현재 위치를 이전 위치로 저장 (이동 시도 전에 저장해야 함)
	pacman_old_x = pacman.x;
	pacman_old_y = pacman.y;

	int input_dir = get_next_pacman_dir();

	// 입력이 없으면 움직이지 않고, 화면 업데이트가 되지 않도록 여기서 바로 종료
	if (input_dir == -1) {
		// pacman.x == pacman_old_x 상태로 유지되어 GameScreen_Update_Screen에서 지우지 않음
		return;
	}

	int next_x = pacman.x;
	int next_y = pacman.y;

	// 다음 위치 계산 (0:Right, 1:Up, 2:Left, 3:Down)
	if (input_dir == 0)
		next_x++;
	else if (input_dir == 1)
		next_y--;
	else if (input_dir == 2)
		next_x--;
	else if (input_dir == 3)
		next_y++;

	// 충돌 검사 (벽=1)
	if (next_x >= 0 && next_x < 15 && next_y >= 0 && next_y < 20
			&& game_map[next_y][next_x] != 1) {
		// 움직임 허용
		pacman.x = next_x;
		pacman.y = next_y;

		// 🚨 이동 성공 시에만 방향 업데이트
		pacman.dir = input_dir;
		char str_buffer[17];

		// 도트(2) 먹기
		if (game_map[pacman.y][pacman.x] == 2) {
			game_map[pacman.y][pacman.x] = 0; // 도트 제거

			// 💰 아이템 획득 사운드 (간단한 고음 재생)
			PLAY_Pacman_eating();
			score = score + 5;

			 // 문자열을 저장할 버퍼 (16글자 + null 종료 문자)

			// 1. 커서 이동 (0행 1열, 즉 두 번째 줄 시작)
			LCD_XY(0, 1);

			// 2. 숫자 값을 문자열로 변환
			// 'score' 변수의 값을 10진수 문자열로 변환하여 str_buffer에 저장합니다.
			sprintf(str_buffer, "SCORE: %*d   ", SCORE_MAX_DIGITS, score);

			// 3. 문자열 출력
			LCD_PUTS(str_buffer);

		}
	} else {
		// 🚨 벽에 막혀 움직이지 못한 경우:
		// pacman.x가 pacman_old_x와 같아지도록 원래대로 놔둠.
		// 이는 GameScreen_Update_Screen에서 팩맨을 지우지 않도록 보장함.
	}
}

// 고스트 이동 (단순 무작위 이동)
void Update_Ghost_Logic(void) {
	// ✨ 1. 현재 위치를 이전 위치로 저장 (화면 지우기를 위해)
	ghost_old_x = ghost.x;
	ghost_old_y = ghost.y;

	if (HAL_GetTick() - ghost_last_move_time < 200)
		return; // 200ms마다 이동
	ghost_last_move_time = HAL_GetTick();

	int dx[] = { 1, 0, -1, 0 }; // R, U, L, D
	int dy[] = { 0, -1, 0, 1 };

	int next_x = ghost.x + dx[ghost.dir];
	int next_y = ghost.y + dy[ghost.dir];

	// 벽에 부딪히거나 통로가 막혔을 경우 무작위 방향으로 변경
	if (next_x < 0 || next_x >= 15 || next_y < 0 || next_y >= 20
			|| game_map[next_y][next_x] == 1) {
		ghost.dir = HAL_GetTick() % 4; // 방향 전환
	} else {
		ghost.x = next_x;
		ghost.y = next_y;
	}
}

void Update_Game_Logic(void) {
	// 1. 팩맨 이동
	Update_Pacman_Logic();

	// 2. 고스트 이동
	Update_Ghost_Logic();

	// 3. 충돌 검사 (팩맨과 고스트가 같은 칸에 있는지)
	if (pacman.x == ghost.x && pacman.y == ghost.y) {

		//사망 화면

		// Game Over! (사망 효과음)
		PLAY_Pacman_dead();
		DeadScreen_Draw();

		// 초기 리셋
		Game_ResetMap();
		pacman.x = 1;
		pacman.y = 1;
		ghost.x = 13;
		ghost.y = 10;
	} else { // 4. ✨ 성공 로직은 충돌이 없을 때만 검사

		// 4. ✨ 도트(펠릿) 모두 먹었는지 검사 (게임 클리어)
		int pellets_remaining = 0;
		// 맵 크기는 [20][15]
		for (int y = 0; y < 20; y++) {
			for (int x = 0; x < 15; x++) {
				if (game_map[y][x] == 2) {
					pellets_remaining++;
					break; // 하나라도 찾으면 루프 종료 후 카운트 증가
				}
			}
			if (pellets_remaining > 0)
				break; // 불필요한 전체 순회 방지
		}

		if (pellets_remaining == 0) {
			// 게임 클리어!

			// ✨ 성공 화면/효과음
			// (예: Stage Clear! 메시지 표시 및 승리 사운드)
			// 성공 효과음 (높은 음계 반복)
			PLAY_Pacman_clear();
			ClearScreen_Draw();

			// 다음 단계로 넘어가거나, 맵 리셋 후 재시작
			Game_ResetMap();
			pacman.x = 1;
			pacman.y = 1;
			ghost.x = 13;
			ghost.y = 10;
		}
	}
}

// **화면 업데이트 함수 (지우고 그리기)**

void GameScreen_Update_Screen(void) {

// 이전 위치를 배경색(BLACK)으로 지웁니다.
// 팩맨 이전 위치 지우기

	ILI9341_FillRect(pacman_old_x * GRID_SIZE, pacman_old_y * GRID_SIZE,
	GRID_SIZE, GRID_SIZE, COLOR_BLACK);

	// 고스트 이전 위치 지우기
	ILI9341_FillRect(ghost_old_x * GRID_SIZE, ghost_old_y * GRID_SIZE,
	GRID_SIZE, GRID_SIZE, COLOR_BLACK);

	// 맵의 도트가 지워졌을 수도 있으므로 이전 위치의 도트를 다시 그립니다.
	// ✨ 지워진 칸에 도트가 있다면 다시 그립니다.

	if (game_map[pacman_old_y][pacman_old_x] == 2) {
		ILI9341_FillRect(
				pacman_old_x * GRID_SIZE + GRID_SIZE / 2 - DOT_SIZE / 2,
				pacman_old_y * GRID_SIZE + GRID_SIZE / 2 - DOT_SIZE / 2,
				DOT_SIZE, DOT_SIZE, COLOR_WHITE);
	}

	if (game_map[ghost_old_y][ghost_old_x] == 2) {

		ILI9341_FillRect(ghost_old_x * GRID_SIZE + GRID_SIZE / 2 - DOT_SIZE / 2,
				ghost_old_y * GRID_SIZE + GRID_SIZE / 2 - DOT_SIZE / 2,
				DOT_SIZE, DOT_SIZE, COLOR_WHITE);

	}

// 현재 위치를 다시 그립니다.

	GameScreen_DrawEntities();

}

//void I2C_ScanAddresses(void) {
//	HAL_StatusTypeDef result;
//	uint8_t i;
//
//	printf("Scanning I2C addresses...\r\n");
//
//	for (i = 1; i < 128; i++) {
//		/*
//		 * HAL_I2C_IsDeviceReady: If a device at the specified address exists return HAL_OK.
//		 * Since I2C devices must have an 8-bit address, the 7-bit address is shifted left by 1 bit.
//		 */
//		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 1, 10);
//		if (result == HAL_OK) {
//			printf("I2C device found at address 0x%02X\r\n", i);
//		}
//	}
//
//	printf("Scan complete.\r\n");
//}

void delay_us(int us) {
	value = 3;
	delay = us * value;
	for (int i = 0; i < delay; i++)
		;
}
// --- Bit-Banging I2C 핵심 함수 구현 ---

/**
 * @brief  I2C Start Condition을 생성합니다.
 * SCL = High인 상태에서 SDA를 High에서 Low로 전환.
 * @retval None
 */
void I2C_Start(void) {
	SDA_HIGH();
	SCL_HIGH();
	I2C_DELAY();
	SDA_LOW(); // Start Condition: SCL이 High일 때 SDA High -> Low
	I2C_DELAY();
	SCL_LOW();
	I2C_DELAY();
}

/**
 * @brief  I2C Stop Condition을 생성합니다.
 * SDA = Low인 상태에서 SCL을 High로 올린 후 SDA를 Low에서 High로 전환.
 * @retval None
 */
void I2C_Stop(void) {
	SDA_LOW();
	I2C_DELAY();
	SCL_HIGH();
	I2C_DELAY();
	SDA_HIGH(); // Stop Condition: SCL이 High일 때 SDA Low -> High
	I2C_DELAY();
}

/**
 * @brief  I2C 버스에 1바이트 데이터를 전송합니다.
 * @param  data: 전송할 8비트 데이터.
 * @retval 0: ACK 수신, 1: NACK 수신.
 */
uint8_t I2C_Write_Byte(uint8_t data) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		// 1. 데이터 비트 출력 (MSB부터)
		if (data & 0x80) {
			SDA_HIGH();
		} else {
			SDA_LOW();
		}
		I2C_DELAY();

		// 2. 클럭 펄스 생성
		SCL_HIGH();
		I2C_DELAY();
		SCL_LOW();
		I2C_DELAY();

		data <<= 1;
	}

	// 3. ACK 비트 수신
	SDA_HIGH();// 마스터는 SDA를 High-Z (Pull-up) 상태로 둡니다.
	I2C_DELAY();
	SCL_HIGH();
	I2C_DELAY();
	uint8_t ack = SDA_READ(); // SDA 라인의 상태를 읽어 ACK(0) 또는 NACK(1) 확인
	SCL_LOW();
	I2C_DELAY();

	return ack; // 0이면 ACK, 1이면 NACK
}

/**
 * @brief  I2C 버스로부터 1바이트 데이터를 수신합니다.
 * @param  ack: 0이면 ACK를 전송 (다음 바이트를 계속 받겠다는 의미), 1이면 NACK를 전송 (마지막 바이트라는 의미).
 * @retval 수신한 8비트 데이터.
 */
uint8_t I2C_Read_Byte(uint8_t ack) {
	uint8_t i, data = 0;

	SDA_HIGH(); // 마스터는 SDA를 High-Z (Pull-up) 상태로 둡니다.
	I2C_DELAY();

	for (i = 0; i < 8; i++) {
		data <<= 1;

		// 1. 클럭 펄스 생성
		SCL_HIGH();
		I2C_DELAY();
		if (SDA_READ()) {
			data |= 0x01;
		}
		SCL_LOW();
		I2C_DELAY();
	}

	// 2. ACK 비트 전송
	if (ack == 0) // ACK 전송
			{
		SDA_LOW();
	} else // NACK 전송
	{
		SDA_HIGH();
	}
	I2C_DELAY();
	SCL_HIGH();
	I2C_DELAY();
	SCL_LOW();
	I2C_DELAY();

	return data;
}

// --- LCD 통신 함수에 I2C_Write_Byte 적용 ---

// 데이터를 I2C 버스를 통해 LCD로 전송하는 하위 레벨 함수
void I2C_Write(uint8_t byte) {
	I2C_Start();
	I2C_Write_Byte(ADDRESS); // LCD 어드레스 전송
	I2C_Write_Byte(byte);    // 데이터/커맨드 전송
	I2C_Stop();
}

/* I2C Bit-Banging 기반 LCD 통신 함수 (이전에 구현된 I2C_Write 함수 사용) */
// I2C_Write(uint8_t byte) 함수는 Start, 주소 전송, 데이터 전송, Stop을 모두 포함합니다.
/**
 * @brief  LCD에 커맨드를 전송합니다. (4비트 모드)
 * @param  cmd: 전송할 8비트 커맨드.
 * @retval None
 */
void LCD_CMD(uint8_t cmd) {
	uint8_t high_nibble = (cmd & 0xF0) | BackLight;
	uint8_t low_nibble = ((cmd << 4) & 0xF0) | BackLight;

	// 1. High Nibble 전송 (RS=0, EN=1)
	I2C_Write(high_nibble | RS0_EN1);
	delay_us(4); // E 펄스 유지 시간

	// 2. E=0으로 설정 (E 펄스 종료)
	I2C_Write(high_nibble | RS0_EN0);
	delay_us(50); // 커맨드 처리 시간 확보

	// 3. Low Nibble 전송 (RS=0, EN=1)
	I2C_Write(low_nibble | RS0_EN1);
	delay_us(4); // E 펄스 유지 시간

	// 4. E=0으로 설정 (E 펄스 종료)
	I2C_Write(low_nibble | RS0_EN0);
	delay_us(50); // 커맨드 처리 시간 확보 (0x01, 0x02 커맨드는 더 긴 딜레이 필요)
}

/**
 * @brief  LCD에 데이터(문자)를 전송합니다. (4비트 모드)
 * @param  data: 전송할 8비트 데이터.
 * @retval None
 */
void LCD_DATA(uint8_t data) {
	uint8_t high_nibble = (data & 0xF0) | BackLight;
	uint8_t low_nibble = ((data << 4) & 0xF0) | BackLight;

	// 1. High Nibble 전송 (RS=1, EN=1)
	I2C_Write(high_nibble | RS1_EN1);
	delay_us(4);

	// 2. E=0으로 설정
	I2C_Write(high_nibble | RS1_EN0);
	delay_us(50);

	// 3. Low Nibble 전송 (RS=1, EN=1)
	I2C_Write(low_nibble | RS1_EN1);
	delay_us(4);

	// 4. E=0으로 설정
	I2C_Write(low_nibble | RS1_EN0);
	delay_us(50);
}

// 4비트 초기화 시 사용되는 특수 함수 (RS 비트 무시하고 4비트만 전송)
void LCD_CMD_4bit(uint8_t cmd) {
	uint8_t high_nibble = (cmd & 0xF0) | BackLight;

	// High Nibble 전송 (RS=0, EN=1)
	I2C_Write(high_nibble | RS0_EN1);
	delay_us(4);

	// E=0으로 설정
	I2C_Write(high_nibble | RS0_EN0);
	delay_us(50);
}

void LCD_INIT(void) {

	delay_ms(100);

	// 1. 4비트 초기화 시퀀스 (0x03 3번, 0x02 1번)
	LCD_CMD_4bit(0x03);
	delay_ms(5);
	LCD_CMD_4bit(0x03);
	delay_us(100);
	LCD_CMD_4bit(0x03);
	delay_us(100);
	LCD_CMD_4bit(0x02);
	delay_us(100);

	// 2. 기능 설정 (Function Set: 4비트, 2줄, 5x8 폰트)
	LCD_CMD(0x28);

	// 3. 디스플레이 끄기 (Display Off)
	LCD_CMD(0x08);

	// 4. 화면 클리어 (Clear Display)
	LCD_CMD(0x01);
	delay_ms(3); // Clear Display는 긴 딜레이 필요

	// 5. 엔트리 모드 설정 (Entry Mode Set)
	LCD_CMD(0x06);

	// 6. 디스플레이 켜기 (Display On)
	LCD_CMD(0x0C);
}

void LCD_XY(char x, char y) {
	if (y == 0)
		LCD_CMD(0x80 + x);
	else if (y == 1)
		LCD_CMD(0xC0 + x);
	else if (y == 2)
		LCD_CMD(0x94 + x);
	else if (y == 3)
		LCD_CMD(0xD4 + x);
}

void LCD_CLEAR(void) {
	LCD_CMD(0x01);
	delay_ms(3); // Clear Display 커맨드는 약 1.52ms가 필요하므로 3ms로 여유있게 설정
}

void LCD_PUTS(char *str) {
	while (*str)
		LCD_DATA(*str++);
}

/**
 * @brief  I2C 버스에서 응답하는 주소를 스캔합니다. (Bit-Banging 버전)
 * @note   printf 함수가 UART를 통해 정상적으로 작동하고 있어야 합니다.
 * @retval None
 */
void I2C_ScanAddresses(void) {
	uint8_t i;
	uint8_t ack_result;

	printf("Scanning I2C addresses (Bit-Banging)...\r\n");

	// 7비트 주소 1부터 127까지 스캔
	for (i = 1; i < 128; i++) {
		uint8_t slave_addr_write = (uint8_t) (i << 1); // 7비트 주소를 8비트 주소 (Write)로 변환

		// 1. START 조건 전송
		I2C_Start();

		// 2. 주소 + Write 비트 전송
		// I2C_Write_Byte는 ACK(0) 또는 NACK(1)를 반환합니다.
		ack_result = I2C_Write_Byte(slave_addr_write);

		// 3. STOP 조건 전송 (ACK 수신 여부와 관계없이 버스 해제)
		I2C_Stop();

		// ack_result가 0이면 ACK를 받았다는 의미이며, 해당 주소에 장치가 있다는 뜻입니다.
		if (ack_result == 0) {
			printf("I2C device found at address 0x%02X\r\n", i);
		}
	}

	printf("Scan complete.\r\n");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
//
	LCD_INIT();
//	I2C_ScanAddresses();
	LCD_XY(0, 0); LCD_PUTS((char*) "--PACMAN GAME--");
	ILI9341_Init();

	StartScreen_Draw();
	/* 재생: 시작 화면이 뜨자마자 2성부 인트로 */
	PLAY_PacmanIntro_Blocking();
	wait_for_any_button_press();

	//맵 체크
	GameScreen_DrawMap();
	GameScreen_DrawEntities();
	PLAY_Pacman_map();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* 아무 방향 버튼을 누르면 다음 화면 */
		Update_Game_Logic();
		GameScreen_Update_Screen();

		HAL_Delay(100);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 64 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 64 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LCD_RST_Pin | I2C1_SDA_Pin | LCD_D1_Pin | I2C1_SCL_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LCD_RD_Pin | LCD_WR_Pin | LCD_RS_Pin | LD2_Pin | LCD_D7_Pin
					| LCD_D0_Pin | LCD_D2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LCD_CS_Pin | LCD_D6_Pin | LCD_D3_Pin | LCD_D5_Pin | LCD_D4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
	GPIO_InitStruct.Pin = LCD_RST_Pin | LCD_D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : UP_Pin LEFT_Pin */
	GPIO_InitStruct.Pin = UP_Pin | LEFT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LD2_Pin
	 LCD_D7_Pin LCD_D0_Pin LCD_D2_Pin */
	GPIO_InitStruct.Pin = LCD_RD_Pin | LCD_WR_Pin | LCD_RS_Pin | LD2_Pin
			| LCD_D7_Pin | LCD_D0_Pin | LCD_D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
	 LCD_D4_Pin */
	GPIO_InitStruct.Pin = LCD_CS_Pin | LCD_D6_Pin | LCD_D3_Pin | LCD_D5_Pin
			| LCD_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DOWN_Pin RIGHT_Pin */
	GPIO_InitStruct.Pin = DOWN_Pin | RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C1_SDA_Pin I2C1_SCL_Pin */
	GPIO_InitStruct.Pin = I2C1_SDA_Pin | I2C1_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
