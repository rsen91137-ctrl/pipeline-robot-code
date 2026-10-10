#ifndef OLED_H
#define OLED_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7位地址；如果扫描到0x3D，在这里修改 */
#define OLED_ADDRESS       0x3C
#define OLED_WIDTH         128
#define OLED_HEIGHT        64

HAL_StatusTypeDef OLED_Check(void);
HAL_StatusTypeDef OLED_Init(void);

/* 下面三个函数修改内存中的图像，需要调用OLED_Update显示 */
void OLED_Clear(void);
void OLED_Fill(uint8_t on);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
void OLED_DrawRectangle(uint8_t x, uint8_t y,
                        uint8_t width, uint8_t height,
                        uint8_t on);

void OLED_ShowHelloWorld(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *text);
void OLED_ShowLine(uint8_t line, const char *text);

/* 将内存中的图像发送到屏幕 */
HAL_StatusTypeDef OLED_Update(void);

#ifdef __cplusplus
}
#endif

#endif
