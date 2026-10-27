/**
  ******************************************************************************
  * File Name          : SDIO.c
  * Description        : This file provides code for the configuration
  *                      of the SDIO instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "sdio.h"

/* 句柄由本文件唯一持有；底层使用目标工程原有的 STM32F4 HAL。 */
SD_HandleTypeDef hsd;

/**
 * @brief 准备 SDIO 参数，不访问卡、不挂载文件系统。
 * @note 必须先在主程序完成系统时钟配置；实际卡初始化交给 BSP_SD_Init()。
 *       当前 PLLQ 输出为 45 MHz，HAL 的初始化分频 118 对应 375 kHz，
 *       本处传输分频 0 对应 22.5 MHz。先以 1 位完成卡识别，再由 BSP 切换 4 位。
 *       本函数只在启动或重新初始化前调用，不能在传输过程中修改句柄参数。
 */
void MX_SDIO_SD_Init(void)
{
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0U;
}

/**
 * @brief HAL 初始化时配置 A 板 TF 卡对应的时钟和引脚。
 * @note 当前阶段只实现轮询读写，不启用 SDIO 中断、不配置 DMA，
 *       不占用现有 USART2 或未来 UART8 的 DMA 资源。
 */
void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if ((sdHandle == NULL) || (sdHandle->Instance != SDIO))
    {
        return;
    }

    /* 显式开启端口时钟，避免依赖其他外设初始化的先后顺序。 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SDIO_CLK_ENABLE();

    /* PC8~PC11 = D0~D3，PC12 = CLK；保留官方 AF12 和无内部上下拉配置。 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PD2 = CMD；同一 GPIO 配置其余字段沿用上面的 AF12 设置。 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
 * @brief 释放 SDIO 引脚和外设时钟，不影响同端口的 CAN、串口或调零开关。
 * @note 卡检测 PE15 由 gpio.c 管理，不能在此释放；本阶段没有 DMA/IRQ 可释放。
 */
void HAL_SD_MspDeInit(SD_HandleTypeDef *sdHandle)
{
    if ((sdHandle == NULL) || (sdHandle->Instance != SDIO))
    {
        return;
    }

    __HAL_RCC_SDIO_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                           GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
