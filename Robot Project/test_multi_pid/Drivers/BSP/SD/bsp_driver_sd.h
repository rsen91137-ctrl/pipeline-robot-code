/**
 ******************************************************************************
  * @file    bsp_driver_sd.h for F4 (based on stm324x9i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for 
  *          the bsp_driver_sd.c driver.
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

#ifndef BSP_DRIVER_SD_H
#define BSP_DRIVER_SD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 保留官方磁盘适配层使用的类型别名、返回值和函数签名。 */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef
#define MSD_OK              ((uint8_t)0x00)
#define MSD_ERROR           ((uint8_t)0x01)
#define SD_TRANSFER_OK      ((uint8_t)0x00)
#define SD_TRANSFER_BUSY    ((uint8_t)0x01)
#define SD_TRANSFER_ERROR   ((uint8_t)0x02)
#define SD_PRESENT          ((uint8_t)0x01)
#define SD_NOT_PRESENT      ((uint8_t)0x00)

/**
 * @brief 初始化卡并切换到 4 位总线；已成功初始化且空闲时不重复识别。
 * @retval MSD_OK 成功；MSD_ERROR 无卡、卡忙、初始化失败或从中断调用。
 * @note 必须由前台串行调用，HAL 超时依赖持续运行的 SysTick；不能关中断访问卡。
 */
uint8_t BSP_SD_Init(void);

/**
 * @brief 以轮询方式读写 512 字节逻辑扇区，地址为 LBA 而不是字节地址。
 * @param pData 缓冲区首地址，空间至少为 NumOfBlocks * 512 字节。
 * @param ReadAddr / WriteAddr 起始逻辑扇区编号，从 0 开始。
 * @param NumOfBlocks 非零块数，受卡容量和 SDIO 单次数据长度限制。
 * @param Timeout 有限超时，单位毫秒；拒绝 0 和 HAL_MAX_DELAY。
 * @retval MSD_OK HAL 数据搬运成功；MSD_ERROR 参数、卡状态或传输错误。
 * @note 数据搬运完成后，磁盘适配层仍需在有限超时内等待卡返回 TRANSFER 状态。
 *       当前实现会阻塞主循环，不能直接用于边运动边记录的最终实时方案。
 */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr,
                          uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr,
                           uint32_t NumOfBlocks, uint32_t Timeout);

/**
 * @brief 查询实际卡状态。
 * @retval SD_TRANSFER_OK 卡可接受新操作；SD_TRANSFER_BUSY 卡正在读写/编程；
 *         SD_TRANSFER_ERROR 无卡、未初始化、状态异常或从中断调用。
 * @note 查询发送 CMD13，属于前台阻塞调用，不是纯软件标志读取。
 */
uint8_t BSP_SD_GetCardState(void);

/**
 * @brief 返回卡容量等信息；保留官方 void 签名，失败时将输出结构全部清零。
 * @note 调用方应检查 LogBlockNbr 非零且 LogBlockSize 为 512 后再使用。
 */
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);

/**
 * @brief 轮询 PE15 卡座检测脚，低电平表示有卡；GPIO 上拉由 MX_GPIO_Init() 配置。
 * @note 仅检测物理插入状态，不表示已初始化或文件系统可用。检测到拔卡后，
 *       会使初始化记录失效，重新插入后必须重新初始化并重新挂载文件系统。
 */
uint8_t BSP_SD_IsDetected(void);

/* 本阶段没有 SDIO/DMA 中断和 DMA 句柄，故不提供 DMA/IT 读写及 IRQ 包装接口。
 * 这样可避免误调用官方模板的 DMA 接口后解引用空句柄。
 * 同时不提供本阶段不需要的擦除接口，文件操作由 FatFs 完成。
 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_DRIVER_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
