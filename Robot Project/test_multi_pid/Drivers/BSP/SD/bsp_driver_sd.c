/**
 ******************************************************************************
  * @file    bsp_driver_sd.c for F4 (based on stm324x9i_eval_sd.c)
  * @brief   This file includes a generic uSD card driver.
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

#include "bsp_driver_sd.h"
#include "sdio.h"
#include <string.h>

/* 只有完整通过卡识别和 4 位切换才置 1；物理拔卡或传输错误将使其失效。
 * 本模块设计为单一前台调用者使用，不提供线程或中断间的并发保护。
 */
static uint8_t sd_initialized = 0U;

/**
 * @brief 检查轮询传输前置条件，避免 HAL 内部的地址加法溢出或数据长度截断。
 * @note SDIO DLEN 的有效数据长度为 25 位，512 字节扇区最多一次传输 65535 块。
 */
static uint8_t SD_CheckTransfer(uint32_t *pData, uint32_t BlockAddr,
                                uint32_t NumOfBlocks, uint32_t Timeout)
{
    if ((__get_IPSR() != 0U) || (pData == NULL) ||
        (NumOfBlocks == 0U) || (NumOfBlocks > (0x01FFFFFFU / 512U)) ||
        (Timeout == 0U) || (Timeout == HAL_MAX_DELAY))
    {
        return MSD_ERROR;
    }

    if ((BSP_SD_IsDetected() != SD_PRESENT) || (sd_initialized == 0U) ||
        (hsd.Instance != SDIO) || (hsd.State != HAL_SD_STATE_READY))
    {
        return MSD_ERROR;
    }

    /* 用减法验证范围，不能直接计算 BlockAddr + NumOfBlocks 后再判断。 */
    if ((hsd.SdCard.LogBlockSize != 512U) ||
        (BlockAddr >= hsd.SdCard.LogBlockNbr) ||
        (NumOfBlocks > (hsd.SdCard.LogBlockNbr - BlockAddr)))
    {
        return MSD_ERROR;
    }

    return MSD_OK;
}

uint8_t BSP_SD_Init(void)
{
    uint8_t card_state;

    if ((__get_IPSR() != 0U) || (BSP_SD_IsDetected() != SD_PRESENT))
    {
        return MSD_ERROR;
    }

    if (sd_initialized != 0U)
    {
        card_state = BSP_SD_GetCardState();
        if (card_state == SD_TRANSFER_OK)
        {
            return MSD_OK;
        }
        if (card_state == SD_TRANSFER_BUSY)
        {
            /* 卡仍在执行上一操作，不能为了初始化而打断它。 */
            return MSD_ERROR;
        }
        sd_initialized = 0U;
    }

    /* 失败重试或重新插卡时先释放旧 HAL 状态，确保 HAL 再次运行 MSP 初始化。
     * 启动时主程序只填写了参数，句柄仍为 RESET，此时无需反初始化。
     */
    if ((hsd.Instance == SDIO) && (hsd.State != HAL_SD_STATE_RESET))
    {
        if (HAL_SD_DeInit(&hsd) != HAL_OK)
        {
            return MSD_ERROR;
        }
    }

    MX_SDIO_SD_Init();
    if (HAL_SD_Init(&hsd) != HAL_OK)
    {
        return MSD_ERROR;
    }

    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
    {
        /* 不伪装成成功，也不回退为未经上层知晓的 1 位工作模式。
         * 保留 HAL 错误码供调试器查看，下次显式初始化时会重新释放句柄。
         */
        return MSD_ERROR;
    }

    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        return MSD_ERROR;
    }

    sd_initialized = 1U;
    return MSD_OK;
}

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr,
                          uint32_t NumOfBlocks, uint32_t Timeout)
{
    if (SD_CheckTransfer(pData, ReadAddr, NumOfBlocks, Timeout) != MSD_OK)
    {
        return MSD_ERROR;
    }

    if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)pData, ReadAddr,
                          NumOfBlocks, Timeout) != HAL_OK)
    {
        sd_initialized = 0U;
        return MSD_ERROR;
    }

    /* 传输期间拔卡也应返回失败，不能把内存搬运完成当成卡仍然可用。 */
    return (BSP_SD_IsDetected() == SD_PRESENT) ? MSD_OK : MSD_ERROR;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr,
                           uint32_t NumOfBlocks, uint32_t Timeout)
{
    if (SD_CheckTransfer(pData, WriteAddr, NumOfBlocks, Timeout) != MSD_OK)
    {
        return MSD_ERROR;
    }

    if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr,
                           NumOfBlocks, Timeout) != HAL_OK)
    {
        sd_initialized = 0U;
        return MSD_ERROR;
    }

    return (BSP_SD_IsDetected() == SD_PRESENT) ? MSD_OK : MSD_ERROR;
}

uint8_t BSP_SD_GetCardState(void)
{
    HAL_SD_CardStateTypeDef card_state;

    if ((__get_IPSR() != 0U) || (BSP_SD_IsDetected() != SD_PRESENT) ||
        (sd_initialized == 0U) || (hsd.Instance != SDIO))
    {
        return SD_TRANSFER_ERROR;
    }

    /* HAL 正在执行操作时不再向卡发送 CMD13，避免重入。
     * 本阶段没有异步读写，正常返回到前台时 HAL 应为 READY。
     */
    if ((hsd.State == HAL_SD_STATE_RESET) ||
        (hsd.State == HAL_SD_STATE_TIMEOUT) || (hsd.State == HAL_SD_STATE_ERROR))
    {
        sd_initialized = 0U;
        return SD_TRANSFER_ERROR;
    }
    if (hsd.State != HAL_SD_STATE_READY)
    {
        return SD_TRANSFER_BUSY;
    }

    card_state = HAL_SD_GetCardState(&hsd);
    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        return SD_TRANSFER_ERROR;
    }
    if (hsd.ErrorCode != HAL_SD_ERROR_NONE)
    {
        /* HAL 的状态查询可能同时给出状态位和命令错误；错误不能被状态位掩盖。 */
        sd_initialized = 0U;
        return SD_TRANSFER_ERROR;
    }
    if (card_state == HAL_SD_CARD_TRANSFER)
    {
        return SD_TRANSFER_OK;
    }
    if ((card_state == HAL_SD_CARD_SENDING) ||
        (card_state == HAL_SD_CARD_RECEIVING) || (card_state == HAL_SD_CARD_PROGRAMMING))
    {
        return SD_TRANSFER_BUSY;
    }

    /* 未得到有效工作状态（包括 CMD13 无响应）时明确报告错误。 */
    sd_initialized = 0U;
    return SD_TRANSFER_ERROR;
}

void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
    if (CardInfo == NULL)
    {
        return;
    }

    /* 官方 void 接口没有返回错误的通道，清零可防止上层继续使用过期容量。 */
    memset(CardInfo, 0, sizeof(*CardInfo));
    if ((BSP_SD_IsDetected() != SD_PRESENT) || (sd_initialized == 0U) ||
        (hsd.Instance != SDIO) || (hsd.State != HAL_SD_STATE_READY))
    {
        return;
    }
    if (HAL_SD_GetCardInfo(&hsd, CardInfo) != HAL_OK)
    {
        memset(CardInfo, 0, sizeof(*CardInfo));
    }
}

uint8_t BSP_SD_IsDetected(void)
{
    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin) == GPIO_PIN_RESET)
    {
        return SD_PRESENT;
    }

    sd_initialized = 0U;
    return SD_NOT_PRESENT;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
