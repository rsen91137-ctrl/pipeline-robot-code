/**
  ******************************************************************************
  * @file    sd_diskio.c (based on sd_diskio_template.c v2.0.2)
  * @brief   SD Disk I/O driver
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

#include "sd_diskio.h"
#include "bsp_driver_sd.h"

/*
 * 软件超时的单位是毫秒，与 SDIO 硬件数据超时计数器的单位不同。
 * 不使用 SDMMC_DATATIMEOUT（0xFFFFFFFF）充当 HAL 的毫秒超时参数。
 * 每次块读写和每次卡就绪等待分别设置 2000ms 上限；一项完整文件操作
 * 可能包含多次扇区操作，因此这里不是整个文件操作的总耗时保证。
 * 当前实现保持官方的同步轮询模式，不占用 DMA，不保证 1.5ms 同步落盘。
 */
#define SD_DISKIO_TIMEOUT_MS   2000U
#define SD_DISKIO_SECTOR_SIZE  512U

static volatile DSTATUS Stat = STA_NOINIT;

static DSTATUS SD_CheckStatus(BYTE lun);
static DRESULT SD_WaitReady(void);
static DRESULT SD_TransferFailed(void);
static DSTATUS SD_initialize(BYTE lun);
static DSTATUS SD_status(BYTE lun);
static DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
static DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
static DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
#endif

const Diskio_drvTypeDef SD_Driver =
{
    SD_initialize,
    SD_status,
    SD_read,
#if _USE_WRITE == 1
    SD_write,
#endif
#if _USE_IOCTL == 1
    SD_ioctl,
#endif
};

/* 检测移除时同时置位 NODISK 和 NOINIT；重新插卡只清除 NODISK，
 * 必须重新执行初始化才清除 NOINIT，防止沿用上一张卡的旧状态。
 * 这里不把临时的卡内部编程 BUSY 误判为“未初始化”。 */
static DSTATUS SD_CheckStatus(BYTE lun)
{
    if (lun != 0U)
    {
        return STA_NOINIT;
    }

    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        Stat = STA_NOINIT | STA_NODISK;
    }
    else
    {
        Stat &= (DSTATUS)~STA_NODISK;
    }
    return Stat;
}

/* 等待卡完成内部编程并回到 TRANSFER。
 * BSP 将硬件错误和 BUSY 区分；错误立即退出，BUSY 也有有限超时。
 * 超时或硬件错误后标记需要重新初始化，由后续上层处理失败并重新挂载，
 * 不在驱动内部无限重试，也不自动格式化或假装写入成功。 */
static DRESULT SD_WaitReady(void)
{
    uint32_t tick_start = HAL_GetTick();
    uint8_t card_state;

    for (;;)
    {
        if ((SD_CheckStatus(0U) & STA_NOINIT) != 0U)
        {
            return RES_NOTRDY;
        }

        card_state = BSP_SD_GetCardState();
        if (card_state == SD_TRANSFER_OK)
        {
            return RES_OK;
        }
        if (card_state == SD_TRANSFER_ERROR)
        {
            return SD_TransferFailed();
        }
        if ((uint32_t)(HAL_GetTick() - tick_start) >= SD_DISKIO_TIMEOUT_MS)
        {
            return SD_TransferFailed();
        }
    }
}

/* 先更新检测状态，拔卡返回 NOTRDY；其他传输错误返回 ERROR。
 * 所有失败都使旧初始化状态失效，避免后续操作继续使用可疑的卡状态。 */
static DRESULT SD_TransferFailed(void)
{
    DSTATUS status = SD_CheckStatus(0U);
    Stat |= STA_NOINIT;
    return ((status & STA_NODISK) != 0U) ? RES_NOTRDY : RES_ERROR;
}

static DSTATUS SD_initialize(BYTE lun)
{
    if (lun != 0U)
    {
        return STA_NOINIT;
    }

    Stat = STA_NOINIT;
    if ((SD_CheckStatus(lun) & STA_NODISK) != 0U)
    {
        return Stat;
    }

    /* BSP 内部按“一位低速识别 -> 四位数据传输”初始化卡。
     * 只有初始化和卡状态均成功时才向 FatFs 报告就绪。 */
    if (BSP_SD_Init() == MSD_OK)
    {
        Stat = 0U;
        if (SD_WaitReady() != RES_OK)
        {
            Stat |= STA_NOINIT;
        }
    }
    return SD_CheckStatus(lun);
}

static DSTATUS SD_status(BYTE lun)
{
    return SD_CheckStatus(lun);
}

static DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT result;

    if ((lun != 0U) || (buff == 0) || (count == 0U))
    {
        return RES_PARERR;
    }
    if ((SD_CheckStatus(lun) & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    result = SD_WaitReady();
    if (result != RES_OK)
    {
        return result;
    }

    if (BSP_SD_ReadBlocks((uint32_t *)buff, (uint32_t)sector,
                          (uint32_t)count, SD_DISKIO_TIMEOUT_MS) != MSD_OK)
    {
        return SD_TransferFailed();
    }

    /* HAL 搬运完成后仍确认卡状态，确保错误不会被当作成功上报。 */
    return SD_WaitReady();
}

#if _USE_WRITE == 1
static DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT result;

    if ((lun != 0U) || (buff == 0) || (count == 0U))
    {
        return RES_PARERR;
    }
    if ((SD_CheckStatus(lun) & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    result = SD_WaitReady();
    if (result != RES_OK)
    {
        return result;
    }

    /* 官方 BSP/HAL 写接口未使用 const，但同步写入不会修改源缓冲区。
     * 底层采用目标工程现有 HAL 版本，不要求调用方按 uint32_t 搬运数据。 */
    if (BSP_SD_WriteBlocks((uint32_t *)buff, (uint32_t)sector,
                           (uint32_t)count, SD_DISKIO_TIMEOUT_MS) != MSD_OK)
    {
        return SD_TransferFailed();
    }

    /* 传输结束并不等于卡内部编程结束，等待 TRANSFER 后才返回成功。 */
    return SD_WaitReady();
}
#endif

#if _USE_IOCTL == 1
static DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    BSP_SD_CardInfo card_info = {0};

    if (lun != 0U)
    {
        return RES_PARERR;
    }
    if ((SD_CheckStatus(lun) & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
        case CTRL_SYNC:
            /* f_sync/f_close 要求确认卡已完成编程，不能无条件返回成功。 */
            return SD_WaitReady();

        case GET_SECTOR_COUNT:
        case GET_SECTOR_SIZE:
        case GET_BLOCK_SIZE:
            if (buff == 0)
            {
                return RES_PARERR;
            }
            BSP_SD_GetCardInfo(&card_info);
            if ((card_info.LogBlockNbr == 0U) ||
                (card_info.LogBlockSize != SD_DISKIO_SECTOR_SIZE))
            {
                return SD_TransferFailed();
            }
            if (cmd == GET_SECTOR_COUNT)
            {
                *(DWORD *)buff = (DWORD)card_info.LogBlockNbr;
            }
            else if (cmd == GET_SECTOR_SIZE)
            {
                *(WORD *)buff = (WORD)card_info.LogBlockSize;
            }
            else
            {
                /* 现 BSP 未公开擦除单元，使用 FatFs 允许的保守值 1 扇区。
                 * 当前关闭格式化和 TRIM，不把逻辑块长伪称为真实擦除单元。 */
                *(DWORD *)buff = 1U;
            }
            return RES_OK;

        default:
            return RES_PARERR;
    }
}
#endif

