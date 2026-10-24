/**
  ******************************************************************************
  *  FatFs - Generic FAT file system module  R0.12c (C)ChaN, 2017
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

#ifndef _FFCONF
#define _FFCONF 68300 /* 必须与随项目移植的 FatFs R0.12c 修订号一致。 */

/*
 * 第一阶段配置说明：
 * 1. 支持 FAT12/FAT16/FAT32，32GB microSD 卡请在电脑上准备为 FAT32；
 *    本工程不支持 exFAT，也不提供自动格式化功能。
 * 2. 仅使用英文、数字及合法的 8.3 短文件名，例如 DATA0001.CSV；
 *    文件名编码与 CSV 文件内容编码是两回事，f_write 按原始字节写入内容。
 * 3. 当前只允许一个前台执行流访问同一 SD 卡；不要在中断中调用 FatFs。
 *    _FS_LOCK 只限制重复打开等非法操作，不能代替任务互斥。
 * 4. 上层文件管理、缓存、CSV 格式化与测试均留到后续阶段。
 */

/* 保留基础读写、定位、查询和目录接口；关闭字符串包装及格式化接口。
 * 后续上层使用 snprintf + f_write，避免隐式换行转换及格式能力差异。 */
#define _FS_READONLY       0
#define _FS_MINIMIZE       0
#define _USE_STRFUNC       0
#define _USE_FIND          0
#define _USE_MKFS          0
#define _USE_FASTSEEK      0
#define _USE_EXPAND        0
#define _USE_CHMOD         0
#define _USE_LABEL         0
#define _USE_FORWARD       0

/* 禁用长文件名后，不编译 option/ccsbcs.c、unicode.c 或动态内存适配。
 * _CODE_PAGE=1 表示 ASCII；_MAX_LFN 和编码选项在当前配置下不参与工作。 */
#define _CODE_PAGE         1
#define _USE_LFN           0
#define _MAX_LFN           255
#define _LFN_UNICODE       0
#define _STRF_ENCODE        3
#define _FS_RPATH          0

/* 单个物理卡、单个逻辑卷，固定 512 字节扇区。 */
#define _VOLUMES           1
#define _STR_VOLUME_ID     0
#define _VOLUME_STRS       "SD"
#define _MULTI_PARTITION   0
#define _MIN_SS            512
#define _MAX_SS            512
#define _USE_TRIM          0
#define _FS_NOFSINFO       0

/* 保留每个文件的独立扇区缓存；不启用 exFAT。 */
#define _FS_TINY           0
#define _FS_EXFAT          0

/* 当前未接入真实 RTC，使用有效固定日期，不再返回无效的 FAT 时间 0。
 * 该时间仅为文件元数据占位，不能作为传感器采样时间或任务时间戳。 */
#define _FS_NORTC          1
#define _NORTC_MON         1
#define _NORTC_MDAY        1
#define _NORTC_YEAR        2026

/* 最多跟踪两个打开对象；当前配置不是线程安全实现。
 * 将来若增加多任务访问，需同步设计卷级互斥和磁盘驱动串行化。 */
#define _FS_LOCK           2
#define _FS_REENTRANT      0
#define _FS_TIMEOUT        1000
#define _SYNC_t            void *

#endif /* _FFCONF */

