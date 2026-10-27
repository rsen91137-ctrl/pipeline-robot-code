/**
 * @file sd_card_test.c
 * @brief 第三阶段独立存储测试：确定性 CSV、分段追加、定位与两遍完整读回。
 * @note 只生成虚拟数据，不读取真实传感器、电机或无线模块，不修改旧文件。
 */
#include "sd_card_test.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

volatile SD_Card_Test_Report g_SD_Card_Test_Result; //g前缀表示全局变量（global）

#if SD_CARD_TEST_MODE

static const char sd_test_header[] = "seq,time_us,laser_um,motor_angle_mrad";
static char sd_test_filename[13];
static char sd_test_line[96];
static uint8_t sd_test_read_buffer[SD_CARD_TEST_READ_BLOCK_BYTES];
static uint8_t sd_test_has_run = 0U;
static uint8_t sd_test_write_timing_active = 0U;
static uint32_t sd_test_write_start_tick = 0U;
static uint32_t sd_test_last_write_end_tick = 0U;

/* 预期字节流的游标；复用 96 字节行缓冲，不在 RAM 中存放整个 CSV 文件。
 * 每次读取校验均从表头开始重新生成，定位检查不使用这个游标。 */
static uint32_t sd_test_expected_next_seq = 0U;
static uint32_t sd_test_expected_index = 0U;
static uint32_t sd_test_expected_length = 0U;
static uint8_t sd_test_expected_is_header = 1U;

/**
 * @brief 读取存储模块的纯内存状态快照，供 Keil 查看。
 * @note Get_Current_State 不改存储错误诊断；本函数也不覆盖测试的 last_result。
 */
static void SD_Test_Refresh_Storage_Info(void)
{
    SD_Card_Info info;
    if (SD_Card_Get_Current_State(&info) == SD_CARD_OK)
    {
        g_SD_Card_Test_Result.storage_info = info;
        g_SD_Card_Test_Result.storage_info_valid = 1U;
    }
    else
    {
        g_SD_Card_Test_Result.storage_info_valid = 0U;
    }
}

/**
 * @brief 记录首个测试失败并停止；不关闭文件、不丢弃缓存、不自动复位。
 * @param error 测试层的失败原因，实际存储返回值另见 last_result。
 * @retval 0 便于调用方立即逐层返回。
 */
static uint8_t SD_Test_Fail(SD_Card_Test_Error error)
{
    if (g_SD_Card_Test_Result.status != SD_CARD_TEST_FAILED)
    {
        if (sd_test_write_timing_active != 0U)
        {
            g_SD_Card_Test_Result.write_elapsed_ms = HAL_GetTick() - sd_test_write_start_tick;
        }
        g_SD_Card_Test_Result.test_error = error;
        SD_Test_Refresh_Storage_Info();
        /* 最后发布 FAILED，保持 stage、缓存和底层故障原因供调试器观察。 */
        g_SD_Card_Test_Result.status = SD_CARD_TEST_FAILED;
    }
    return 0U;
}

/**
 * @brief 检查一次存储接口调用；缓冲满的可恢复分支由接收行函数单独处理。
 * @param result 待检查的存储接口返回结果。
 * @retval 1 成功；0 已记录错误并停止测试。
 */
static uint8_t SD_Test_Storage_Call_Ok(SD_Card_Result result)
{
    g_SD_Card_Test_Result.last_result = result;
    if (result != SD_CARD_OK)
    {
        return SD_Test_Fail(SD_CARD_TEST_ERROR_STORAGE_CALL);
    }
    return 1U;
}

/**
 * @brief 统计写入阶段一个显式卡操作的停顿，不把准备、读回或快照复制计入。
 * @param operation 写缓存、保存、写模式关闭或打开追加文件的无参操作。
 * @return 被调用操作的原始结果，供上层决定继续或记录失败。
 * @note HAL_GetTick 差值使用无符号减法，允许一次毫秒计数回绕。
 */
static SD_Card_Result SD_Test_Timed_Write_Operation(SD_Card_Result (*operation)(void))
{
    uint32_t start_tick = HAL_GetTick();
    SD_Card_Result result = operation();
    uint32_t end_tick = HAL_GetTick();
    uint32_t elapsed = end_tick - start_tick;
    sd_test_last_write_end_tick = end_tick;
    if (elapsed > g_SD_Card_Test_Result.max_write_pause_ms)
    {
        g_SD_Card_Test_Result.max_write_pause_ms = elapsed;
    }
    SD_Test_Refresh_Storage_Info();
    return result;
}

/**
 * @brief 为计时函数适配“打开已有文件用于追加”的带文件名接口。
 * @note 只重新打开本次刚创建并关闭的文件，绝不据此创建或覆盖另一个文件。
 */
static SD_Card_Result SD_Test_Open_Append_File(void)
{
    return SD_Card_Open_File_To_Add_Data(sd_test_filename);
}

/**
 * @brief 顺序尝试 1000 个安全短文件名，只使用 Create_New，不打开重名文件。
 * @retval 1 已创建空文件；0 命名/存储错误或 1000 个名称全部占用。
 */
static uint8_t SD_Test_Create_Unused_File(void)
{
    uint32_t candidate;
    uint32_t index;
    int length;
    SD_Card_Result result;

    for (candidate = 1U; candidate <= SD_CARD_TEST_MAX_FILE_NAMES; ++candidate)
    {
        length = snprintf(sd_test_filename, sizeof(sd_test_filename), "ST%06lu.CSV",
                          (unsigned long)candidate);
        if (length != 12)
        {
            return SD_Test_Fail(SD_CARD_TEST_ERROR_FILE_NAME_FORMAT);
        }
        /* 报告对象是 volatile，逐字节复制包括 NUL，不强制去掉 volatile。 */
        for (index = 0U; index < sizeof(sd_test_filename); ++index)
        {
            g_SD_Card_Test_Result.filename[index] = sd_test_filename[index];
        }
        result = SD_Card_Create_New_File(sd_test_filename);
        g_SD_Card_Test_Result.last_result = result;
        if (result == SD_CARD_OK)
        {
            SD_Test_Refresh_Storage_Info();
            return 1U;
        }
        if (result != SD_CARD_FILE_ALREADY_EXISTS)
        {
            return SD_Test_Fail(SD_CARD_TEST_ERROR_STORAGE_CALL);
        }
    }
    return SD_Test_Fail(SD_CARD_TEST_ERROR_NO_FREE_FILE_NAME);
}

/**
 * @brief 按固定公式生成一条不带换行的 CSV 记录，整数不使用浮点格式化。
 * @param seq 从 0 开始的记录序号。
 * @param length 输出正文的字节数；不含 NUL、CRLF。
 * @retval 1 格式化成功；0 格式化失败且已记录测试错误。
 * @note time_us=seq*1500 只是虚拟数据，不代表按 1.5ms 间隔实际采样。
 */
static uint8_t SD_Test_Format_Row(uint32_t seq, uint32_t *length)
{
    int count = snprintf(sd_test_line, sizeof(sd_test_line), "%lu,%lu,%lu,%lu",
                         (unsigned long)seq,
                         (unsigned long)(seq * 1500U),
                         (unsigned long)(100000U + (seq % 1000U)),
                         (unsigned long)(seq * 5U));
    /* 另留两个字节，供定位校验时把 CRLF 接到这同一缓冲区尾部。 */
    if ((count < 0) || ((uint32_t)count > (sizeof(sd_test_line) - 2U)))
    {
        return SD_Test_Fail(SD_CARD_TEST_ERROR_CSV_FORMAT);
    }
    *length = (uint32_t)count;
    return 1U;
}

/**
 * @brief 将整行交给纯内存接收接口；缓存满时显式写出，再重试同一行一次。
 * @param text 不含 CR/LF 的文本，接收接口会添加 CRLF。
 * @param length 正文长度，成功后计入预期文件字节数。
 * @param write_stage 正常接收本段数据时显示的测试阶段。
 * @param drain_stage 缓存满后批量写出时显示的测试阶段。
 * @retval 1 整行接收成功；0 已记录错误并停止。
 * @note BUFFER_FULL 保证此次整行完全未接收，所以此处重试不会重复半行。
 */
static uint8_t SD_Test_Add_Line(const char *text, uint32_t length,
                               SD_Card_Test_Stage write_stage,
                               SD_Card_Test_Stage drain_stage)
{
    SD_Card_Result result;
    g_SD_Card_Test_Result.stage = write_stage;
    result = SD_Card_Add_One_Line_To_Memory(text);
    if (result == SD_CARD_MEMORY_BUFFER_FULL)
    {
        g_SD_Card_Test_Result.stage = drain_stage;
        if (SD_Test_Storage_Call_Ok(SD_Test_Timed_Write_Operation(
                SD_Card_Write_Memory_Data_To_File)) == 0U)
        {
            return 0U;
        }
        g_SD_Card_Test_Result.stage = write_stage;
        result = SD_Card_Add_One_Line_To_Memory(text);
        if (result == SD_CARD_MEMORY_BUFFER_FULL)
        {
            g_SD_Card_Test_Result.last_result = result;
            return SD_Test_Fail(SD_CARD_TEST_ERROR_BUFFER_RETRY);
        }
    }
    if (SD_Test_Storage_Call_Ok(result) == 0U)
    {
        return 0U;
    }
    g_SD_Card_Test_Result.bytes_written_expected += length + 2U;
    return 1U;
}

/**
 * @brief 写入一段确定性记录，每 1000 行执行一次显式 Save_All_Data_To_Card。
 * @param first_seq 首条序号，包含。
 * @param end_seq 末尾序号，不包含。
 * @param write_stage 本段接收记录时显示的测试阶段。
 * @param drain_stage 本段缓存满而写出数据时显示的测试阶段。
 * @param save_stage 本段周期同步保存时显示的测试阶段。
 * @retval 1 本段全部接收成功；0 已记录错误并停止。
 * @note rows_written 只在完整行接收成功后增加；表头不计入记录数。
 */
static uint8_t SD_Test_Write_Rows(uint32_t first_seq, uint32_t end_seq,
                                 SD_Card_Test_Stage write_stage,
                                 SD_Card_Test_Stage drain_stage,
                                 SD_Card_Test_Stage save_stage)
{
    uint32_t seq;
    uint32_t length;
    for (seq = first_seq; seq < end_seq; ++seq)
    {
        g_SD_Card_Test_Result.stage = write_stage;
        if (SD_Test_Format_Row(seq, &length) == 0U)
        {
            return 0U;
        }
        if (seq == SD_CARD_TEST_SEEK_ROW_INDEX)
        {
            g_SD_Card_Test_Result.seek_test_offset =
                g_SD_Card_Test_Result.bytes_written_expected;
        }
        if (SD_Test_Add_Line(sd_test_line, length, write_stage, drain_stage) == 0U)
        {
            return 0U;
        }
        ++g_SD_Card_Test_Result.rows_written;
        if (((seq + 1U) % SD_CARD_TEST_SAVE_INTERVAL_ROWS) == 0U)
        {
            g_SD_Card_Test_Result.stage = save_stage;
            if (SD_Test_Storage_Call_Ok(SD_Test_Timed_Write_Operation(
                    SD_Card_Save_All_Data_To_Card)) == 0U)
            {
                return 0U;
            }
        }
    }
    return 1U;
}

/**
 * @brief 将预期字节流重置到 CSV 表头，不访问卡。
 * @note 之后逐行重新调用相同的确定性公式，避免把整份预期文件放入 RAM。
 */
static void SD_Test_Reset_Expected_Stream(void)
{
    sd_test_expected_next_seq = 0U;
    sd_test_expected_index = 0U;
    sd_test_expected_length = (uint32_t)(sizeof(sd_test_header) - 1U);
    sd_test_expected_is_header = 1U;
}

/**
 * @brief 生成预期文件的下一个字节，表头和记录末尾均显式生成 CRLF。
 * @param value 输出 0..255 的字节，文件结束时输出 END_OF_STREAM=0x100。
 * @retval 1 已输出；0 行格式化失败且已终止测试。
 */
static uint8_t SD_Test_Next_Expected_Byte(uint16_t *value)
{
    uint32_t length;
    if (sd_test_expected_index >= (sd_test_expected_length + 2U))
    {
        if (sd_test_expected_next_seq >= SD_CARD_TEST_ROW_COUNT)
        {
            *value = SD_CARD_TEST_END_OF_STREAM;
            return 1U;
        }
        if (SD_Test_Format_Row(sd_test_expected_next_seq, &length) == 0U)
        {
            return 0U;
        }
        ++sd_test_expected_next_seq;
        sd_test_expected_is_header = 0U;
        sd_test_expected_length = length;
        sd_test_expected_index = 0U;
    }

    if (sd_test_expected_index < sd_test_expected_length)
    {
        *value = (uint16_t)(uint8_t)((sd_test_expected_is_header != 0U) ?
                    sd_test_header[sd_test_expected_index] : sd_test_line[sd_test_expected_index]);
    }
    else
    {
        *value = (sd_test_expected_index == sd_test_expected_length) ?
                    (uint16_t)'\r' : (uint16_t)'\n';
    }
    ++sd_test_expected_index;
    return 1U;
}

/**
 * @brief 保留首个字节差异，支持缺失或多余数据的一侧 EOF 标记。
 * @param offset 从文件开头起的零基字节位置。
 * @param expected 预期字节值；0x100 表示预期数据流已结束。
 * @param actual 实际字节值；0x100 表示实际文件已结束。
 * @note 本函数不覆盖已经记录的首错信息，也不自行访问文件。
 */
static void SD_Test_Record_Mismatch(uint32_t offset, uint16_t expected, uint16_t actual)
{
    if (g_SD_Card_Test_Result.first_mismatch_valid == 0U)
    {
        g_SD_Card_Test_Result.first_mismatch_offset = offset;
        g_SD_Card_Test_Result.first_mismatch_expected = expected;
        g_SD_Card_Test_Result.first_mismatch_actual = actual;
        g_SD_Card_Test_Result.first_mismatch_valid = 1U;
    }
}

/**
 * @brief 定位到 seq=12345 的已知记录，比较整行，然后显式定位回文件开头。
 * @note 定位检查不计入 bytes_verified；它验证追加后半段确实可被随机读取。
 */
static uint8_t SD_Test_Check_Seek(void)
{
    uint32_t length;
    uint32_t actual = 0U;
    uint32_t index;
    uint32_t offset = g_SD_Card_Test_Result.seek_test_offset;

    if (SD_Test_Storage_Call_Ok(SD_Card_Set_File_Read_Position(offset)) == 0U)
    {
        return 0U;
    }
    if (SD_Test_Format_Row(SD_CARD_TEST_SEEK_ROW_INDEX, &length) == 0U)
    {
        return 0U;
    }
    sd_test_line[length] = '\r';
    sd_test_line[length + 1U] = '\n';
    length += 2U;
    if (SD_Test_Storage_Call_Ok(SD_Card_Read_File_Data(sd_test_read_buffer, length, &actual)) == 0U)
    {
        return 0U;
    }
    if (actual > length)
    {
        return SD_Test_Fail(SD_CARD_TEST_ERROR_READ_COUNT);
    }
    for (index = 0U; index < actual; ++index)
    {
        if (sd_test_read_buffer[index] != (uint8_t)sd_test_line[index])
        {
            SD_Test_Record_Mismatch(offset + index, (uint16_t)(uint8_t)sd_test_line[index],
                                    (uint16_t)sd_test_read_buffer[index]);
            return SD_Test_Fail(SD_CARD_TEST_ERROR_DATA_MISMATCH);
        }
    }
    if (actual < length)
    {
        SD_Test_Record_Mismatch(offset + actual, (uint16_t)(uint8_t)sd_test_line[actual],
                                SD_CARD_TEST_END_OF_STREAM);
        return SD_Test_Fail(SD_CARD_TEST_ERROR_UNEXPECTED_EOF);
    }
    if (SD_Test_Storage_Call_Ok(SD_Card_Set_File_Read_Position(0U)) == 0U)
    {
        return 0U;
    }
    g_SD_Card_Test_Result.seek_check_passed = 1U;
    SD_Test_Refresh_Storage_Info();
    return 1U;
}

/**
 * @brief 以 512 字节分块读取，重新生成 CSV 并逐字节比较，直到明确读到 EOF。
 * @note 检查错字节、缺失字节和额外字节；不把“成功读到部分内容”当作通过。
 */
static uint8_t SD_Test_Verify_Whole_File(void)
{
    uint32_t actual;
    uint32_t index;
    uint16_t expected;

    SD_Test_Reset_Expected_Stream();
    g_SD_Card_Test_Result.bytes_verified = 0U;
    g_SD_Card_Test_Result.eof_verified = 0U;
    for (;;)
    {
        actual = 0U;
        if (SD_Test_Storage_Call_Ok(SD_Card_Read_File_Data(
                sd_test_read_buffer, sizeof(sd_test_read_buffer), &actual)) == 0U)
        {
            return 0U;
        }
        if (actual > sizeof(sd_test_read_buffer))
        {
            return SD_Test_Fail(SD_CARD_TEST_ERROR_READ_COUNT);
        }
        for (index = 0U; index < actual; ++index)
        {
            if (SD_Test_Next_Expected_Byte(&expected) == 0U)
            {
                return 0U;
            }
            if (expected == SD_CARD_TEST_END_OF_STREAM)
            {
                SD_Test_Record_Mismatch(g_SD_Card_Test_Result.bytes_verified, expected,
                                        (uint16_t)sd_test_read_buffer[index]);
                return SD_Test_Fail(SD_CARD_TEST_ERROR_EXTRA_DATA);
            }
            if (expected != (uint16_t)sd_test_read_buffer[index])
            {
                SD_Test_Record_Mismatch(g_SD_Card_Test_Result.bytes_verified, expected,
                                        (uint16_t)sd_test_read_buffer[index]);
                return SD_Test_Fail(SD_CARD_TEST_ERROR_DATA_MISMATCH);
            }
            ++g_SD_Card_Test_Result.bytes_verified;
        }
        if (actual == 0U)
        {
            if (SD_Test_Next_Expected_Byte(&expected) == 0U)
            {
                return 0U;
            }
            if (expected != SD_CARD_TEST_END_OF_STREAM)
            {
                SD_Test_Record_Mismatch(g_SD_Card_Test_Result.bytes_verified, expected,
                                        SD_CARD_TEST_END_OF_STREAM);
                return SD_Test_Fail(SD_CARD_TEST_ERROR_UNEXPECTED_EOF);
            }
            if (g_SD_Card_Test_Result.bytes_verified !=
                g_SD_Card_Test_Result.bytes_written_expected)
            {
                return SD_Test_Fail(SD_CARD_TEST_ERROR_VERIFIED_LENGTH);
            }
            g_SD_Card_Test_Result.eof_verified = 1U;
            return 1U;
        }
    }
}

#endif /* SD_CARD_TEST_MODE */

/**
 * @brief 执行一次完整同步测试，不自动重复运行。
 * @details 创建一个未占用的 STxxxxxx.CSV，先写前 10000 行并关闭，再以已有文件
 *          追加后 10000 行；读取时先定位检查，再从头逐字节比较整份文件。
 *          第一遍完成后关闭、卸载、重新准备，再做第二遍独立完整读取。
 *          只有所有数据、EOF、关闭和结束使用均成功才报告 PASSED。
 * @note 测试失败立即停止，不自动 Abort/删除/格式化或重跑。平均吞吐未达目标只反映
 *       到独立性能标志，不改变数据正确性结论。MODE=0 时此函数不产生任何副作用。
 */
void SD_Card_Test_Run_Storage_And_Read_Back(void)
{
#if SD_CARD_TEST_MODE
    uint32_t pass;
    uint32_t elapsed;

    if ((sd_test_has_run != 0U) || (g_SD_Card_Test_Result.status != SD_CARD_TEST_NOT_RUN))
    {
        return;
    }
    sd_test_has_run = 1U;
    g_SD_Card_Test_Result.status = SD_CARD_TEST_RUNNING;
    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_PREPARE;
    if (SD_Test_Storage_Call_Ok(SD_Card_Prepare_For_Use()) == 0U)
    {
        return;
    }

    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_FIND_NEW_FILE;
    if (SD_Test_Create_Unused_File() == 0U)
    {
        return;
    }
    sd_test_write_start_tick = HAL_GetTick();
    sd_test_write_timing_active = 1U;

    if (SD_Test_Add_Line(sd_test_header, (uint32_t)(sizeof(sd_test_header) - 1U),
                         SD_CARD_TEST_STAGE_WRITE_FIRST_HALF,
                         SD_CARD_TEST_STAGE_DRAIN_FIRST_HALF) == 0U)
    {
        return;
    }
    if (SD_Test_Write_Rows(0U, SD_CARD_TEST_FIRST_HALF_ROWS,
                           SD_CARD_TEST_STAGE_WRITE_FIRST_HALF,
                           SD_CARD_TEST_STAGE_DRAIN_FIRST_HALF,
                           SD_CARD_TEST_STAGE_SAVE_FIRST_HALF) == 0U)
    {
        return;
    }
    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_CLOSE_FIRST_HALF;
    if (SD_Test_Storage_Call_Ok(SD_Test_Timed_Write_Operation(
            SD_Card_Save_And_Close_File)) == 0U)
    {
        return;
    }
    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_OPEN_APPEND;
    if (SD_Test_Storage_Call_Ok(SD_Test_Timed_Write_Operation(SD_Test_Open_Append_File)) == 0U)
    {
        return;
    }
    if (SD_Test_Write_Rows(SD_CARD_TEST_FIRST_HALF_ROWS, SD_CARD_TEST_ROW_COUNT,
                           SD_CARD_TEST_STAGE_WRITE_SECOND_HALF,
                           SD_CARD_TEST_STAGE_DRAIN_SECOND_HALF,
                           SD_CARD_TEST_STAGE_SAVE_SECOND_HALF) == 0U)
    {
        return;
    }
    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_CLOSE_APPEND;
    if (SD_Test_Storage_Call_Ok(SD_Test_Timed_Write_Operation(
            SD_Card_Save_And_Close_File)) == 0U)
    {
        return;
    }

    /* 使用最终关闭刚返回时的时刻，排除随后报告快照复制的时间。 */
    elapsed = sd_test_last_write_end_tick - sd_test_write_start_tick;
    sd_test_write_timing_active = 0U;
    g_SD_Card_Test_Result.write_elapsed_ms = elapsed;
    if (elapsed != 0U)
    {
        g_SD_Card_Test_Result.rows_per_second = (uint32_t)(
            ((uint64_t)g_SD_Card_Test_Result.rows_written * 1000U) / elapsed);
        g_SD_Card_Test_Result.avg_meets_667hz =
            (((uint64_t)elapsed * 1000U) <=
             ((uint64_t)g_SD_Card_Test_Result.rows_written * 1500U)) ? 1U : 0U;
        g_SD_Card_Test_Result.throughput_valid = 1U;
    }

    for (pass = 1U; pass <= 2U; ++pass)
    {
        g_SD_Card_Test_Result.current_read_pass = (uint8_t)pass;
        g_SD_Card_Test_Result.seek_check_passed = 0U;
        if (pass == 2U)
        {
            g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_REMOUNT;
            if (SD_Test_Storage_Call_Ok(SD_Card_Prepare_For_Use()) == 0U)
            {
                return;
            }
        }
        g_SD_Card_Test_Result.stage = (pass == 1U) ? SD_CARD_TEST_STAGE_OPEN_READ_FIRST :
                                                                   SD_CARD_TEST_STAGE_OPEN_READ_SECOND;
        if (SD_Test_Storage_Call_Ok(SD_Card_Open_File_To_Read_Data(sd_test_filename)) == 0U)
        {
            return;
        }
        g_SD_Card_Test_Result.stage = (pass == 1U) ? SD_CARD_TEST_STAGE_SEEK_FIRST :
                                                                   SD_CARD_TEST_STAGE_SEEK_SECOND;
        if (SD_Test_Check_Seek() == 0U)
        {
            return;
        }
        g_SD_Card_Test_Result.stage = (pass == 1U) ? SD_CARD_TEST_STAGE_VERIFY_FIRST :
                                                                   SD_CARD_TEST_STAGE_VERIFY_SECOND;
        if (SD_Test_Verify_Whole_File() == 0U)
        {
            return;
        }
        g_SD_Card_Test_Result.completed_read_passes = (uint8_t)pass;
        g_SD_Card_Test_Result.stage = (pass == 1U) ? SD_CARD_TEST_STAGE_CLOSE_READ_FIRST :
                                                                   SD_CARD_TEST_STAGE_CLOSE_READ_SECOND;
        if (SD_Test_Storage_Call_Ok(SD_Card_Save_And_Close_File()) == 0U)
        {
            return;
        }
        g_SD_Card_Test_Result.stage = (pass == 1U) ? SD_CARD_TEST_STAGE_UNMOUNT_BEFORE_SECOND :
                                                                   SD_CARD_TEST_STAGE_FINISH;
        if (SD_Test_Storage_Call_Ok(SD_Card_Finish_Using()) == 0U)
        {
            return;
        }
    }

    SD_Test_Refresh_Storage_Info();
    g_SD_Card_Test_Result.stage = SD_CARD_TEST_STAGE_DONE;
    g_SD_Card_Test_Result.status = SD_CARD_TEST_PASSED;
#endif
}

/**
 * @brief 在前台循环中显示测试结果；本函数绝不调用任何存储接口。
 * @details GPIOG pin1 低有效：通过时常亮，失败时每 500ms 翻转亮灭；尚未运行或
 *          正在同步测试时熄灭。无阻塞延时，调用频率决定 LED 刷新的及时程度。
 * @note MODE=0 时不读取时钟也不改变 LED；GPIO 初始化由项目原有代码负责。
 */
void SD_Card_Test_Show_Result(void)  //纯用来测试函数功能的函数
{
#if SD_CARD_TEST_MODE
    static SD_Card_Test_Status previous_status = SD_CARD_TEST_NOT_RUN;
    static uint32_t last_change_tick = 0U;
    static uint8_t failure_led_on = 0U;
    SD_Card_Test_Status status = g_SD_Card_Test_Result.status;
    uint32_t now = HAL_GetTick();

    if (status == SD_CARD_TEST_PASSED)  //测试通过时，LED常亮
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else if (status == SD_CARD_TEST_FAILED)  //测试失败时，LED每500ms翻转一次
    {
        if (previous_status != SD_CARD_TEST_FAILED)
        {
            last_change_tick = now;
            failure_led_on = 1U;
        }
        else if ((uint32_t)(now - last_change_tick) >= 500U)
        {
            last_change_tick = now;
            failure_led_on = (failure_led_on == 0U) ? 1U : 0U;
        }
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,
                          (failure_led_on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else  //测试未运行或正在运行时，LED熄灭
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    }
    previous_status = status;
#endif
}
