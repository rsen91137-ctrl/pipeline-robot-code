/**
 * @file sd_card_test.h
 * @brief 第三阶段：不依赖真实传感器或电机的 SD 卡写入、追加与读取验证。
 * @note 本头文件不依赖 HAL，可由现有 C/C++ 主程序和中断源文件包含。
 */
#ifndef SD_CARD_TEST_H
#define SD_CARD_TEST_H

#include <stdint.h>
#include "sd_storage.h"

/* 工程可通过 -DSD_CARD_TEST_MODE=0 关闭测试；默认使用独立存储测试模式。
 * 主程序、GPIO 和中断文件也应使用同一宏隔离正常的机器人控制逻辑。 */
#ifndef SD_CARD_TEST_MODE
#define SD_CARD_TEST_MODE 1
#endif

#if (SD_CARD_TEST_MODE != 0) && (SD_CARD_TEST_MODE != 1)
#error SD_CARD_TEST_MODE_must_be_0_or_1
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SD_CARD_TEST_ROW_COUNT           20000U
#define SD_CARD_TEST_FIRST_HALF_ROWS     10000U
#define SD_CARD_TEST_SAVE_INTERVAL_ROWS  1000U
#define SD_CARD_TEST_MAX_FILE_NAMES      1000U
#define SD_CARD_TEST_READ_BLOCK_BYTES    512U
#define SD_CARD_TEST_SEEK_ROW_INDEX      12345U
#define SD_CARD_TEST_END_OF_STREAM       0x100U

typedef enum
{
    SD_CARD_TEST_NOT_RUN = 0,
    SD_CARD_TEST_RUNNING,
    SD_CARD_TEST_PASSED,
    SD_CARD_TEST_FAILED
} SD_Card_Test_Status;

/** 失败后保留当时阶段，不把阶段改成 DONE，以便在 Keil 中定位失败点。 */
typedef enum
{
    SD_CARD_TEST_STAGE_IDLE = 0,
    SD_CARD_TEST_STAGE_PREPARE,
    SD_CARD_TEST_STAGE_FIND_NEW_FILE,
    SD_CARD_TEST_STAGE_WRITE_FIRST_HALF,
    SD_CARD_TEST_STAGE_DRAIN_FIRST_HALF,
    SD_CARD_TEST_STAGE_SAVE_FIRST_HALF,
    SD_CARD_TEST_STAGE_CLOSE_FIRST_HALF,
    SD_CARD_TEST_STAGE_OPEN_APPEND,
    SD_CARD_TEST_STAGE_WRITE_SECOND_HALF,
    SD_CARD_TEST_STAGE_DRAIN_SECOND_HALF,
    SD_CARD_TEST_STAGE_SAVE_SECOND_HALF,
    SD_CARD_TEST_STAGE_CLOSE_APPEND,
    SD_CARD_TEST_STAGE_OPEN_READ_FIRST,
    SD_CARD_TEST_STAGE_SEEK_FIRST,
    SD_CARD_TEST_STAGE_VERIFY_FIRST,
    SD_CARD_TEST_STAGE_CLOSE_READ_FIRST,
    SD_CARD_TEST_STAGE_UNMOUNT_BEFORE_SECOND,
    SD_CARD_TEST_STAGE_REMOUNT,
    SD_CARD_TEST_STAGE_OPEN_READ_SECOND,
    SD_CARD_TEST_STAGE_SEEK_SECOND,
    SD_CARD_TEST_STAGE_VERIFY_SECOND,
    SD_CARD_TEST_STAGE_CLOSE_READ_SECOND,
    SD_CARD_TEST_STAGE_FINISH,
    SD_CARD_TEST_STAGE_DONE
} SD_Card_Test_Stage;

/** 测试本身的错误与存储接口返回值分开；例如内容不符时存储读取仍可能成功。 */
typedef enum
{
    SD_CARD_TEST_ERROR_NONE = 0,
    SD_CARD_TEST_ERROR_STORAGE_CALL,
    SD_CARD_TEST_ERROR_FILE_NAME_FORMAT,
    SD_CARD_TEST_ERROR_NO_FREE_FILE_NAME,
    SD_CARD_TEST_ERROR_CSV_FORMAT,
    SD_CARD_TEST_ERROR_BUFFER_RETRY,
    SD_CARD_TEST_ERROR_READ_COUNT,
    SD_CARD_TEST_ERROR_DATA_MISMATCH,
    SD_CARD_TEST_ERROR_UNEXPECTED_EOF,
    SD_CARD_TEST_ERROR_EXTRA_DATA,
    SD_CARD_TEST_ERROR_VERIFIED_LENGTH
} SD_Card_Test_Error;

/**
 * Keil 可直接观察 g_SD_Card_Test_Result。
 * - rows_written：已整行接收到存储模块的记录数，不含表头；不能单独证明已保存。
 * - bytes_written_expected：已接收的预期文件字节数，含表头和所有 CRLF。
 * - bytes_verified：当前完整读取遍次已经逐字节匹配的数量；第二遍开始会清零，
 *   成功结束时等于 bytes_written_expected；定位检查不计入该数值。
 * - first_mismatch_expected/actual：0..255 为字节，0x100 表示该侧已到流结束，
 *   从而区分错字节、文件缺失尾部和额外尾部。offset 是从文件开头起的零基偏移。
 * - write_elapsed_ms：成功创建后开始写表头，到追加部分最终关闭完成的耗时；
 *   不包含找空文件名、准备卡或任何读回校验；失败时保留已经过的写入时间。
 * - max_write_pause_ms：写入阶段单次写缓存、保存、关闭或打开追加文件的最大耗时。
 * - throughput_valid：完整写入已完成且计时非零；否则平均速度和达标标志不可解释。
 * - avg_meets_667hz：以每条 1500us 的平均预算比较，不代表每次写卡均小于 1.5ms。
 *   虚拟 time_us 只是文件字段，测试没有按 1.5ms 间隔真实采样或调度。
 * - storage_info：最后一次更新的存储状态快照；故障时不自动复位或补写。
 */
typedef struct
{
    SD_Card_Test_Status status;
    SD_Card_Test_Stage stage;
    SD_Card_Test_Error test_error;
    SD_Card_Result last_result;
    SD_Card_Info storage_info;
    uint8_t storage_info_valid;
    char filename[13];
    uint32_t rows_written;
    uint32_t bytes_written_expected;
    uint32_t bytes_verified;
    uint32_t write_elapsed_ms;
    uint32_t max_write_pause_ms;
    uint32_t rows_per_second;
    uint32_t seek_test_offset;
    uint32_t first_mismatch_offset;
    uint16_t first_mismatch_expected;
    uint16_t first_mismatch_actual;
    uint8_t first_mismatch_valid;
    uint8_t throughput_valid;
    uint8_t avg_meets_667hz;
    uint8_t current_read_pass;
    uint8_t completed_read_passes;
    uint8_t seek_check_passed;
    uint8_t eof_verified;
} SD_Card_Test_Report;

extern volatile SD_Card_Test_Report g_SD_Card_Test_Result;

/* 函数说明集中在 .c 中对应函数前。测试只运行一次，不格式化、删除或覆盖旧文件。 */
void SD_Card_Test_Run_Storage_And_Read_Back(void);
void SD_Card_Test_Show_Result(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_TEST_H */
