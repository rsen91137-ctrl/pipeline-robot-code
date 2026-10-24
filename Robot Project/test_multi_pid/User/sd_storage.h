/**
 * @file sd_storage.h
 * @brief 单张 SD 卡、单个文件的存储接口；隐藏 FatFs 文件对象。
 *
 * 使用顺序：准备卡 -> 新建/追加/读取文件 -> 操作文件 -> 保存并关闭 -> 结束使用。
 * 本模块没有动态内存、后台任务、自动保存周期或中断队列。
 * 所有接口只供同一个前台执行流串行调用；不能在中断、关中断或临界区内调用。
 * 1.5ms 采样只是未来性能目标，本模块不保证每条数据同步落盘。
 */
#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_CARD_MEMORY_CAPACITY_BYTES 4096U
#define SD_CARD_LINE_MAX_TEXT_BYTES   (SD_CARD_MEMORY_CAPACITY_BYTES - 2U)

/** 包装层结果；与 FatFs 的 FRESULT 数值没有对应关系。 */
typedef enum
{
    SD_CARD_OK = 0,
    SD_CARD_INVALID_ARGUMENT,          /**< 数据/输出空指针、无效偏移或行内含 CR/LF。 */
    SD_CARD_INVALID_FILE_NAME,         /**< 不是根目录 ASCII 8.3 文件名。 */
    SD_CARD_UNSAFE_CONTEXT,            /**< 在中断、关中断或屏蔽中断的临界区调用。 */
    SD_CARD_NOT_PREPARED,              /**< 还没有成功准备 SD 卡。 */
    SD_CARD_FILE_ALREADY_OPEN,         /**< 已有文件，必须先关闭。 */
    SD_CARD_NO_FILE_OPEN,              /**< 尚未打开文件。 */
    SD_CARD_WRONG_FILE_MODE,           /**< 读写方向不符合当前打开模式。 */
    SD_CARD_DATA_TOO_LARGE,            /**< 单次超过 4KiB、行超长或超过 FAT32 文件上限。 */
    SD_CARD_MEMORY_BUFFER_FULL,        /**< 内存剩余空间不足；此次数据完全未接收。 */
    SD_CARD_FILE_NOT_FOUND,            /**< 读取/追加要求文件原本存在。 */
    SD_CARD_FILE_ALREADY_EXISTS,       /**< 新建时重名，原文件没有被覆盖。 */
    SD_CARD_NO_CARD,                   /**< 没有检测到 SD 卡。 */
    SD_CARD_UNSUPPORTED_FILE_SYSTEM,   /**< 未找到支持的 FAT 文件系统；不会自动格式化。 */
    SD_CARD_DRIVER_LINK_ERROR,         /**< 第一阶段的磁盘驱动注册没有成功。 */
    SD_CARD_PARTIAL_WRITE,             /**< FatFs 无错误但未接收全部字节，可能空间不足。 */
    SD_CARD_FILE_SYSTEM_ERROR,         /**< 真实文件系统/读写/同步错误；查看原始结果。 */
    SD_CARD_NEED_RESET                 /**< 会话已经故障，须明确放弃未保存数据并复位。 */
} SD_Card_Result;

typedef enum
{
    SD_CARD_STATE_NOT_PREPARED = 0,
    SD_CARD_STATE_READY,               /**< 卡已准备，当前没有打开文件。 */
    SD_CARD_STATE_WRITING,
    SD_CARD_STATE_READING,
    SD_CARD_STATE_FAULT                /**< 故障锁存，禁止继续访问原文件。 */
} SD_Card_State;

typedef enum
{
    SD_CARD_FILE_MODE_NONE = 0,
    SD_CARD_FILE_MODE_CREATE_NEW,
    SD_CARD_FILE_MODE_APPEND_EXISTING,
    SD_CARD_FILE_MODE_READ
} SD_Card_File_Mode;

/**
 * 纯内存状态快照；Get_Current_State 不访问 SD 卡，也不修改既有错误诊断。
 * file_information_valid=1 表示存在打开文件的最近已知快照；故障时仍可用于诊断，
 * 并不表示原句柄还能继续使用。size/position 均不包含仍在本模块缓存的数据。
 * file_size_bytes 不是“已安全保存的长度”，FatFs 自身仍可能有未同步的元数据。
 * raw_fatfs_result_valid=0 时不要解释 raw_fatfs_result；正常纯内存操作没有 FRESULT。
 */
typedef struct
{
    SD_Card_State state;
    SD_Card_File_Mode file_mode;
    uint32_t file_size_bytes;
    uint32_t file_position_bytes;
    uint32_t pending_bytes;
    SD_Card_Result last_result;
    uint32_t raw_fatfs_result;
    uint8_t raw_fatfs_result_valid;
    uint8_t file_information_valid;
} SD_Card_Info;

//SD_Card_Result是枚举类型变量，用于表示SD卡操作的结果
SD_Card_Result SD_Card_Prepare_For_Use(void);
SD_Card_Result SD_Card_Finish_Using(void);
SD_Card_Result SD_Card_Create_New_File(const char *name);
SD_Card_Result SD_Card_Open_File_To_Add_Data(const char *name);
SD_Card_Result SD_Card_Open_File_To_Read_Data(const char *name);
SD_Card_Result SD_Card_Add_Data_To_Memory(const void *data, uint32_t length);
SD_Card_Result SD_Card_Add_One_Line_To_Memory(const char *text);
SD_Card_Result SD_Card_Write_Memory_Data_To_File(void);
SD_Card_Result SD_Card_Save_All_Data_To_Card(void);
SD_Card_Result SD_Card_Save_And_Close_File(void);
SD_Card_Result SD_Card_Read_File_Data(void *buffer, uint32_t capacity, uint32_t *count);
SD_Card_Result SD_Card_Set_File_Read_Position(uint32_t offset);
SD_Card_Result SD_Card_Get_Current_State(SD_Card_Info *info);
SD_Card_Result SD_Card_Give_Up_Unsaved_Data_And_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_STORAGE_H */
