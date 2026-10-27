/**
 * @file sd_storage.c
 * @brief SD 卡文件操作封装；只管理一个文件及一个静态 4KiB 写缓存。
 *
 * 采集数据的结构、采样时间、CSV 字段和何时保存均由上层决定。
 * Add 两个接口只处理内存，其余明确的文件接口才可能访问 SD 卡。
 */
#include "sd_storage.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static FIL sd_file;
static uint8_t sd_memory[SD_CARD_MEMORY_CAPACITY_BYTES];
static uint32_t sd_pending = 0U;
static uint8_t sd_file_open = 0U;
static SD_Card_State sd_state = SD_CARD_STATE_NOT_PREPARED;
static SD_Card_File_Mode sd_mode = SD_CARD_FILE_MODE_NONE;

/* 使用最近已知快照，故障后的查询不再读取可能失效的 FatFs 对象。
 * 长度和位置不包含本模块尚未交给 FatFs 的 sd_pending 字节。 */
static uint32_t sd_file_size = 0U;
static uint32_t sd_file_position = 0U;
static SD_Card_Result sd_last_result = SD_CARD_OK;
static uint32_t sd_last_fatfs_result = 0U;
static uint8_t sd_last_fatfs_valid = 0U;

/* 底层 HAL 超时依赖中断持续运行。BASEPRI/FAULTMASK 也视为不安全，
 * 避免从临界区执行阻塞文件操作。检查 CPU 寄存器不会访问 SDIO。 */
static uint8_t SD_Context_Is_Allowed(void)
{
    return ((__get_IPSR() == 0U) && (__get_PRIMASK() == 0U) &&
            (__get_BASEPRI() == 0U) && (__get_FAULTMASK() == 0U)) ? 1U : 0U;
}

/* 故障锁存后，误调用其他接口也不能覆盖原始故障原因。 */
static SD_Card_Result SD_Record_Simple_Result(SD_Card_Result result)
{
    if (sd_state != SD_CARD_STATE_FAULT)
    {
        sd_last_result = result;
        sd_last_fatfs_result = 0U;
        sd_last_fatfs_valid = 0U;
    }
    return result;
}

static SD_Card_Result SD_Record_FatFs_Result(SD_Card_Result result,
                                            FRESULT raw_result, uint8_t fault)
{
    sd_last_result = result;
    sd_last_fatfs_result = (uint32_t)raw_result;
    sd_last_fatfs_valid = 1U;
    if (fault != 0U)
    {
        sd_state = SD_CARD_STATE_FAULT;
    }
    return result;
}

static SD_Card_Result SD_Check_Entry(void)
{
    if (SD_Context_Is_Allowed() == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_UNSAFE_CONTEXT);
    }
    if (sd_state == SD_CARD_STATE_FAULT)
    {
        return SD_CARD_NEED_RESET;
    }
    return SD_CARD_OK;
}

static SD_Card_Result SD_Require_Open_File(void)
{
    SD_Card_Result result = SD_Check_Entry();
    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state == SD_CARD_STATE_NOT_PREPARED)
    {
        return SD_Record_Simple_Result(SD_CARD_NOT_PREPARED);
    }
    if (sd_file_open == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_NO_FILE_OPEN);
    }
    return SD_CARD_OK;
}

static SD_Card_Result SD_Require_Write_File(void)
{
    SD_Card_Result result = SD_Require_Open_File();
    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state != SD_CARD_STATE_WRITING)
    {
        return SD_Record_Simple_Result(SD_CARD_WRONG_FILE_MODE);
    }
    return SD_CARD_OK;
}

/* 只在 FatFs 操作成功且对象仍然打开时调用；关闭/故障后不调用。 */
static void SD_Update_File_Snapshot(void)
{
    sd_file_size = (uint32_t)f_size(&sd_file);
    sd_file_position = (uint32_t)f_tell(&sd_file);
}

static void SD_Clear_File_Session(void)
{
    memset(&sd_file, 0, sizeof(sd_file));
    sd_file_open = 0U;
    sd_mode = SD_CARD_FILE_MODE_NONE;
    sd_file_size = 0U;
    sd_file_position = 0U;
    sd_pending = 0U;
}

/* 允许标准短文件名的可见 ASCII 字符；路径、盘符、空格以及 FatFs
 * 禁止的短文件名字符一律拒绝，不在封装中悄悄转换或截短文件名。 */
static uint8_t SD_Name_Character_Is_Allowed(unsigned char character)
{
    if ((character < 0x21U) || (character > 0x7EU))
    {
        return 0U;
    }
    switch (character)
    {
        case '"': case '*': case '+': case ',': case '/': case ':':
        case ';': case '<': case '=': case '>': case '?': case '[':
        case '\\': case ']': case '|':
            return 0U;
        default:
            return 1U;
    }
}

static uint8_t SD_File_Name_Is_Valid(const char *name)
{
    uint32_t base_length = 0U;
    uint32_t extension_length = 0U;
    uint32_t index = 0U;
    uint8_t has_dot = 0U;
    unsigned char character;

    if ((name == 0) || (name[0] == '\0'))
    {
        return 0U;
    }
    while (name[index] != '\0')
    {
        if (index >= 12U)
        {
            return 0U;
        }
        character = (unsigned char)name[index];
        if (character == (unsigned char)'.')
        {
            if ((has_dot != 0U) || (base_length == 0U))
            {
                return 0U;
            }
            has_dot = 1U;
        }
        else
        {
            if (SD_Name_Character_Is_Allowed(character) == 0U)
            {
                return 0U;
            }
            if (has_dot == 0U)
            {
                if (++base_length > 8U)
                {
                    return 0U;
                }
            }
            else if (++extension_length > 3U)
            {
                return 0U;
            }
        }
        ++index;
    }
    return ((base_length != 0U) &&
            ((has_dot == 0U) || (extension_length != 0U))) ? 1U : 0U;
}

/* 与原始 FRESULT 一起记录；NO_CARD 来自卡座实际检测，不把所有磁盘
 * 故障都解释为拔卡。Add 接口不使用本函数，保证它们完全不访问卡。 */
static SD_Card_Result SD_Translate_FatFs_Error(FRESULT result)
{
    if (result == FR_NO_FILE || result == FR_NO_PATH)
    {
        return SD_CARD_FILE_NOT_FOUND;
    }
    if (result == FR_EXIST)
    {
        return SD_CARD_FILE_ALREADY_EXISTS;
    }
    if (result == FR_INVALID_NAME)
    {
        return SD_CARD_INVALID_FILE_NAME;
    }
    if (result == FR_NO_FILESYSTEM)
    {
        return SD_CARD_UNSUPPORTED_FILE_SYSTEM;
    }
    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        return SD_CARD_NO_CARD;
    }
    return SD_CARD_FILE_SYSTEM_ERROR;
}

static SD_Card_Result SD_Open_File(const char *name, SD_Card_File_Mode mode)
{
    SD_Card_Result result = SD_Check_Entry();
    FRESULT raw_result;
    BYTE flags;
    char full_name[16]; /* "0:/" + 最长 12 字节短文件名 + NUL。 */
    uint32_t name_length;
    uint32_t end_position;
    uint8_t expected_error;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state == SD_CARD_STATE_NOT_PREPARED)
    {
        return SD_Record_Simple_Result(SD_CARD_NOT_PREPARED);
    }
    if (sd_file_open != 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_FILE_ALREADY_OPEN);
    }
    if (SD_File_Name_Is_Valid(name) == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_INVALID_FILE_NAME);
    }

    name_length = (uint32_t)strlen(name);
    memcpy(full_name, SDPath, 3U);
    memcpy(full_name + 3U, name, name_length + 1U);
    flags = (mode == SD_CARD_FILE_MODE_READ) ? FA_READ : FA_WRITE;
    if (mode == SD_CARD_FILE_MODE_CREATE_NEW)
    {
        flags |= FA_CREATE_NEW;
    }
    /* FA_OPEN_EXISTING 的数值为 0；追加不用 FA_OPEN_APPEND，以免自动建文件。 */
    raw_result = f_open(&sd_file, full_name, flags);
    if (raw_result != FR_OK)
    {
        expected_error = ((raw_result == FR_NO_FILE) || (raw_result == FR_NO_PATH) ||
                          (raw_result == FR_EXIST) || (raw_result == FR_INVALID_NAME)) ? 1U : 0U;
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result,
                                      (expected_error == 0U) ? 1U : 0U);
    }

    sd_file_open = 1U;
    sd_mode = mode;
    sd_pending = 0U;
    sd_state = (mode == SD_CARD_FILE_MODE_READ) ? SD_CARD_STATE_READING : SD_CARD_STATE_WRITING;
    SD_Update_File_Snapshot();

    if (mode == SD_CARD_FILE_MODE_APPEND_EXISTING)
    {
        end_position = sd_file_size;
        raw_result = f_lseek(&sd_file, (FSIZE_t)end_position);
        if (raw_result != FR_OK)
        {
            return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
        }
        SD_Update_File_Snapshot();
        if (sd_file_position != end_position)
        {
            return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, raw_result, 1U);
        }
    }
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/* 提前拒绝超过 FAT32 最大文件长度的接收，避免将明确的文件长度限制
 * 混淆为后续的满盘短写。用减法判断，避免 size+pending+length 溢出。 */
static SD_Card_Result SD_Check_Memory_Space(uint32_t length)
{
    uint32_t file_room = UINT32_MAX - sd_file_size;

    if ((length > SD_CARD_MEMORY_CAPACITY_BYTES) || (sd_pending > file_room) ||
        (length > (file_room - sd_pending)))
    {
        return SD_Record_Simple_Result(SD_CARD_DATA_TOO_LARGE);
    }
    if (length > (SD_CARD_MEMORY_CAPACITY_BYTES - sd_pending))
    {
        return SD_Record_Simple_Result(SD_CARD_MEMORY_BUFFER_FULL);
    }
    return SD_CARD_OK;
}

/**
 * @brief 准备 SD 卡并立即挂载文件系统，供后续新建、追加和读取文件使用。
 *
 * @return SD_CARD_OK 准备成功；SD_CARD_NO_CARD 无卡；SD_CARD_UNSUPPORTED_FILE_SYSTEM 卡格式不支持；其余结果表示注册、挂载或调用状态错误。
 * @note 先完成系统时钟和 GPIO 初始化，在同一前台执行流中调用。正常重复调用不会破坏已打开文件和缓存。
 *       首次准备失败且清理成功后可直接重试；已经进入故障状态时须先明确放弃旧会话。不会自动格式化卡。
 */
SD_Card_Result SD_Card_Prepare_For_Use(void)
{
    SD_Card_Result result = SD_Check_Entry();
    FRESULT raw_result;
    FRESULT cleanup_result;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state != SD_CARD_STATE_NOT_PREPARED)
    {
        return SD_Record_Simple_Result(SD_CARD_OK);
    }

    MX_FATFS_Init();
    if (retSD != 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_DRIVER_LINK_ERROR);
    }
    raw_result = f_mount(&SDFatFS, SDPath, 1U);
    if (raw_result != FR_OK)
    {
        result = SD_Translate_FatFs_Error(raw_result);
        /* 首次准备失败还没有文件或用户缓存，解除挂载后允许直接重试。
         * 若清理也失败，必须锁存清理错误，不能宣称卷已安全解除关联。 */
        cleanup_result = f_mount(0, SDPath, 0U);
        if (cleanup_result != FR_OK)
        {
            return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, cleanup_result, 1U);
        }
        return SD_Record_FatFs_Result(result, raw_result, 0U);
    }

    sd_state = SD_CARD_STATE_READY;
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 在文件关闭后卸载文件系统，结束本次 SD 卡使用。
 *
 * @return SD_CARD_OK 已卸载或原本未准备；SD_CARD_FILE_ALREADY_OPEN 当前文件尚未关闭；其他结果表示上下文、会话或卸载错误。
 * @note 先调用 SD_Card_Save_And_Close_File()。本函数不代替保存关闭，也不取消底层驱动注册；卸载失败会锁存故障。
 */
SD_Card_Result SD_Card_Finish_Using(void)
{
    SD_Card_Result result = SD_Check_Entry();
    FRESULT raw_result;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_file_open != 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_FILE_ALREADY_OPEN);
    }
    if (sd_state == SD_CARD_STATE_NOT_PREPARED)
    {
        return SD_Record_Simple_Result(SD_CARD_OK);
    }
    raw_result = f_mount(0, SDPath, 0U);
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    SD_Clear_File_Session();
    sd_state = SD_CARD_STATE_NOT_PREPARED;
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 创建并打开一个新文件，准备从文件开头写入数据，已有同名文件不会被覆盖。
 *
 * @param[in] name 以 NUL 结尾的根目录 ASCII 8.3 文件名，例如 RUN00001.CSV；不带盘符、目录或空格，扩展名可省略。
 *
 * @return SD_CARD_OK 新建成功；SD_CARD_FILE_ALREADY_EXISTS 文件已存在；SD_CARD_INVALID_FILE_NAME 名称为空或不合法；其余结果表示状态或文件系统错误。
 * @note 先准备 SD 卡，并关闭上一个文件。同一时刻只允许打开一个文件；需要表头时，创建成功后自行加入一行表头。
 */
SD_Card_Result SD_Card_Create_New_File(const char *name)
{
    return SD_Open_File(name, SD_CARD_FILE_MODE_CREATE_NEW);
}

/**
 * @brief 打开已有文件并移到文件末尾，供后续继续添加数据。
 *
 * @param[in] name 以 NUL 结尾的根目录 ASCII 8.3 文件名，例如 RUN00001.CSV；不带盘符、目录或空格，扩展名可省略。
 *
 * @return SD_CARD_OK 已打开并定位末尾；SD_CARD_FILE_NOT_FOUND 文件不存在；SD_CARD_INVALID_FILE_NAME 名称为空或不合法；其余结果表示状态或文件系统错误。
 * @note 先准备 SD 卡，并关闭上一个文件。文件不存在时不会自动新建；已有表头一般不需要再次加入。
 */
SD_Card_Result SD_Card_Open_File_To_Add_Data(const char *name)
{
    return SD_Open_File(name, SD_CARD_FILE_MODE_APPEND_EXISTING);
}

/**
 * @brief 以只读方式打开已有文件，从第 0 字节开始读取，供后续分块发送文件。
 *
 * @param[in] name 以 NUL 结尾的根目录 ASCII 8.3 文件名，例如 RUN00001.CSV；不带盘符、目录或空格，扩展名可省略。
 *
 * @return SD_CARD_OK 打开成功；SD_CARD_FILE_NOT_FOUND 文件不存在；SD_CARD_INVALID_FILE_NAME 名称为空或不合法；其余结果表示状态或文件系统错误。
 * @note 先准备 SD 卡，并保存关闭正在写入的文件。只读打开后不能向内存写缓存中添加数据。
 */
SD_Card_Result SD_Card_Open_File_To_Read_Data(const char *name)
{
    return SD_Open_File(name, SD_CARD_FILE_MODE_READ);
}

/**
 * @brief 将一批原始字节完整暂存到内存中，不访问 SD 卡，也不自动补换行。
 *
 * @param[in] data 待复制数据的首地址，至少包含 length 字节；length 为 0 时允许为 NULL。
 * @param[in] length 本次数据长度，单位字节，最大为 4096；必须能整体放入剩余缓存。
 *
 * @return SD_CARD_OK 整批已进入内存；SD_CARD_MEMORY_BUFFER_FULL 空间不足且整批未接收；SD_CARD_DATA_TOO_LARGE 输入或文件长度超限；其余结果表示参数或状态错误。
 * @note 仅供已打开的新建或追加文件使用；所有接口由同一前台串行调用，不能在中断内使用。
 *       成功不表示已经保存到卡。缓存满时先在前台批量写出，成功后再重新提交这批数据。
 */
SD_Card_Result SD_Card_Add_Data_To_Memory(const void *data, uint32_t length)
{
    SD_Card_Result result = SD_Require_Write_File();
    if (result != SD_CARD_OK)
    {
        return result;
    }
    if ((data == 0) && (length != 0U))
    {
        return SD_Record_Simple_Result(SD_CARD_INVALID_ARGUMENT);
    }
    result = SD_Check_Memory_Space(length);
    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (length != 0U)
    {
        memcpy(sd_memory + sd_pending, data, length);
        sd_pending += length;
    }
    return SD_Record_Simple_Result(SD_CARD_OK);
}

/**
 * @brief 将一整行 CSV 文本暂存到内存，并在末尾自动添加 CRLF 换行，不访问 SD 卡。
 *
 * @param[in] text 以 NUL 结尾的文本，正文最多 4094 字节且不含 CR 或 LF；空字符串会加入一个空行，结尾 NUL 不写入文件。
 *
 * @return SD_CARD_OK 整行及换行已进入内存；SD_CARD_MEMORY_BUFFER_FULL 空间不足且整行未接收；SD_CARD_DATA_TOO_LARGE 行或文件长度超限；其余结果表示参数或状态错误。
 * @note 仅在写文件模式下由前台调用。成功仅表示完整暂存；本函数不自动腾空间，不解析、转义 CSV 字段。
 *       磁盘写入不是事务，后续故障仍可能使文件只写出半行；缓存满时须先写出旧数据，再重试当前行。
 */
SD_Card_Result SD_Card_Add_One_Line_To_Memory(const char *text)
{
    SD_Card_Result result = SD_Require_Write_File();
    uint32_t length = 0U;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (text == 0)
    {
        return SD_Record_Simple_Result(SD_CARD_INVALID_ARGUMENT);
    }
    /* 有界扫描，不对可能超长的输入使用无界 strlen。 */
    while (text[length] != '\0')
    {
        if (length >= SD_CARD_LINE_MAX_TEXT_BYTES)
        {
            return SD_Record_Simple_Result(SD_CARD_DATA_TOO_LARGE);
        }
        if ((text[length] == '\r') || (text[length] == '\n'))
        {
            return SD_Record_Simple_Result(SD_CARD_INVALID_ARGUMENT);
        }
        ++length;
    }
    result = SD_Check_Memory_Space(length + 2U);
    if (result != SD_CARD_OK)
    {
        return result;
    }
    memcpy(sd_memory + sd_pending, text, length);
    sd_memory[sd_pending + length] = (uint8_t)'\r';
    sd_memory[sd_pending + length + 1U] = (uint8_t)'\n';
    sd_pending += length + 2U;
    return SD_Record_Simple_Result(SD_CARD_OK);
}

/**
 * @brief 把内存中暂存的数据批量写入当前文件，成功后清空本模块的待写缓存。
 *
 * @return SD_CARD_OK 已全部交给文件系统或原本无待写数据；SD_CARD_PARTIAL_WRITE 只处理了部分数据；其他结果表示状态或文件系统错误。
 * @note 仅在写文件模式下由前台调用；该函数同步等待写卡，不能在中断或关中断时调用。
 *       一次调用执行一次 f_write，不执行 f_sync；成功不表示所有文件信息已同步。
 *       部分写或错误时按实际处理长度移除前缀、保留剩余尾部并锁存故障，不得自动重放整批数据。
 */
SD_Card_Result SD_Card_Write_Memory_Data_To_File(void)
{
    SD_Card_Result result = SD_Require_Write_File();
    FRESULT raw_result;
    UINT written = 0U;
    uint32_t requested;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_pending == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_OK);
    }
    requested = sd_pending;
    raw_result = f_write(&sd_file, sd_memory, (UINT)requested, &written);
    if ((uint32_t)written > requested)
    {
        /* 防御异常回调，避免用错误的字节数造成缓冲区越界或计数下溢。 */
        return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, raw_result, 1U);
    }
    if (written != 0U)
    {
        sd_pending -= (uint32_t)written;
        memmove(sd_memory, sd_memory + written, sd_pending);
        /* 写模式始终位于末尾；错误返回时只按已报告的实际字节数推进
         * 最近已知快照，不再读取可能失效的 FIL。它不证明数据已持久化。 */
        sd_file_position += (uint32_t)written;
        if (sd_file_position > sd_file_size)
        {
            sd_file_size = sd_file_position;
        }
    }
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    SD_Update_File_Snapshot();
    if ((uint32_t)written != requested)
    {
        /* FatFs 的满盘短写可返回 FR_OK，必须另行比较实际字节数。 */
        return SD_Record_FatFs_Result(SD_CARD_PARTIAL_WRITE, raw_result, 1U);
    }
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 写出全部暂存数据，并同步文件数据及文件信息，完成一个明确的保存节点。
 *
 * @return SD_CARD_OK 写入及同步均成功；其他结果表示写入、同步、模式或调用状态错误，可查询当前状态取得诊断。
 * @note 仅在写文件模式下由前台调用；先执行批量写入，再执行 f_sync。任一步失败都会按原错误返回。
 *       同步成功表示当前文件系统和卡驱动报告完成，不是突然断电绝不丢数据的保证。
 */
SD_Card_Result SD_Card_Save_All_Data_To_Card(void)
{
    SD_Card_Result result = SD_Card_Write_Memory_Data_To_File();
    FRESULT raw_result;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    raw_result = f_sync(&sd_file);
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 保存并关闭当前写入文件；如果当前为只读文件，则直接关闭。
 *
 * @return SD_CARD_OK 文件已成功关闭；SD_CARD_NO_FILE_OPEN 尚无打开文件；其他结果表示保存、关闭或调用状态错误。
 * @note 写模式先保存暂存数据及文件信息，再关闭文件。任一步失败都会锁存故障，保留诊断，不假报关闭成功。
 *       成功关闭后才可打开下一个文件或结束 SD 卡使用；故障后只能查询或明确放弃旧会话。
 */
SD_Card_Result SD_Card_Save_And_Close_File(void)
{
    SD_Card_Result result = SD_Require_Open_File();
    FRESULT raw_result;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state == SD_CARD_STATE_WRITING)
    {
        result = SD_Card_Save_All_Data_To_Card();
        if (result != SD_CARD_OK)
        {
            return result;
        }
    }
    raw_result = f_close(&sd_file);
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    SD_Clear_File_Session();
    sd_state = SD_CARD_STATE_READY;
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 从当前读取位置获取一块原始文件数据，并把实际读取长度交给调用方。
 *
 * @param[out] buffer 接收缓冲区，至少能容纳 capacity 字节；capacity 为 0 时允许为 NULL，不自动补字符串结束符。
 * @param[in] capacity 本次最多读取的字节数，单位字节；0 表示不读取数据。
 * @param[out] count 实际读取的字节数，必须非 NULL；调用方只应处理这部分数据。
 *
 * @return SD_CARD_OK 读取成功，capacity 大于 0 且 *count 为 0 表示已到文件末尾；其他结果表示参数、模式或文件系统错误。
 * @note 只读打开文件后由前台调用。正常短读也表示已到文件末尾，不属于故障。
 *       出错时即使 count 非零，也不能把本次数据当作完整块；文件系统错误会锁存故障。
 */
SD_Card_Result SD_Card_Read_File_Data(void *buffer, uint32_t capacity, uint32_t *count)
{
    SD_Card_Result result = SD_Require_Open_File();
    FRESULT raw_result;
    UINT actual = 0U;

    if (count != 0)
    {
        *count = 0U;
    }
    if (result != SD_CARD_OK)
    {
        return result;
    }
    if ((count == 0) || ((buffer == 0) && (capacity != 0U)))
    {
        return SD_Record_Simple_Result(SD_CARD_INVALID_ARGUMENT);
    }
    if (sd_state != SD_CARD_STATE_READING)
    {
        return SD_Record_Simple_Result(SD_CARD_WRONG_FILE_MODE);
    }
    if (capacity == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_OK);
    }
    raw_result = f_read(&sd_file, buffer, (UINT)capacity, &actual);
    if (((uint32_t)actual > capacity) ||
        ((uint32_t)actual > (sd_file_size - sd_file_position)))
    {
        return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, raw_result, 1U);
    }
    *count = (uint32_t)actual;
    sd_file_position += (uint32_t)actual;
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    SD_Update_File_Snapshot();
    /* 实际读取 0 字节或短读是正常 EOF，不升级为故障，不向 buffer 补 NUL。 */
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 设置下一次读取的字节位置，供重新读取文件开头或某一段内容。
 *
 * @param[in] offset 相对文件开头的字节偏移，0 为文件开头，最大允许等于文件长度，即末尾位置。
 *
 * @return SD_CARD_OK 已定位；SD_CARD_INVALID_ARGUMENT 偏移超出文件长度；其余结果表示模式、定位或调用状态错误。
 * @note 仅允许只读文件，由前台调用，不会扩展文件。定位成功后也会检查实际位置，异常时锁存故障。
 */
SD_Card_Result SD_Card_Set_File_Read_Position(uint32_t offset)
{
    SD_Card_Result result = SD_Require_Open_File();
    FRESULT raw_result;

    if (result != SD_CARD_OK)
    {
        return result;
    }
    if (sd_state != SD_CARD_STATE_READING)
    {
        return SD_Record_Simple_Result(SD_CARD_WRONG_FILE_MODE);
    }
    if (offset > sd_file_size)
    {
        return SD_Record_Simple_Result(SD_CARD_INVALID_ARGUMENT);
    }
    raw_result = f_lseek(&sd_file, (FSIZE_t)offset);
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_Translate_FatFs_Error(raw_result), raw_result, 1U);
    }
    SD_Update_File_Snapshot();
    if (sd_file_position != offset)
    {
        return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, raw_result, 1U);
    }
    return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
}

/**
 * @brief 取得当前 SD 卡文件会话的内存快照，包括打开模式、文件长度、待写量和最近错误。
 *
 * @param[out] info 状态结构体地址，必须非 NULL；函数把各状态字段复制到此结构体中。
 *
 * @return SD_CARD_OK 快照复制成功；SD_CARD_INVALID_ARGUMENT 输出指针为空；SD_CARD_UNSAFE_CONTEXT 调用上下文不安全。
 * @note 本函数不访问 SD 卡，不改变原始错误，也可在会话故障后由前台查询。
 *       文件长度和位置不包含 pending_bytes，且不是已同步保存长度；file_information_valid 表示最近已知文件信息有效。
 *       只有 raw_fatfs_result_valid 为 1 时才能解释原始 FatFs 结果；故障快照不表示旧文件句柄还能使用。
 */
SD_Card_Result SD_Card_Get_Current_State(SD_Card_Info *info)
{
    /* 查询不能改变 last_result，即使调用方给错参数，也保留原始诊断。 */
    if (SD_Context_Is_Allowed() == 0U)
    {
        return SD_CARD_UNSAFE_CONTEXT;
    }
    if (info == 0)
    {
        return SD_CARD_INVALID_ARGUMENT;
    }
    info->state = sd_state;
    info->file_mode = sd_mode;
    info->file_size_bytes = sd_file_size;
    info->file_position_bytes = sd_file_position;
    info->pending_bytes = sd_pending;
    info->last_result = sd_last_result;
    info->raw_fatfs_result = sd_last_fatfs_result;
    info->raw_fatfs_result_valid = sd_last_fatfs_valid;
    info->file_information_valid = sd_file_open;
    return SD_CARD_OK;
}

/**
 * @brief 明确放弃未保存数据，清理文件会话及挂载关系，为重新准备 SD 卡做准备。
 *
 * @return SD_CARD_OK 已清理，可重新准备并打开文件；SD_CARD_UNSAFE_CONTEXT 调用上下文不安全；其他结果表示卸载清理失败并重新锁存故障。
 * @note 此操作会丢弃内存中尚未保存的数据，不尝试补写、同步、关闭或格式化；它不会复位单片机。
 *       调用方确认接受数据损失后在前台调用。成功后须重新 Prepare_For_Use，再明确打开文件，不会向新卡重放旧数据。
 */
SD_Card_Result SD_Card_Give_Up_Unsaved_Data_And_Reset(void)
{
    FRESULT raw_result = FR_OK;
    uint8_t called_fatfs = 0U;

    if (SD_Context_Is_Allowed() == 0U)
    {
        return SD_Record_Simple_Result(SD_CARD_UNSAFE_CONTEXT);
    }
    /* 不尝试 f_close/f_sync，避免向新插入的卡补写旧会话。
     * f_mount(NULL) 清除卷关联及文件锁表，不执行物理 SDIO 读写。 */
    if (retSD == 0U)
    {
        called_fatfs = 1U;
        raw_result = f_mount(0, SDPath, 0U);
    }
    SD_Clear_File_Session();
    memset(sd_memory, 0, sizeof(sd_memory));
    sd_state = SD_CARD_STATE_NOT_PREPARED;
    if (raw_result != FR_OK)
    {
        return SD_Record_FatFs_Result(SD_CARD_FILE_SYSTEM_ERROR, raw_result, 1U);
    }
    if (called_fatfs != 0U)
    {
        return SD_Record_FatFs_Result(SD_CARD_OK, raw_result, 0U);
    }
    return SD_Record_Simple_Result(SD_CARD_OK);
}
