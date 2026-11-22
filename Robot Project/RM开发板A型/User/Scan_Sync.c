#include "Scan_Sync.h"

/**
 * @file Scan_Sync.c
 * @brief 测距数据与里程、转角的同步(有线连接版)
 *
 * 测距舱300rpm时 1ms对应1.8度 所以ρ和θ必须按同一时刻配对
 * 有线连接下 数据从采样到进入主控的延时是固定的(约0.7ms) 所以:
 *   采样时刻 = 到达时刻 - 固定延时
 * 再拿这个时刻去转角里程历史里插值 就得到采样那一刻的θ和x
 *
 * 调用关系: Push_Pose在CAN接收中断 Rx_Data在USART6接收中断 Pop_Record在主循环
 * 主控上CAN和串口中断优先级相同 互相不会打断 所以前两者之间不需要加锁
 */

/* ---------------- 转角里程历史 ---------------- */

typedef struct
{
    uint32_t Time_us;
    uint16_t Encoder;
    float X_m;
} Struct_Scan_Pose;

static Struct_Scan_Pose Pose_History[SCAN_POSE_HISTORY_SIZE];
static uint32_t Pose_Count = 0; //累计写入条数 最新一条的下标是 (Pose_Count - 1) & (SIZE - 1)

/* ---------------- 测点缓冲区 中断写 主循环读 ---------------- */

static Struct_Scan_Record Record_Buffer[SCAN_RECORD_BUFFER_SIZE];
static volatile uint32_t Record_Write = 0; //只有中断改
static volatile uint32_t Record_Read = 0;  //只有主循环改

/* ---------------- 帧解析状态 ---------------- */

static uint8_t Frame_Buffer[SCAN_FRAME_LENGTH];
static uint8_t Frame_Count = 0;

/* ---------------- 序号 ---------------- */

static uint8_t Sequence_Valid = 0; //是否已经收到过第一帧
static uint16_t Last_Sequence16 = 0;
static uint32_t Sequence32 = 0;

static Struct_Scan_Statistics Statistics;
static volatile float Last_Distance_mm = -1.0f;

/**
 * @brief 初始化 清空所有状态
 */
void Scan_Sync_Init(void)
{
    Pose_Count = 0;
    Record_Write = 0;
    Record_Read = 0;
    Frame_Count = 0;
    Sequence_Valid = 0;
    Sequence32 = 0;
    Statistics.Frames_Ok = 0;
    Statistics.Checksum_Errors = 0;
    Statistics.Lost_Samples = 0;
    Statistics.No_Pose_Drops = 0;
    Statistics.Overflow_Drops = 0;
    Last_Distance_mm = -1.0f;
}

/**
 * @brief 帧校验 CRC8 多项式0x07 初值0xFF 覆盖除最后一个字节外的整帧
 *        帧变长以后 简单求和发现不了两个字节互相抵消的错误 所以换成CRC
 */
uint8_t Scan_Sync_Frame_Checksum(const uint8_t *Frame)
{
    uint8_t crc = 0xFF;
    uint8_t i;
    uint8_t bit;

    for (i = 0; i < SCAN_FRAME_LENGTH - 1; i++)
    {
        crc ^= Frame[i];

        for (bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

/**
 * @brief ADC原始值换算为传感器读数 单位微米 报警、超范围或未安装返回SCAN_RHO_INVALID
 */
uint16_t Scan_Sync_Raw_To_Rho_um(uint16_t Raw)
{
    uint32_t drop;

    if (Raw > SCAN_RAW_ALARM)
    {
        return SCAN_RHO_INVALID;
    }

    drop = ((uint32_t)Raw * SCAN_RHO_SLOPE_NUM + SCAN_RHO_SLOPE_DEN / 2U) / SCAN_RHO_SLOPE_DEN;

    return (uint16_t)(SCAN_RHO_ZERO_VOLT_UM - drop);
}

/**
 * @brief 存一条转角里程历史
 */
void Scan_Sync_Push_Pose(uint32_t Time_us, uint16_t Encoder, float X_m)
{
    Struct_Scan_Pose *pose = &Pose_History[Pose_Count & (SCAN_POSE_HISTORY_SIZE - 1U)];

    pose->Time_us = Time_us;
    pose->Encoder = (uint16_t)(Encoder & 8191U);
    pose->X_m = X_m;

    Pose_Count++;
}

/**
 * @brief 查出某一时刻的转角和里程 在相邻两条历史之间线性插值
 * @return 1成功 0失败(历史不够 或该时刻已经超出历史范围)
 */
static uint8_t Scan_Sync_Lookup_Pose(uint32_t Time_us, uint16_t *Theta, float *X_m)
{
    uint32_t available = (Pose_Count < SCAN_POSE_HISTORY_SIZE) ? Pose_Count : SCAN_POSE_HISTORY_SIZE;
    uint32_t back;
    const Struct_Scan_Pose *older;
    const Struct_Scan_Pose *newer;
    int32_t interval;
    int32_t elapsed;
    int32_t delta_encoder;
    int32_t theta;

    if (available < 2U)
    {
        return 0;
    }

    //从最新一条往回找 找到第一条不晚于目标时刻的历史
    for (back = 1U; back <= available; back++)
    {
        older = &Pose_History[(Pose_Count - back) & (SCAN_POSE_HISTORY_SIZE - 1U)];

        if ((int32_t)(Time_us - older->Time_us) >= 0)
        {
            break;
        }
    }

    if (back > available)
    {
        return 0; //目标时刻比最老的历史还早 延时超出了历史覆盖范围
    }

    if (back == 1U)
    {
        //目标时刻比最新一条还新 用最新两条外推 外推太久说明GM6020反馈断了
        if ((int32_t)(Time_us - older->Time_us) > SCAN_MAX_EXTRAPOLATE_US)
        {
            return 0;
        }
        newer = older;
        older = &Pose_History[(Pose_Count - 2U) & (SCAN_POSE_HISTORY_SIZE - 1U)];
    }
    else
    {
        newer = &Pose_History[(Pose_Count - back + 1U) & (SCAN_POSE_HISTORY_SIZE - 1U)];
    }

    interval = (int32_t)(newer->Time_us - older->Time_us);
    elapsed = (int32_t)(Time_us - older->Time_us);

    if (interval <= 0 || interval > 20000)
    {
        return 0; //两条历史间隔异常 中间丢了很多反馈 不可信
    }

    //编码器0~8191循环 取最短方向的差值
    delta_encoder = (int32_t)(((uint32_t)newer->Encoder - (uint32_t)older->Encoder + 4096U) & 8191U) - 4096;

    //乘8把8192刻度放大到65536刻度 插值后的小数部分得以保留
    theta = (int32_t)older->Encoder * 8 + delta_encoder * 8 * elapsed / interval;

    *Theta = (uint16_t)((uint32_t)theta & 0xFFFFU);
    *X_m = older->X_m + (newer->X_m - older->X_m) * (float)elapsed / (float)interval;

    return 1;
}

/**
 * @brief 处理一帧校验通过的测距数据
 */
static void Scan_Sync_Handle_Frame(const uint8_t *Frame, uint32_t Arrival_Time_us)
{
    uint16_t sequence16 = (uint16_t)(Frame[1] | ((uint16_t)Frame[2] << 8));
    uint16_t raw1 = (uint16_t)(Frame[3] | ((uint16_t)(Frame[4] & 0x0FU) << 8));
    uint16_t raw2 = (uint16_t)((Frame[4] >> 4) | ((uint16_t)Frame[5] << 4));
    uint16_t rho1_um = Scan_Sync_Raw_To_Rho_um(raw1);
    uint16_t rho2_um = Scan_Sync_Raw_To_Rho_um(raw2);
    uint16_t step;
    uint32_t sample_time;
    uint32_t write;
    Struct_Scan_Record *record;
    uint16_t theta;
    float x_m;

    Statistics.Frames_Ok++;
    Last_Distance_mm = (rho1_um == SCAN_RHO_INVALID) ? -1.0f : (float)rho1_um / 1000.0f;

    //16位序号扩展成32位 顺便统计丢点 正常情况下相邻两帧的序号正好差1
    if (Sequence_Valid)
    {
        step = (uint16_t)(sequence16 - Last_Sequence16);

        if (step == 0U)
        {
            return; //重复帧
        }

        if (step > SCAN_MAX_SEQUENCE_GAP)
        {
            Sequence32 += 1U; //测距板重启过 序号重新开始 不算丢点
        }
        else
        {
            Sequence32 += step;
            Statistics.Lost_Samples += (uint32_t)(step - 1U);
        }
    }

    Sequence_Valid = 1;
    Last_Sequence16 = sequence16;

    //有线连接延时固定 到达时刻减去固定延时就是采样时刻
    sample_time = Arrival_Time_us - (uint32_t)SCAN_FIXED_LATENCY_US;

    if (!Scan_Sync_Lookup_Pose(sample_time, &theta, &x_m))
    {
        Statistics.No_Pose_Drops++;
        return;
    }

    write = Record_Write;

    if (write - Record_Read >= SCAN_RECORD_BUFFER_SIZE)
    {
        Statistics.Overflow_Drops++;
        return;
    }

    record = &Record_Buffer[write & (SCAN_RECORD_BUFFER_SIZE - 1U)];
    record->Sequence = Sequence32;
    record->Theta = theta;
    record->Rho1_um = rho1_um;
    record->Rho2_um = rho2_um;
    record->Reserved = 0;
    record->X_m = x_m;

    Record_Write = write + 1U; //数据写完再移动写指针 主循环不会读到半条
}

/**
 * @brief 处理USART6收到的一批字节 逐字节找帧头并校验 允许一帧被拆在两批里
 */
void Scan_Sync_Rx_Data(const uint8_t *Buffer, uint16_t Length, uint32_t Arrival_Time_us)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        Frame_Buffer[Frame_Count++] = Buffer[i];

        if (Frame_Buffer[0] != SCAN_FRAME_HEADER)
        {
            Frame_Count = 0; //还没对上帧头 丢掉这个字节
            continue;
        }

        if (Frame_Count < SCAN_FRAME_LENGTH)
        {
            continue;
        }

        if (Scan_Sync_Frame_Checksum(Frame_Buffer) == Frame_Buffer[SCAN_FRAME_LENGTH - 1])
        {
            //这一帧后面还有多少字节 它就至少早到了多少个字节时间
            uint32_t bytes_after = (uint32_t)(Length - 1U - i);

            Scan_Sync_Handle_Frame(Frame_Buffer, Arrival_Time_us - bytes_after * SCAN_UART_BYTE_TIME_US);
            Frame_Count = 0;
        }
        else
        {
            //校验错 可能是把数据里的0xA5当成了帧头 往后挪到下一个0xA5重新对齐
            uint8_t shift;
            uint8_t k;

            Statistics.Checksum_Errors++;

            for (shift = 1; shift < SCAN_FRAME_LENGTH; shift++)
            {
                if (Frame_Buffer[shift] == SCAN_FRAME_HEADER)
                {
                    break;
                }
            }

            for (k = shift; k < SCAN_FRAME_LENGTH; k++)
            {
                Frame_Buffer[k - shift] = Frame_Buffer[k];
            }

            Frame_Count = (uint8_t)(SCAN_FRAME_LENGTH - shift);
        }
    }
}

/**
 * @brief 取出一条测点
 */
uint8_t Scan_Sync_Pop_Record(Struct_Scan_Record *Record)
{
    uint32_t read = Record_Read;

    if (read == Record_Write)
    {
        return 0;
    }

    *Record = Record_Buffer[read & (SCAN_RECORD_BUFFER_SIZE - 1U)];
    Record_Read = read + 1U;

    return 1;
}

float Scan_Sync_Get_Last_Distance_mm(void)
{
    return Last_Distance_mm;
}

void Scan_Sync_Get_Statistics(Struct_Scan_Statistics *Statistics_Out)
{
    *Statistics_Out = Statistics;
}
