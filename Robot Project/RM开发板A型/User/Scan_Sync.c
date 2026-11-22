#include "Scan_Sync.h"

/**
 * @file Scan_Sync.c
 * @brief 测距数据与里程、转角的同步
 *
 * 为什么不能直接用"收到数据的时刻"去配θ:
 *   433M模块是攒一批再发 数据到达主控的时刻比采样时刻晚 而且晚多少每次不一样
 *   测距舱300rpm时 1ms对应1.8度 到达时刻抖动10ms 点就偏18度
 *
 * 做法:
 *   L432KC的采样是定时器触发的 间隔严格相等 所以 采样时刻 = 序号 × 周期 + 常数
 *   每一帧 到达时刻 - 序号 × 周期 = 常数 + 本次延时  延时只会让它偏大
 *   所以跟踪这个量的最小值 就得到一条不受抖动影响的时间线
 *   具体是每1024个采样为一块 记下块内延时最小的那一帧 保留最近32块
 *   周期 = 最新一块与最老一块的这两帧 时间差除以序号差
 *   时间线 = 以该周期为斜率 从下方贴住最近4块这几帧的那条直线
 *   两块板的晶振有偏差(L432KC用的是内部RC 偏差可到1%) 所以周期也要在线估计
 *
 * 调用关系: Push_Pose在CAN接收中断 Rx_Data在USART6接收中断 Pop_Record在主循环
 * 主控上CAN和串口中断优先级相同 互相不会打断 所以前两者之间不需要加锁
 */

#define SCAN_PERIOD_NOMINAL_Q16 ((uint32_t)SCAN_SAMPLE_PERIOD_US << 16)
#define SCAN_PERIOD_LIMIT_Q16 (SCAN_PERIOD_NOMINAL_Q16 / 50U) //周期估计值限制在标称值±2%以内
#define SCAN_PERIOD_BLOCK 1024U    //每这么多个采样为一个统计块
#define SCAN_MIN_HISTORY 32U       //保留最近多少个统计块的最小延时帧 用于估计周期 跨度越长周期越准
#define SCAN_ANCHOR_BLOCKS 4U      //时间线只贴住最近这么多块 太多会把周期的残余误差放大
#define SCAN_MAX_SEQUENCE_GAP 2000U //序号跳变超过这个值视为测距板重启或长时间断流 重新开始估计
#define SCAN_MAX_EXTRAPOLATE_US 5000 //采样时刻比最新一条转角历史还新时 最多允许外推这么久

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

/* ---------------- 序号与时间线估计状态 ---------------- */

static uint8_t Timeline_Valid = 0;
static uint16_t Last_Sequence16 = 0;
static uint32_t Sequence32 = 0;

static uint32_t Ref_Sequence = 0; //时间线上的参考点: 序号
static uint32_t Ref_Time_us = 0;  //时间线上的参考点: 该序号对应的时刻
static uint32_t Ref_Frac_Q16 = 0; //参考点时刻的小数部分
static uint32_t Period_Q16 = SCAN_PERIOD_NOMINAL_Q16;

static uint32_t Block_Sequence0 = 0; //当前统计块的起点
static uint32_t Block_Time0 = 0;
static uint8_t Block_Has_Min = 0;
static int32_t Block_Min_Error = 0;
static uint32_t Block_Min_Sequence = 0;
static uint32_t Block_Min_Time = 0;
static uint32_t Min_Sequence[SCAN_MIN_HISTORY]; //最近几个统计块里各自延时最小的那一帧 下标0最老
static uint32_t Min_Time[SCAN_MIN_HISTORY];
static uint8_t Min_Count = 0;

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
    Timeline_Valid = 0;
    Period_Q16 = SCAN_PERIOD_NOMINAL_Q16;
    Min_Count = 0;
    Statistics.Frames_Ok = 0;
    Statistics.Samples_Ok = 0;
    Statistics.Checksum_Errors = 0;
    Statistics.Lost_Samples = 0;
    Statistics.No_Pose_Drops = 0;
    Statistics.Overflow_Drops = 0;
    Statistics.Period_Q16 = SCAN_PERIOD_NOMINAL_Q16;
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
 * @brief ADC原始值换算为传感器读数 单位微米 报警或超范围返回SCAN_RHO_INVALID
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
 * @brief 开始一个新的周期统计块
 */
static void Scan_Sync_Start_Block(uint32_t Sequence, uint32_t Arrival_Time_us)
{
    Block_Sequence0 = Sequence;
    Block_Time0 = Arrival_Time_us;
    Block_Has_Min = 0;
}

/**
 * @brief 由序号和到达时刻估计采样时刻
 * @return 估计的采样时刻(含固定延时 调用者再减SCAN_FIXED_LATENCY_US)
 */
static uint32_t Scan_Sync_Estimate_Time(uint32_t Sequence, uint32_t Arrival_Time_us)
{
    uint32_t delta_sequence;
    uint64_t advance;
    uint32_t predicted;
    int32_t error;
    int32_t block_error;

    if (!Timeline_Valid)
    {
        Timeline_Valid = 1;
        Ref_Sequence = Sequence;
        Ref_Time_us = Arrival_Time_us;
        Ref_Frac_Q16 = 0;
        Min_Count = 0;
        Scan_Sync_Start_Block(Sequence, Arrival_Time_us);
        return Arrival_Time_us;
    }

    //沿时间线从参考点推到当前序号
    delta_sequence = Sequence - Ref_Sequence;
    advance = (uint64_t)delta_sequence * Period_Q16 + Ref_Frac_Q16;
    predicted = Ref_Time_us + (uint32_t)(advance >> 16);

    //到达时刻不可能早于采样时刻 所以实际到达比预测早 说明时间线偏晚了 直接拉回来
    //实际到达比预测晚 是正常的传输延时 不动时间线
    error = (int32_t)(Arrival_Time_us - predicted);

    if (error < 0)
    {
        predicted = Arrival_Time_us;
        Ref_Frac_Q16 = 0;
    }
    else
    {
        Ref_Frac_Q16 = (uint32_t)(advance & 0xFFFFU);
    }

    Ref_Sequence = Sequence;
    Ref_Time_us = predicted;

    //记下本统计块里延时最小的一帧
    block_error = (int32_t)(Arrival_Time_us - Block_Time0) - (int32_t)(((uint64_t)(Sequence - Block_Sequence0) * Period_Q16) >> 16);

    if (!Block_Has_Min || block_error < Block_Min_Error)
    {
        Block_Has_Min = 1;
        Block_Min_Error = block_error;
        Block_Min_Sequence = Sequence;
        Block_Min_Time = Arrival_Time_us;
    }

    if (Sequence - Block_Sequence0 >= SCAN_PERIOD_BLOCK)
    {
        uint8_t k;
        uint32_t line;
        uint32_t lowest;

        //存入历史 满了就挤掉最老的
        if (Min_Count == SCAN_MIN_HISTORY)
        {
            for (k = 1; k < SCAN_MIN_HISTORY; k++)
            {
                Min_Sequence[k - 1] = Min_Sequence[k];
                Min_Time[k - 1] = Min_Time[k];
            }
            Min_Count--;
        }
        Min_Sequence[Min_Count] = Block_Min_Sequence;
        Min_Time[Min_Count] = Block_Min_Time;
        Min_Count++;

        //周期 = 最新与最老两帧的时间差除以序号差 跨度越长越准
        if (Min_Count >= 2U && (Min_Sequence[Min_Count - 1U] - Min_Sequence[0]) >= SCAN_PERIOD_BLOCK / 2U)
        {
            uint32_t measured = (uint32_t)((((uint64_t)(Min_Time[Min_Count - 1U] - Min_Time[0])) << 16) / (Min_Sequence[Min_Count - 1U] - Min_Sequence[0]));

            if (measured > SCAN_PERIOD_NOMINAL_Q16 + SCAN_PERIOD_LIMIT_Q16)
            {
                measured = SCAN_PERIOD_NOMINAL_Q16 + SCAN_PERIOD_LIMIT_Q16;
            }
            else if (measured < SCAN_PERIOD_NOMINAL_Q16 - SCAN_PERIOD_LIMIT_Q16)
            {
                measured = SCAN_PERIOD_NOMINAL_Q16 - SCAN_PERIOD_LIMIT_Q16;
            }

            Period_Q16 = measured;
            Statistics.Period_Q16 = Period_Q16;
        }

        //时间线: 以当前周期为斜率 把最近几块的最小延时帧都推到当前序号 取最早的那个 即从下方贴住它们
        lowest = Arrival_Time_us;
        for (k = (Min_Count > SCAN_ANCHOR_BLOCKS) ? (uint8_t)(Min_Count - SCAN_ANCHOR_BLOCKS) : 0; k < Min_Count; k++)
        {
            line = Min_Time[k] + (uint32_t)(((uint64_t)(Sequence - Min_Sequence[k]) * Period_Q16) >> 16);
            if ((int32_t)(line - lowest) < 0)
            {
                lowest = line;
            }
        }
        Ref_Time_us = lowest;
        Ref_Frac_Q16 = 0;

        Scan_Sync_Start_Block(Sequence, Arrival_Time_us);
    }

    return predicted;
}

/**
 * @brief 取出帧内第Index个采样的ADC原始值 每两个12位采样挤在3个字节里
 */
static uint16_t Scan_Sync_Unpack_Sample(const uint8_t *Frame, uint8_t Index)
{
    const uint8_t *group = &Frame[3 + (Index / 2U) * 3U];

    if ((Index & 1U) == 0U)
    {
        return (uint16_t)(group[0] | ((uint16_t)(group[1] & 0x0FU) << 8));
    }

    return (uint16_t)((group[1] >> 4) | ((uint16_t)group[2] << 4));
}

/**
 * @brief 处理一帧校验通过的测距数据 一帧里有SCAN_SAMPLES_PER_FRAME个采样
 */
static void Scan_Sync_Handle_Frame(const uint8_t *Frame, uint32_t Arrival_Time_us)
{
    uint16_t sequence16 = (uint16_t)(Frame[1] | ((uint16_t)Frame[2] << 8)); //本帧第一个采样的序号
    uint16_t step;
    uint32_t last_time;
    uint32_t period_q16;
    uint8_t index;

    Statistics.Frames_Ok++;
    Statistics.Samples_Ok += SCAN_SAMPLES_PER_FRAME;

    //16位序号扩展成32位 顺便统计丢点 正常情况下相邻两帧的序号正好差一帧的采样数
    if (Timeline_Valid)
    {
        step = (uint16_t)(sequence16 - Last_Sequence16);

        if (step == 0U)
        {
            return; //重复帧
        }

        if (step > SCAN_MAX_SEQUENCE_GAP)
        {
            Timeline_Valid = 0; //测距板重启或长时间断流 时间线作废 下面重新开始
            Sequence32 += SCAN_SAMPLES_PER_FRAME;
        }
        else
        {
            Sequence32 += step;

            if (step > SCAN_SAMPLES_PER_FRAME)
            {
                Statistics.Lost_Samples += (uint32_t)(step - SCAN_SAMPLES_PER_FRAME);
            }
        }
    }

    Last_Sequence16 = sequence16;

    //帧是在最后一个采样完成后才发出的 所以到达时刻对应的是最后一个采样
    //用最后一个采样的序号去更新时间线 再按周期往前倒推出帧内其余采样的时刻
    last_time = Scan_Sync_Estimate_Time(Sequence32 + SCAN_SAMPLES_PER_FRAME - 1U, Arrival_Time_us) - (uint32_t)SCAN_FIXED_LATENCY_US;
    period_q16 = Period_Q16;

    for (index = 0; index < SCAN_SAMPLES_PER_FRAME; index++)
    {
        uint16_t rho_um = Scan_Sync_Raw_To_Rho_um(Scan_Sync_Unpack_Sample(Frame, index));
        uint32_t samples_before_last = (uint32_t)(SCAN_SAMPLES_PER_FRAME - 1U - index);
        uint32_t sample_time = last_time - (uint32_t)(((uint64_t)samples_before_last * period_q16) >> 16);
        uint32_t write;
        Struct_Scan_Record *record;
        uint16_t theta;
        float x_m;

        if (index == SCAN_SAMPLES_PER_FRAME - 1U)
        {
            Last_Distance_mm = (rho_um == SCAN_RHO_INVALID) ? -1.0f : (float)rho_um / 1000.0f;
        }

        if (!Scan_Sync_Lookup_Pose(sample_time, &theta, &x_m))
        {
            Statistics.No_Pose_Drops++;
            continue;
        }

        write = Record_Write;

        if (write - Record_Read >= SCAN_RECORD_BUFFER_SIZE)
        {
            Statistics.Overflow_Drops++;
            continue;
        }

        record = &Record_Buffer[write & (SCAN_RECORD_BUFFER_SIZE - 1U)];
        record->Sequence = Sequence32 + index;
        record->Theta = theta;
        record->Rho_um = rho_um;
        record->X_m = x_m;

        Record_Write = write + 1U; //数据写完再移动写指针 主循环不会读到半条
    }
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
