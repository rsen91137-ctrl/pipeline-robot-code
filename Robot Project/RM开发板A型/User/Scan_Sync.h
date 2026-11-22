#ifndef __Scan_Sync_h__
#define __Scan_Sync_h__

/**
 * @file Scan_Sync.h
 * @brief 测距数据与里程、转角的同步 把(ρ, θ, x)配成一条完整测点
 *
 * 数据流:
 *   L432KC 每次ADC采样 -> 6字节帧(带采样序号) -> 433M -> 主控USART6
 *   主控 GM6020每次CAN反馈 -> 存一条(时间, 编码器, 里程)历史
 *   主控 收到测距帧 -> 用序号还原采样时刻 -> 到历史里插值出当时的θ和x -> 写入测点缓冲区
 *
 * 本文件不依赖HAL 可以在电脑上单独编译测试
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- L432KC -> 主控 的帧格式 两块板必须一致 ----------------
 * 字节0     帧头 0xA5
 * 字节1~2   采样序号 uint16 小端 每次采样加1 溢出回0
 * 字节3~4   ADC原始值 uint16 小端 (12位)
 * 字节5     校验 = 前5字节求和后按位取反
 */
#define SCAN_FRAME_HEADER 0xA5
#define SCAN_FRAME_LENGTH 6

#define SCAN_SAMPLE_PERIOD_US 1000 //L432KC采样周期标称值 TIM2为1000Hz
#define SCAN_UART_BYTE_TIME_US 87  //USART6一个字节的传输时间 115200波特率 10位
#define SCAN_FIXED_LATENCY_US 0    //采样到主控收到的固定延时 需台架标定 标定前为0 只会让点云整体绕轴转一个固定角度

/* ADC原始值 -> 传感器读数 与原L432KC代码的换算一致
 * 引脚电压 = raw * 3.3 / 4095   传感器电压 = 引脚电压 * 30 / 18   距离mm = 65 - 6 * 传感器电压
 * 合并后: 距离um = 65000 - raw * 33000 / 4095 */
#define SCAN_RHO_ZERO_VOLT_UM 65000U
#define SCAN_RHO_SLOPE_NUM 33000U
#define SCAN_RHO_SLOPE_DEN 4095U
#define SCAN_RAW_ALARM 3797U //传感器电压超过5.1V视为报警(超量程) 对应的ADC原始值
#define SCAN_RHO_INVALID 0U  //Rho_um为0表示该点无效

#define SCAN_POSE_HISTORY_SIZE 256   //转角里程历史条数 必须是2的幂 1kHz反馈时覆盖256ms
#define SCAN_RECORD_BUFFER_SIZE 1024 //测点缓冲区条数 必须是2的幂

/**
 * @brief 一条完整测点 共12字节 小端
 * 三维坐标换算: (X_m, ρcosθ, ρsinθ)  其中 ρ = Rho_um / 1e6 + 安装偏置  θ = Theta / 65536 * 2π
 */
typedef struct
{
    uint32_t Sequence; //采样序号 已扩展为32位 不连续说明中间丢了点
    uint16_t Theta;    //GM6020转角 0~65535对应一整圈
    uint16_t Rho_um;   //传感器读数 单位微米 不含安装偏置 0表示无效
    float X_m;         //轴向里程 单位米
} Struct_Scan_Record;

/**
 * @brief 运行统计 用来判断链路质量
 */
typedef struct
{
    uint32_t Frames_Ok;       //校验通过的帧数
    uint32_t Checksum_Errors; //帧头对上但校验错的次数
    uint32_t Lost_Samples;    //由序号跳变算出的丢点数
    uint32_t No_Pose_Drops;   //找不到对应时刻的转角里程而丢弃的点数
    uint32_t Overflow_Drops;  //测点缓冲区满而丢弃的点数
    uint32_t Period_Q16;      //估计出的采样周期 单位微秒 Q16定点 除以65536得到微秒
} Struct_Scan_Statistics;

void Scan_Sync_Init(void);

//GM6020每次CAN反馈时调用 存一条转角里程历史  Encoder为0~8191的原始编码器值
void Scan_Sync_Push_Pose(uint32_t Time_us, uint16_t Encoder, float X_m);

//USART6收到数据时调用  Arrival_Time_us为这批数据最后一个字节到达的时刻
void Scan_Sync_Rx_Data(const uint8_t *Buffer, uint16_t Length, uint32_t Arrival_Time_us);

//主循环调用 取出一条测点 返回1表示取到 0表示缓冲区已空
uint8_t Scan_Sync_Pop_Record(Struct_Scan_Record *Record);

//最近一次有效的传感器读数 单位mm 无效时为-1
float Scan_Sync_Get_Last_Distance_mm(void);

void Scan_Sync_Get_Statistics(Struct_Scan_Statistics *Statistics);

//工具函数
uint16_t Scan_Sync_Raw_To_Rho_um(uint16_t Raw);
uint8_t Scan_Sync_Frame_Checksum(const uint8_t *Frame);

#ifdef __cplusplus
}
#endif

#endif
