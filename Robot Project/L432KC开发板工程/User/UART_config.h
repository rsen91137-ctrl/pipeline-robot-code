#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32l4xx_hal.h"  
#include "usart.h"
#define BUFFER_SIZE 256
#define COMMAND_MAX_SIZE 100

/* 发给主控的测距帧 必须与主控工程 Scan_Sync.h 里的定义一致
 * 有线连接(经滑环)后 串口115200可以持续跑满 约11500字节/秒 所以一个采样发一帧:
 *   每帧7字节 1000Hz时合7000字节/秒
 *
 * 字节0     帧头 0xA5
 * 字节1~2   采样序号 uint16 小端 每次采样加1 溢出回0
 * 字节3~5   两个传感器的ADC原始值 各12位 挤成3字节:
 *             第1字节 = 传感器1的低8位
 *             第2字节 = 传感器1的高4位(放低半字节) | 传感器2的低4位(放高半字节)
 *             第3字节 = 传感器2的高8位
 *           没装第二个传感器时 传感器2固定填0xFFF 主控会把它当成无效
 * 字节6     CRC8校验 多项式0x07 初值0xFF 覆盖前6字节
 */
#define SCAN_FRAME_HEADER 0xA5
#define SCAN_FRAME_LENGTH 7
#define SCAN_RAW_NOT_INSTALLED 0x0FFFU //该传感器未安装

#define SCAN_SENSOR_COUNT 1 //已接入的传感器数量 1或2
                            //改成2之前 要先在CubeMX里把ADC1配成两个通道的扫描转换(见main.c里的说明)

#define SCAN_SAMPLE_RATE_HZ 1000 //采样率

typedef void (*UART_Callback_Function)(uint8_t* Buffer , uint16_t Length); //UART接收自定义数据处理函数

typedef struct //UART信息处理结构体
{

    UART_HandleTypeDef *UART_Handler;
    uint8_t Tx_Buffer[BUFFER_SIZE];
    uint8_t Rx_Buffer[BUFFER_SIZE]; //缓冲区字节长度
    uint16_t Rx_Buffer_Length;
    UART_Callback_Function Callback_Function;
    
}Struct_UART_Manage_Object;


extern UART_HandleTypeDef huart1;

extern Struct_UART_Manage_Object *USART1_Manage_Object; //UART1的通讯处理结构体

extern int ifsend;


void UART_Init(UART_HandleTypeDef *huart, UART_Callback_Function Callback_Function, uint16_t Length);//UART初始化函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size); // HAL库中断时处理信息函数

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length); //UART发送函数

#endif
