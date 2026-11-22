#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32l4xx_hal.h"  
#include "usart.h"
#define BUFFER_SIZE 256
#define COMMAND_MAX_SIZE 100

/* 发给主控的测距帧 必须与主控工程 Scan_Sync.h 里的定义一致
 * 433M模块在128k空中速率下 官方保证能持续传输的量约1920字节/秒(对应19200波特率)
 * 一个采样发一帧的话 1000Hz要6000字节/秒 超了 所以把16个采样合成一帧:
 *   每帧28字节 每16ms一帧 合1750字节/秒 在保证值以内
 *
 * 字节0       帧头 0xA5
 * 字节1~2     本帧第一个采样的序号 uint16 小端 每个采样加1 溢出回0
 * 字节3~26    16个ADC原始值 每个12位 两个采样a b挤成3字节:
 *               第1字节 = a的低8位
 *               第2字节 = a的高4位(放低半字节) | b的低4位(放高半字节)
 *               第3字节 = b的高8位
 * 字节27      CRC8校验 多项式0x07 初值0xFF 覆盖前27字节
 */
#define SCAN_FRAME_HEADER 0xA5
#define SCAN_SAMPLES_PER_FRAME 16 //每帧采样数 必须是偶数 改它要同时改两块板
#define SCAN_FRAME_LENGTH (3 + SCAN_SAMPLES_PER_FRAME * 3 / 2 + 1)

#define SCAN_SAMPLE_RATE_HZ 1000 //采样率 改它要同时改主控的SCAN_SAMPLE_PERIOD_US(= 1000000 / 采样率)
                                 //链路仍然丢数据时可降到800: 字节量降到1400字节/秒

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
