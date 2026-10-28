#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32l4xx_hal.h"  
#include "usart.h"
#define BUFFER_SIZE 256
#define COMMAND_MAX_SIZE 100

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


void UART_Init(UART_HandleTypeDef *huart, UART_Callback_Function Callback_Function, uint16_t Length);//UART初始化函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size); // HAL库中断时处理信息函数

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length); //UART发送函数

#endif
