#include "UART_config.h"

Struct_UART_Manage_Object USART1_Manage_Object= {0};

/**
 * @brief UART初始化函数
 * 
 * @param huart 
 * @param Callback_Function 
 * @param Length 
 */

void UART_Init(UART_HandleTypeDef *huart, UART_Callback_Function Callback_Function, uint16_t Length){

    if(huart->Instance == USART1){

        USART1_Manage_Object.UART_Handler = huart;
        USART1_Manage_Object.Callback_Function = Callback_Function;
        USART1_Manage_Object.Rx_Buffer_Length = Length;
        HAL_UARTEx_ReceiveToIdle_DMA(USART1_Manage_Object.UART_Handler, USART1_Manage_Object.Rx_Buffer, USART1_Manage_Object.Rx_Buffer_Length); //开启UART空闲中断和DMA接收

    }

}

/**
 * @brief 中断时HAL库处理信息函数
 * 
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

    if(huart->Instance == USART1){

        USART1_Manage_Object.Callback_Function(USART1_Manage_Object.Rx_Buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(USART1_Manage_Object.UART_Handler, USART1_Manage_Object.Rx_Buffer, USART1_Manage_Object.Rx_Buffer_Length); //重新开启UART空闲中断和DMA接收

    }

}

/**
 * @brief UART发送函数
 * 
 */

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{

    return (HAL_UART_Transmit_DMA(huart, Data, Length));

}
