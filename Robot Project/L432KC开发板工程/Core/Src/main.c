/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "UART_config.h"
#include "Dic_config.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE BEGIN PV */

volatile uint16_t adc_raw[2] = {0, 0};    // DMA 写入的原始值 [0]传感器1 [1]传感器2
volatile uint16_t sample_raw[2] = {0, 0}; // 采样完成中断里锁存的本次采样值
volatile uint16_t sample_seq = 0;         // 采样序号 每次采样加1 主控靠它发现丢点
volatile uint8_t adc_ready = 0;           // 新数据标志

/* USER CODE END PV */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 对于主板发送命令的处理函数
 * 
 * @param Buffer 
 * @param Length 
 */

void Command_Processing_Callback_Function(uint8_t* Buffer , uint16_t Length){ //UART回调函数

  Command_Processing(Length);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* 上电后执行一次；此时 TIM2 尚未启动 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {

    Error_Handler();

  }

  if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_raw, SCAN_SENSOR_COUNT) != HAL_OK)
  {

    Error_Handler();

  }

  /*
   
  开始以 1000 Hz 采样 : HAL_TIM_Base_Start(&htim2);
  停止产生采样触发 : HAL_TIM_Base_Stop(&htim2);
  读取最新 ADC 原始值 : uint16_t raw = adc_raw[0];

  接第二个传感器时(SCAN_SENSOR_COUNT改为2) 需要先在CubeMX里改ADC1:
    1. 再勾选一个输入通道(第二个传感器接的引脚)
    2. Number Of Conversion 改为 2  Rank 1 选传感器1的通道  Rank 2 选传感器2的通道
    3. Scan Conversion Mode 会自动变为 Enabled  其余保持不变(仍由TIM2触发 DMA循环模式)
  这样每次TIM2触发 两个通道依次转换 DMA把结果写进adc_raw[0]和adc_raw[1] 转换完成中断在两个都转完后进一次
   
   */

  /* TIM2计数频率为1MHz 按SCAN_SAMPLE_RATE_HZ设定采样率 覆盖CubeMX里填的周期 */
  __HAL_TIM_SET_AUTORELOAD(&htim2, 1000000U / SCAN_SAMPLE_RATE_HZ - 1U);

  UART_Init(&huart1, Command_Processing_Callback_Function, COMMAND_MAX_SIZE); //UART接收初始化 定义数据处理函数


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * 每个采样发一帧 7字节 带采样序号和两个传感器的ADC原始值 帧格式见UART_config.h
     * 换算成距离放到主控做
     * 7字节发完约0.61ms 赶得上1ms一次的采样
     * 如果上一帧还没发完 这个采样留到下一圈循环 来不及就被新采样覆盖 主控从序号跳变能看出来
     */
    if (ifsend && adc_ready && huart1.gState == HAL_UART_STATE_READY)
    {
      static uint8_t tx_frame[SCAN_FRAME_LENGTH];
      uint16_t raw1;
      uint16_t raw2;
      uint16_t seq;
      uint8_t crc = 0xFF;
      uint8_t i;
      uint8_t bit;

      /* 序号和采样值要成套取出 中间不能被采样中断打断 */
      __disable_irq();
      raw1 = sample_raw[0];
      raw2 = sample_raw[1];
      seq = sample_seq;
      adc_ready = 0;
      __enable_irq();

      tx_frame[0] = SCAN_FRAME_HEADER;
      tx_frame[1] = (uint8_t)(seq & 0xFF);
      tx_frame[2] = (uint8_t)(seq >> 8);

      /* 两个12位采样挤成3字节 */
      tx_frame[3] = (uint8_t)(raw1 & 0xFF);
      tx_frame[4] = (uint8_t)(((raw1 >> 8) & 0x0F) | ((raw2 & 0x0F) << 4));
      tx_frame[5] = (uint8_t)(raw2 >> 4);

      /* CRC8 多项式0x07 初值0xFF */
      for (i = 0; i < SCAN_FRAME_LENGTH - 1; i++)
      {
        crc ^= tx_frame[i];

        for (bit = 0; bit < 8; bit++)
        {
          crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
      }

      tx_frame[SCAN_FRAME_LENGTH - 1] = crc;

      UART_Send_Data(&huart1, tx_frame, SCAN_FRAME_LENGTH);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        //立刻锁存 下一次采样会覆盖adc_raw
        sample_raw[0] = (uint16_t)(adc_raw[0] & 0x0FFF);
#if SCAN_SENSOR_COUNT >= 2
        sample_raw[1] = (uint16_t)(adc_raw[1] & 0x0FFF);
#else
        sample_raw[1] = SCAN_RAW_NOT_INSTALLED; //没装第二个传感器
#endif
        sample_seq++;
        adc_ready = 1;
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
