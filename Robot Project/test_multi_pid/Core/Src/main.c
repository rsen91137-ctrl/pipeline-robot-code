/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "drv_bsp.h"
#include "dvc_serialplot.h"
#include "dvc_motor.h"
#include "stdio.h"
#include "oled.h"
#include "string.h"
#include "Robot_Control.h"
#include "Robot_test.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



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

Class_Serialplot serialplot;

Class_Motor_C620 motor; //这里定义是c620电调
Class_Motor_GM6020 motor2; //这里定义是GM6020电机

Class_Motor_C610 motor3; //两个M2006电机
Class_Motor_C610 motor4;

/*

      Motor3为前端丝杆结构控制电机 Motor4为后端丝杆结构控制电机

      */

float Robot_Target_Distance, Robot_Now_Distance, Robot_Target_Speed, Robot_Now_Speed, //机器人距离与速度控制

      Rotate_Target_Angle, Rotate_Now_Angle, Rotate_Target_Speed, Rotate_Now_Speed, //旋转测距速度控制

      //丝杆结构角度控制放在c610类中
      
      Tube_Radius , Extra_Angle; //管道半径与角度余量

uint32_t Counter = 0;

const char *msg;

char oled_text[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    CAN_Call_Back_Configuration(Rx_Buffer , motor , motor2 , motor3 , motor4);
}

/**
 * @brief 串口接收回调函数
 *
 * @param huart UART编号
 * @param Size 长度
 */
 void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
 {
     Robot_Test_Serialplot_Call_Back_Configuration(Buffer , Length, serialplot, motor, motor2,&Robot_Target_Speed,&Robot_Target_Distance,&Rotate_Target_Speed,&Rotate_Target_Angle);

 } //串口调试 不用时和下面一起注释

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
    
  Dictionary_Define(serialplot);
  UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH); //串口绘图

  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  CAN_Init(&hcan1, CAN_Motor_Call_Back);//24V接口和CAN外设初始化

  Robot_Control_Init(motor, motor2 , motor3 , motor4); //开启24V接口、CAN外设与电机初始化

  Set_M3508_Omega_PID_Parameters(1000.0f, 1330.0f, 0.0f, 20000.0f, motor); 
  Set_M3508_Angle_PID_Parameters(0.3f, 0.0f, 0.6f, motor); 
  Set_GM6020_Omega_PID_Parameters(80.0f, 100.0f, 0.0f, 2000.0f, motor2);
  Set_M2006_Omega_PID_Parameters(0.0f , 0.0f , 0.0f , 0.0f , motor3 , motor4);
  Set_M2006_Angle_PID_Parameters(0.0f , 0.0f , 0.0f , 0.0f , motor3 , motor4);//PID参数设置

  Structure_Init(Tube_Radius , Extra_Angle);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
    while (1)
    {
        // Move_Forward_Speed_Control(Robot_Target_Speed,MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);  

        // Robot_Move_Distance_Control(Robot_Target_Distance,MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);
        
        Forward_PID_Processing(motor);

        Rotate_PID_Processing(motor2);

        Angle_Adjusting_PID_Processing(motor3 , motor4);
      
        Robot_Test_Serialplot_Plot(serialplot , motor , motor2 , MOTOR_WHEEL_OMEGA_RATIO , RADIUS);

        HAL_Delay(0);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
