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

#include "sdio.h"
#include "fatfs.h"
#include "sd_card_test.h"


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

/*

      Motor3为前端丝杆结构控制电机 Motor4为后端丝杆结构控制电机

      */

Class_Motor_C620 motor;
Class_Motor_GM6020 motor2;
Class_Motor_C610 motor3;
Class_Motor_C610 motor4;
Class_Serialplot serialplot;

int IfPlotSending , IfsychroDataSending; //数据传回标志位

float Robot_Target_Distance, Robot_Now_Distance, Robot_Target_Speed, Robot_Now_Speed, //机器人距离与速度控制

      Rotate_Target_Angle, Rotate_Now_Angle, Rotate_Target_Speed, Rotate_Now_Speed; //旋转测距速度控制

//float Tube_Radius , Extra_Angle; //管道半径与角度余量 已作废: 管径自适应方案取消

/* Latest received laser distance in mm; -1 means invalid or not yet received. */

volatile float Rotation_Measured_Distance_mm = -1.0f; //L432KC开发板测距返回数据

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
 * @brief 电机反馈报文程序
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    CAN_Call_Back_Configuration(Rx_Buffer , motor , motor2 , motor3 , motor4);
}

/**
 * @brief 机器人数据返回电脑回调函数
 *
 * @param huart UART编号
 * @param Size 长度
 */

void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
 {
     Robot_Test_Serialplot_Call_Back_Configuration(Buffer , Length, serialplot, motor, motor2, motor3 ,&Robot_Target_Speed,&Robot_Target_Distance,&Rotate_Target_Speed,&Rotate_Target_Angle);

 } 

/**
 * @brief 测距部分返回数据处理 USART6接收中断里调用
 *        L432KC发来的是6字节二进制帧(帧头 采样序号 ADC原始值 校验) 格式见Scan_Sync.h
 *        解析、还原采样时刻、配上当时的转角和里程 都在同步模块里完成
 * 
 * @param Buffer 接收缓冲区
 * @param Length 本次收到的字节数
 */

void R_D_M_Data_Processing(uint8_t *Buffer, uint16_t Length)
{
    uint32_t Arrival_Time_us = Robot_Get_Time_us(); //先记下到达时刻 越早取越准

    /* DMA半满事件时接收还在进行 忽略 等空闲或收满时一次处理整段 */
    if (Buffer == NULL || huart6.RxState != HAL_UART_STATE_READY)
    {
        return;
    }

    Scan_Sync_Rx_Data(Buffer, Length, Arrival_Time_us);

    Rotation_Measured_Distance_mm = Scan_Sync_Get_Last_Distance_mm();
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

#if SD_CARD_TEST_MODE    //SD_CARD_TEST_MODE=1 时，进入独立 SD 卡读写测试模式，PH2~PH5 保持低电平，不执行后续机器人控制代码。0 时，正常运行机器人控制代码。
  /* 第三阶段独立测试：只初始化文件测试所需资源，PH2~PH5 保持低电平，
   * 不执行后面的 CAN、串口、电机控制和 24V 输出使能。
   * 测试成功或失败后都停在结果显示循环，不会继续进入机器人运行代码。
   * 要恢复机器人控制，请在 sd_card_test.h 中将 SD_CARD_TEST_MODE 改为 0。 */
  MX_GPIO_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  SD_Card_Test_Run_Storage_And_Read_Back();
  while (1)
  {
      SD_Card_Test_Show_Result();
      HAL_Delay(1);
  }
#else

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  /* 第一阶段：准备 SDIO 句柄并注册 FatFs 磁盘驱动。
   * 此处不挂载、不格式化、不打开文件，也不等待 SD 卡，避免无卡影响电机启动。
   * 后续挂载时，FatFs 的 disk_initialize 才调用 BSP_SD_Init 完成实际卡初始化。
   * 激光采样周期为 1.5 ms（约 667 次/秒）：后续应缓存并批量写入，
   * 接收中断只保存数据快照，不能在中断中直接调用文件或阻塞 SD 接口。 */
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
    
  Scan_Sync_Init(); //同步模块要在串口和CAN开始接收之前初始化

  Dictionary_Define(serialplot);
  UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH); //机器人控制串口
  UART_Init(&huart6, R_D_M_Data_Processing, ROTATION_DIS_MEASURE_DATA_MAX_LENGTH); //两块开发板沟通的通信模块连接

  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  CAN_Init(&hcan1, CAN_Motor_Call_Back);//24V接口和CAN外设初始化

  Robot_Control_Init(motor, motor2 , motor3 , motor4); //开启24V接口、CAN外设与电机初始化

  Set_M3508_Omega_PID_Parameters(1000.0f, 1330.0f, 0.0f, 20000.0f, motor); 
  Set_M3508_Angle_PID_Parameters(0.3f, 0.0f, 0.6f, motor); 
  Set_GM6020_Omega_PID_Parameters(80.0f, 100.0f, 0.0f, 2000.0f, motor2);
  Set_M2006_Omega_PID_Parameters(900.0f , 1000.0f , 0.0f , 20000.0f , motor3 , motor4);
  Set_M2006_Angle_PID_Parameters(3.0f , 0.0f , 0.2f , 0.0f , motor3 , motor4);//PID参数设置

  motor.TotalDistance = 0; //累积设0
  IfPlotSending = 1;
  Start_Synchornize(); //开机默认同步状态

  // zero3_state = ZERO_DONE;
  // zero4_state = ZERO_DONE; 

  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);

  //Structure_Init(Tube_Radius , Extra_Angle); //调零与结构自适应 调整PID参数时注释

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
    while (1)
    {
        // Move_Forward_Speed_Control(Robot_Target_Speed,MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);  

        // Robot_Move_Distance_Control(Robot_Target_Distance,MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);
        
        //四个电机先各自算PID 再统一发一次CAN
        Forward_PID_Processing(motor);

        Rotate_PID_Processing(motor2);

        Angle_Adjusting_PID_Processing(motor3);
        Angle_Adjusting_PID_Processing(motor4);

        Motor_CAN_Send_Processing();

        Robot_Test_Scan_Data_Processing(); //测点取出与回传
      
        Robot_Test_Serialplot_Plot(serialplot , motor , motor2 , motor3 , motor4 , MOTOR_WHEEL_OMEGA_RATIO , RADIUS);

        Robot_Offline_Detection();

        HAL_Delay(0); //等到下一个1ms节拍 PID里的时间步长D_T按1ms算 所以循环体必须在1ms内跑完
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
#endif /* SD_CARD_TEST_MODE */
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
  /* VCO=12/6*180=360 MHz；Q=8 为 SDIO 提供 45 MHz（必须不超过 48 MHz）。
   * SYSCLK 仍为 180 MHz，APB1/APB2 仍为 45/90 MHz，保留现有 CAN 与串口时序。
   * HAL 初始化分频 118 得 375 kHz；传输分频 0 得 22.5 MHz。
   * 此方案未提供 USB 所需的精确 48 MHz，未来启用 USB 时需要重新规划。 */
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
