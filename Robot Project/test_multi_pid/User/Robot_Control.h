#ifndef __Robot_Control_h__
#define __Robot_Control_h__

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

#define RADS_TO_RPM (60.0f / (2.0f * PI))

// float Robot_Target_Distance, Robot_Now_Distance, Robot_Target_Speed, Robot_Now_Speed,
//       Rotate_Target_Angle, Rotate_Now_Angle, Rotate_Target_Speed, Rotate_Now_Speed,
//       Wheel_Omega_Ratio, //M3508角速度与后轮角速度之比
//       Wheel_Radius //后轮半径 单位米

      ;

void Robot_Control_Init(Class_Motor_C620 &motor, Class_Motor_GM6020 &motor2);

void Set_M3508_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C620 &motor);

void Set_GM6020_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_GM6020 &motor2);

void Move_Forward_Speed_Control(float Robot_Target_Speed , float Wheel_Omega_Ratio , float Wheel_Radius ,Class_Motor_C620 &motor);

void Forward_PID_Processing(Class_Motor_C620 &motor);

float Get_Robot_Now_Speed(Class_Motor_C620 &motor, float Wheel_Omega_Ratio , float Wheel_Radius);



void Rotate_Distance_Measurement_Speed_Control(float Rotate_Target_Speed , Class_Motor_GM6020 &motor2);

void Rotate_PID_Processing(Class_Motor_GM6020 &motor2);

float Get_Rotate_Now_Speed(Class_Motor_GM6020 &motor2);

#endif
