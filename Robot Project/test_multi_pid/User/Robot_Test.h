#ifndef __Robot_Test_h__
#define __Robot_Test_h__

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

void Dictionary_Define(class Class_Serialplot &serialplot);

// void Robot_Test_Serialplot_Call_Back_Configuration(uint8_t *Buffer,
//     uint16_t Length,
//     Class_Serialplot &serialplot,
//     Class_Motor_C620 &motor,
//     Class_Motor_GM6020 &motor2);

void Robot_Test_Serialplot_Call_Back_Configuration(uint8_t *Buffer,
    uint16_t Length,
    Class_Serialplot &serialplot,
    Class_Motor_C620 &motor,
    Class_Motor_GM6020 &motor2,
    float *Robot_Target_Speed,
    float *Robot_Target_Distance,
    float *Rotate_Target_Speed,
    float *Rotate_Target_Angle);

void Robot_Test_Serialplot_Plot(Class_Serialplot &serialplot , Class_Motor_C620 &motor , Class_Motor_GM6020 &motor2 , float Wheel_Omega_Ratio , float Wheel_Radius);

#endif
