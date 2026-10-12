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

#define ANGLE_TO_RADS ( / 180.0f)
#define RADS_TO_RPM (60.0f / (2.0f * PI))

#define RADIUS 0.018 //驱动轮半径 暂时填0.0018
#define MOTOR_WHEEL_OMEGA_RATIO 1.0

#define MOTOR3_POSITIVE_DERECTION_SYMBOL (+1.0f) 
#define MOTOR3_NEGATIVE_DERECTION_SYMBOL (-1.0f)
/*设前端电机motor3将螺母向正/负方向移动(从零点向下 拉开力臂 即正向移动,向零点靠近 收缩力臂 即反向移动)需要设置的速度与角度正负号
 例如实际安装后 目标设为+45 实际可能是反方向运动
  所以先设定方向 方便修改*/

#define MOTOR4_POSITIVE_DERECTION_SYMBOL (+1.0f) 
#define MOTOR4_NEGATIVE_DERECTION_SYMBOL (-1.0f)
//同上

#define FRONT_ANGLE_MINIMUM 20 //前端力臂最小夹角 角度制
#define FRONT_ANGLE_MAXIMUM 120 //前端力臂最大夹角 角度制
#define FRONT_LENGTH 0.1 //前端力臂长度 转轴到轮心 单位米
#define FRONT_NODE_LENGTH 0.05 //前端弹簧连接支点到转轴距离 单位米
#define FRONT_ZERO_POINT 0.03 //前端丝杆零点到转轴距离 单位米

#define REAR_ANGLE_MINIMUM 20 //后端类似
#define REAR_ANGLE_MAXIMUM 120
#define REAR_LENGTH 0.1
#define REAR_NODE_LENGTH 0.05
#define REAR_ZERO_POINT 0.03

#define SETZEROSPEED 2 //调零螺母正向移动,电机角速度 单位rad/s
#define BACKSPEED 2 //螺母触点后回退,电机角速度 单位rad/s

#define SPRING_ORIGINAL_LENGTH 0.1 //弹簧原长 单位米
#define SCREW_PITCH 0.001 //螺距 单位米
#define SCREW_NUM 1 //螺纹数量


extern float Robot_Move_Target_Delta_Distance; // 记录上位机 ta：本次要走的距离差（米） 便于串口绘图显示距离
extern float Robot_Move_Start_Angle;             // 记录每次移动距离ta指令发出后的起始角度，便于计算当前距离
extern float motor3_Target_Delta_Revolutions;
extern float motor3_Start_Angle;
extern float motor4_Target_Delta_Revolutions;
extern float motor4_Start_Angle;

// float Robot_Target_Distance, Robot_Now_Distance, Robot_Target_Speed, Robot_Now_Speed,
//       Rotate_Target_Angle, Rotate_Now_Angle, Rotate_Target_Speed, Rotate_Now_Speed,
//       Wheel_Omega_Ratio, //M3508角速度与后轮角速度之比
//       Wheel_Radius //后轮半径 单位米

      ;

//调零开关
enum Zero_State
{
    ZERO_WAIT_PRESS,  // 等待按下
    ZERO_BACKING,     // 已按下，正在反转退出
    ZERO_DONE        // 已松开，调零完成
};

extern Zero_State zero3_state;
extern Zero_State zero4_state;

extern Class_Motor_C610 motor3;
extern Class_Motor_C610 motor4;


/**
 * @brief 要找函数别傻乎乎的跑去control.c里面翻半天 control.h里面已经整理的非常有条理了 对准你要看或改的函数按下F12就行
 * 
 */

//初始化
void Robot_Control_Init(Class_Motor_C620 &motor, Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4);


//PID参数调节
void Set_M3508_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C620 &motor);

void Set_M3508_Angle_PID_Parameters(float Angle_Kp, float Angle_Ki, float Angle_Kd, Class_Motor_C620 &motor);

void Set_GM6020_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_GM6020 &motor2);

void Set_M2006_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4);

void Set_M2006_Angle_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4);


//机器人各项参数控制
void Move_Forward_Speed_Control(float Robot_Target_Speed , float Wheel_Omega_Ratio , float Wheel_Radius ,Class_Motor_C620 &motor);

void Robot_Move_Distance_Control(float Robot_Target_Distance, float Wheel_Omega_Ratio, float Wheel_Radius, Class_Motor_C620 &motor);

void Rotate_Distance_Measurement_Speed_Control(float Rotate_Target_Speed , Class_Motor_GM6020 &motor2);

void M2006_Rotate_Specified_Revolutions(float Target_Revolutions , Class_Motor_C610 &motor);  //M2006旋转指定圈数

void Screw_Structure_Angle_Set(float Target_Angle , Class_Motor_C610 &motor);  //丝杆结构角度控制(空载 仅用于初步拟合)



//PID运算
void Forward_PID_Processing(Class_Motor_C620 &motor);

void Rotate_PID_Processing(Class_Motor_GM6020 &motor2);

void Angle_Adjusting_PID_Processing(Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4);


//反馈
float Get_Robot_Now_Speed(Class_Motor_C620 &motor, float Wheel_Omega_Ratio , float Wheel_Radius);

float Get_Rotate_Now_Speed(Class_Motor_GM6020 &motor2);


//回调函数配置
void CAN_Call_Back_Configuration(Struct_CAN_Rx_Buffer *Rx_Buffer , Class_Motor_C620 &motor , Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3, Class_Motor_C610 &motor4);


//调零函数
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); //按钮触发回调函数

void Motor_Set_Zero(Class_Motor_C610 &motor);//调零执行函数

void Motor_Back(Class_Motor_C610 &motor);//电机回转函数

void Operate_Motor_Set_Zero(Class_Motor_C610 &motor);//调零启动函数


//转换计算函数


#endif
