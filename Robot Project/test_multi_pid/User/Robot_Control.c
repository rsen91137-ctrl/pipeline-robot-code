#include "Robot_Control.h"

float Robot_Move_Target_Delta_Distance = 0.0f;
float Robot_Move_Start_Angle = 0.0f;

Zero_State zero3_state = ZERO_WAIT_PRESS;
Zero_State zero4_state = ZERO_WAIT_PRESS;

/**
 * @brief 初始化所有配置
 * 
 */
void Robot_Control_Init(Class_Motor_C620 &motor, Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4){

    motor.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 2500.0f);
    motor.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 3591.0f / 187.0f); 
    //初始化为速度环

    motor2.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor2.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 2500.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 1.0f); 
    //初始化为速度环

    motor3.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor3.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 2500.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE,36.0f);
    //初始化为角度环
    
    motor4.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor4.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 2500.0f);
    motor4.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_ANGLE,36.0f);
    //初始化为角度环

}


/**
 * @brief 设置M3508的PID参数
 * 
 */
 
void Set_M3508_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C620 &motor){

    motor.PID_Omega.Set_K_P(Omega_Kp);
    motor.PID_Omega.Set_K_I(Omega_Ki);
    motor.PID_Omega.Set_K_D(Omega_Kd);
    motor.PID_Omega.Set_K_F(Omega_Kf);
    
}

void Set_M3508_Angle_PID_Parameters(float Angle_Kp, float Angle_Ki, float Angle_Kd, Class_Motor_C620 &motor){

    motor.PID_Angle.Set_K_P(Angle_Kp);
    motor.PID_Angle.Set_K_I(Angle_Ki);
    motor.PID_Angle.Set_K_D(Angle_Kd);
    
}

/**
 * @brief 设置GM6020速度PID参数
 * 
 */
 
void Set_GM6020_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_GM6020 &motor2){

    motor2.PID_Omega.Set_K_P(Omega_Kp);
    motor2.PID_Omega.Set_K_I(Omega_Ki);
    motor2.PID_Omega.Set_K_D(Omega_Kd);
    motor2.PID_Omega.Set_K_F(Omega_Kf);

}


/**
 * @brief 设置M2006的PID参数
 * 
 * @param Robot_Target_Speed 
 * @param Wheel_Omega_Ratio 
 * @param Wheel_Radius 
 * @param motor 
 */

void Set_M2006_Omega_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4){

    motor3.PID_Omega.Set_K_P(Omega_Kp);
    motor3.PID_Omega.Set_K_I(Omega_Ki);
    motor3.PID_Omega.Set_K_D(Omega_Kd);
    motor3.PID_Omega.Set_K_F(Omega_Kf);

    motor4.PID_Omega.Set_K_P(Omega_Kp);
    motor4.PID_Omega.Set_K_I(Omega_Ki);
    motor4.PID_Omega.Set_K_D(Omega_Kd);
    motor4.PID_Omega.Set_K_F(Omega_Kf);

}

void Set_M2006_Angle_PID_Parameters(float Omega_Kp, float Omega_Ki, float Omega_Kd, float Omega_Kf, Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4){

    motor3.PID_Angle.Set_K_P(Omega_Kp);
    motor3.PID_Angle.Set_K_I(Omega_Ki);
    motor3.PID_Angle.Set_K_D(Omega_Kd);
    motor3.PID_Angle.Set_K_F(Omega_Kf);

    motor4.PID_Angle.Set_K_P(Omega_Kp);
    motor4.PID_Angle.Set_K_I(Omega_Ki);
    motor4.PID_Angle.Set_K_D(Omega_Kd);
    motor4.PID_Angle.Set_K_F(Omega_Kf);

}


/**
 * @brief 机器人前进速度控制
 * @param Wheel_Omega_Ratio M350角速度与后轮角速度之比
 * @param Wheel_Radius 单位米
 * @param Robot_Target_Speed 机器人目标速度 单位米每秒
 * @param Robot_Now_Speed 机器人当前速度 单位米每秒
 *
 * 
 */

void Move_Forward_Speed_Control(float Robot_Target_Speed , float Wheel_Omega_Ratio , float Wheel_Radius ,Class_Motor_C620 &motor) {

    float M3508_Target_Omega;

    M3508_Target_Omega = (Robot_Target_Speed) * (Wheel_Omega_Ratio) / (Wheel_Radius);

    motor.Set_Target_Omega(M3508_Target_Omega);

}


/**
 * @brief 机器人移动固定距离控制 记得先设置为角度环模式
 * 
 * @param Robot_Target_Distance 机器人目标距离 单位米
 * @param Wheel_Omega_Ratio M3508角速度与后轮角速度之比
 * @param Wheel_Radius 单位米
 * @param motor 
 */


void Robot_Move_Distance_Control(float Robot_Target_Distance, float Wheel_Omega_Ratio, float Wheel_Radius, Class_Motor_C620 &motor){
    static float pre_Robot_Target_Distance = 0.0f;
     if (Math_Abs(Robot_Target_Distance - pre_Robot_Target_Distance) > 0.0001f){
        Robot_Move_Target_Delta_Distance = Robot_Target_Distance; // 存下 ta
        Robot_Move_Start_Angle = motor.Get_Now_Angle(); // 存下基准角
        pre_Robot_Target_Distance = Robot_Target_Distance;
    }
    motor.Set_Target_Angle(Robot_Move_Start_Angle + Robot_Move_Target_Delta_Distance * Wheel_Omega_Ratio / Wheel_Radius);
}

/**
 * @brief 旋转测距部分RPM控制
 * @param Rotate_Target_Speed 旋转速度 单位RPM
 */

void Rotate_Distance_Measurement_Speed_Control(float Rotate_Target_Speed , Class_Motor_GM6020 &motor2) {

    motor2.Set_Target_Omega(Rotate_Target_Speed * RPM_TO_RADPS);

}

/**
 * @brief 驱动电机M3508的PID计算以及传回电机
 * 
 */


void Forward_PID_Processing(Class_Motor_C620 &motor){

    if(motor.Get_Control_Method() == Control_Method_OMEGA){

          if(motor.Get_Target_Omega() != 0){

            motor.TIM_PID_PeriodElapsedCallback(); //PID计算
            TIM_CAN_PeriodElapsedCallback(); //传回电机进行控制


          }

        }
        else if(motor.Get_Control_Method() == Control_Method_ANGLE){

          if(Robot_Move_Target_Delta_Distance != 0){

             motor.TIM_PID_PeriodElapsedCallback(); 
             TIM_CAN_PeriodElapsedCallback(); 


          }

        }
}

/**
 * @brief 旋转电机GM6020的PID计算以及传回电机
 */

void Rotate_PID_Processing(Class_Motor_GM6020 &motor2){

        if(motor2.Get_Target_Omega() != 0){

            motor2.TIM_PID_PeriodElapsedCallback(); //PID计算
            TIM_CAN_PeriodElapsedCallback(); //传回电机进行控制


          }
    

}

/**
 * @brief 丝杆调整电机M2006的PID计算
 * 
 */


void Angle_Adjusting_PID_Processing(Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4){

    motor3.TIM_PID_PeriodElapsedCallback(); //PID计算
    TIM_CAN_PeriodElapsedCallback(); //传回电机进行控制

    motor4.TIM_PID_PeriodElapsedCallback(); //PID计算
    TIM_CAN_PeriodElapsedCallback(); //传回电机进行控制

}


/**
 * @brief 获取当前机器人的速度
 * 
 * @param Wheel_Omega_Ratio 
 * @param Wheel_Radius 
 * @return float 
 */

float Get_Robot_Now_Speed(Class_Motor_C620 &motor, float Wheel_Omega_Ratio , float Wheel_Radius){

    float M3508_Now_Omega;
    float Robot_Now_Speed;

    M3508_Now_Omega = motor.Get_Now_Omega();

    Robot_Now_Speed = (M3508_Now_Omega) * (Wheel_Radius) / (Wheel_Omega_Ratio);

    return Robot_Now_Speed;

}

/**
 * @brief 获取当前旋转速度 单位rpm
 * @return float 
 */

float Get_Rotate_Now_Speed(Class_Motor_GM6020 &motor2){

    float GM6020_Now_Omega;
    float Rotate_Now_Speed;

    GM6020_Now_Omega = motor2.Get_Now_Omega();

    Rotate_Now_Speed = GM6020_Now_Omega * RADS_TO_RPM;

    return Rotate_Now_Speed;

}

float Get_Robot_Target_Speed(){



}

float Get_Robot_Target_Distance(){



}

float Get_Rotate_Target_Omega(){



}



/**
 * @brief 管理CAN回调函数
 * 
 */


void CAN_Call_Back_Configuration(Struct_CAN_Rx_Buffer *Rx_Buffer , Class_Motor_C620 &motor , Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3, Class_Motor_C610 &motor4){

    switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
        {

            motor.CAN_RxCpltCallback(Rx_Buffer->Data);

        }
        break;
        case(0x206):
        {

            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);

        }
        break;
        case(0x202):{

            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);

        }
        break;
        case(0x203):{

            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);

        }
        break;
    }

}



/**
 * @brief 调零开关回调函数
 * 
 */

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

//PC0 为前端丝杆结构的调零开关
if (GPIO_Pin == GPIO_PIN_0)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if (level == GPIO_PIN_SET && zero3_state == ZERO_WAIT_PRESS)
    {
        zero3_state = ZERO_BACKING;

        // motor3 切换为速度环，设置退出开关方向的速度
    }
    else if (level == GPIO_PIN_RESET && zero3_state == ZERO_BACKING)
    {
        // motor3 停机，并记录当前位置为零点

        zero3_state = ZERO_DONE;
    }
}


//PC1 为前端丝杆结构的调零开关
else if (GPIO_Pin == GPIO_PIN_1){
    
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

    if (level == GPIO_PIN_SET && zero4_state == ZERO_WAIT_PRESS)
    {
        zero4_state = ZERO_BACKING;

        // motor3 切换为速度环，设置退出开关方向的速度
    }
    else if (level == GPIO_PIN_RESET && zero4_state == ZERO_BACKING)
    {
        // motor3 停机，并记录当前位置为零点

        zero4_state = ZERO_DONE;
    }
}

}

/**
 * @brief 
 * 
 * 
 */


