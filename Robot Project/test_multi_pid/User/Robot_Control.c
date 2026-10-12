#include "Robot_Control.h"

//定义一下.h中的extern变量
float Robot_Move_Target_Delta_Distance = 0.0f;
float Robot_Move_Start_Angle = 0.0f;
float motor3_Target_Delta_Revolutions = 0.0f;
float motor3_Start_Angle = 0.0f;
float motor4_Target_Delta_Revolutions = 0.0f;
float motor4_Start_Angle = 0.0f;

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
        Robot_Move_Target_Delta_Distance = Robot_Target_Distance; // 存下 ta
        Robot_Move_Start_Angle = motor.Get_Now_Angle(); // 存下基准角
    motor.Set_Target_Angle(Robot_Move_Start_Angle + Robot_Move_Target_Delta_Distance * Wheel_Omega_Ratio / Wheel_Radius);
}

/**
 * @brief 旋转测距部分RPM控制
 * @param Rotate_Target_Speed 旋转速度 单位RPM
 */

void Rotate_Distance_Measurement_Speed_Control(float Rotate_Target_Speed , Class_Motor_GM6020 &motor2) {

    motor2.Set_Target_Omega(Rotate_Target_Speed * RPM_TO_RADPS);

}


void M2006_Rotate_Specified_Revolutions(float Target_Revolutions , Class_Motor_C610 &motor){
    
    if(&motor == &motor3){
        motor3_Start_Angle = motor3.Get_Now_Angle();
        motor3_Target_Delta_Revolutions = Target_Revolutions;
    motor3.Set_Target_Angle(motor3_Start_Angle + motor3_Target_Delta_Revolutions * 2 * PI);
    }
    
    else if(&motor == &motor4){
        motor4_Start_Angle = motor4.Get_Now_Angle();
        motor4_Target_Delta_Revolutions = Target_Revolutions;
    motor4.Set_Target_Angle(motor4_Start_Angle + motor4_Target_Delta_Revolutions * 2 * PI);
    }

 }

 
/**
 * @brief 丝杆结构角度控制(空载 仅用于初步拟合) 输入为弧度制
 * 
 */
 
void Screw_Structure_Angle_Set(float Target_Angle , Class_Motor_C610 &motor){

    float Rotate_Num = 0.0f , Rotate_Angle = 0.0f , RAD_Target_Angle = 0.0f;

    if(&motor == &motor3){ //前端电机拟合

        Rotate_Num = Angle_Round_Transfer(0.5 * Target_Angle , motor3);
        Rotate_Angle = 2 * PI * Rotate_Num * MOTOR3_DERECTION_SYMBOL;

    }
    else if(&motor == &motor4){

        Rotate_Num = Angle_Round_Transfer(0.5 * Target_Angle , motor4);
        Rotate_Angle = 2 * PI * Rotate_Num * MOTOR4_DERECTION_SYMBOL; 

    }

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

//PC1 为前端丝杆结构的调零开关
if (GPIO_Pin == GPIO_PIN_1){
    
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

    if (level == GPIO_PIN_SET && zero3_state == ZERO_WAIT_PRESS)
    {
        zero3_state = ZERO_BACKING;

        Motor_Back(motor3);

        // motor3 设置退出开关方向的速度
    }
    else if (level == GPIO_PIN_RESET && zero3_state == ZERO_BACKING)
    {

        Motor_Set_Zero(motor3);
        // motor3 切换回角度环，停机，并记录当前位置为零点

        zero3_state = ZERO_DONE;
    }
}

//PC0 为前端丝杆结构的调零开关
else if (GPIO_Pin == GPIO_PIN_0)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if (level == GPIO_PIN_SET && zero4_state == ZERO_WAIT_PRESS)
    {
        zero4_state = ZERO_BACKING;

        Motor_Back(motor4);

        // motor4 设置退出开关方向的速度
    }
    else if (level == GPIO_PIN_RESET && zero3_state == ZERO_BACKING)
    {

        Motor_Set_Zero(motor4);
        // motor4 切换回角度环，停机 ，并记录当前位置为零点

        zero4_state = ZERO_DONE;
    }
}


}

/**
 * @brief 电机停机与调零 调零程序
 * 
 */

void Motor_Set_Zero(Class_Motor_C610 &motor){

    if(&motor == &motor3){

    motor3.Set_Control_Method(Control_Method_ANGLE); //切换回角度环

    motor3.Set_Target_Angle(motor3.Get_Now_Angle());
    motor3.Set_Target_Omega(0.0f); //停机

    motor3.Front_Structure_Now_Angle = FRONT_ANGLE_MINIMUM; //设置零点
    motor3.Front_Structure_Now_Coordinate = 0;

    }
    else if(&motor == &motor4){

    motor4.Set_Control_Method(Control_Method_ANGLE); //切换回角度环

    motor4.Set_Target_Angle(motor4.Get_Now_Angle());
    motor4.Set_Target_Omega(0.0f); //停机

    motor4.Front_Structure_Now_Angle = REAR_ANGLE_MINIMUM; //设置零点
    motor4.Front_Structure_Now_Coordinate = 0;


    }

}


/**
 * @brief 电机触点后退回 即前端电机运动负方向
 * 
 */

void Motor_Back(Class_Motor_C610 &motor){

    if(&motor == &motor3){

    motor3.Set_Target_Omega(MOTOR3_DERECTION_SYMBOL * BACKSPEED); //负方向速度

    }
    else if(&motor == &motor4){

    motor4.Set_Target_Omega(MOTOR4_DERECTION_SYMBOL * BACKSPEED);

    }

}



/**
 * @brief 电机调零程序启动 执行程序
 * 
 */

void Operate_Motor_Set_Zero(Class_Motor_C610 &motor){

    if(&motor == &motor3){

    if(zero3_state == ZERO_DONE){ //判断是否为非调零中状态 调零时向下运动 为正方向

    motor3.Set_Control_Method(Control_Method_OMEGA);
    motor3.Set_Target_Omega(MOTOR3_DERECTION_SYMBOL * SETZEROSPEED);

    zero3_state == ZERO_WAIT_PRESS;

         }
    }
    else if(&motor == &motor4){

    if(zero4_state == ZERO_DONE){ //判断是否为非调零中状态

    motor4.Set_Control_Method(Control_Method_OMEGA);
    motor4.Set_Target_Omega(MOTOR4_DERECTION_SYMBOL * SETZEROSPEED);

    zero4_state == ZERO_WAIT_PRESS;

         }
    }

}

/**
 * @brief 自动拟合函数
 * 
 * @param Radius 管道半径
 * @param Extra 拟合管壁后的余量 使得机器人完全卡紧 可以通过实验修改
 */

void Auto_Fit_Operation(Class_Motor_C610 &motor , float Radius , float Extra){


    if(&motor == &motor3){

        Screw_Structure_Angle_Set( Angle_Caculate_From_Radius(Radius) + Extra , motor3);

    }
    else if(&motor == &motor4){

        Screw_Structure_Angle_Set( Angle_Caculate_From_Radius(Radius) + Extra , motor4);

    }

}

/**
 * @brief 初始化机械结构 调零后卡紧管壁
 * 
 * @param Radius 管道半径
 */
 
void Structure_Init(float Radius , float Extra){

    Operate_Motor_Set_Zero(motor3);
    Operate_Motor_Set_Zero(motor4);

    HAL_Delay(15000);

    Auto_Fit_Operation(motor3 , Radius , Extra);
    Auto_Fit_Operation(motor4 , Radius , Extra);

}


/**
 * @brief 转换角度与旋转圈数
 * 
 */

float Angle_Round_Transfer(float Angle , Class_Motor_C610 &motor){

    float Absolute_Coordinate = 0.0f , Distance , Round;

    if(&motor == &motor3){ //前端电机

    Absolute_Coordinate = sqrt((FRONT_NODE_LENGTH)*(FRONT_NODE_LENGTH) - (SPRING_ORIGINAL_LENGTH * sin(0.5 * Angle))*(SPRING_ORIGINAL_LENGTH * sin(0.5 * Angle))) - SPRING_ORIGINAL_LENGTH * cos(0.5 * Angle) - FRONT_ZERO_POINT;

    Distance = Absolute_Coordinate - motor.Front_Structure_Now_Coordinate; //表示螺母还要运动的距离

    motor.Front_Structure_Now_Coordinate += Distance; //累加坐标

    Round = Distance * DISTANCE_TO_ROUND;

    Distance = 0;

    return Round;

    }
    else if(&motor == &motor4){ //后端电机

    Absolute_Coordinate = sqrt((REAR_NODE_LENGTH)*(REAR_NODE_LENGTH) - (SPRING_ORIGINAL_LENGTH * sin(0.5 * Angle))*(SPRING_ORIGINAL_LENGTH * sin(0.5 * Angle))) - SPRING_ORIGINAL_LENGTH * cos(0.5 * Angle) - REAR_ZERO_POINT;

    Distance = Absolute_Coordinate - motor.Rear_Structure_Now_Coordinate; //表示螺母还要运动的距离

    motor.Rear_Structure_Now_Coordinate += Distance; //累加坐标

    Round = Distance * DISTANCE_TO_ROUND;

    Distance = 0;

    return Round;


    }
    

}

/**
 * @brief 半径为Radius的管子张开角度
 * 
 * @param Radius 管子半径 RADIUS是轮子半径
 * @return float 弧度制
 */

float Angle_Caculate_From_Radius(float Radius){

    float Span_Angle = 0.0f;

    Span_Angle = asin( TOTAL_LENGTH / (Radius - RADIUS) ) * 2;

    return Span_Angle;

}



