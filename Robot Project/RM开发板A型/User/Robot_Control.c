#include "Robot_Control.h"

//定义一下.h中的extern变量
float Robot_Move_Target_Delta_Distance = 0.0f;
float Robot_Move_Start_Angle = 0.0f;
float motor3_Target_Delta_Revolutions = 0.0f;
float motor3_Start_Angle = 0.0f;
float motor4_Target_Delta_Revolutions = 0.0f;
float motor4_Start_Angle = 0.0f;

//上电时不在调零过程中 只有执行调零命令后才进入ZERO_WAIT_PRESS
//原来上电就是ZERO_WAIT_PRESS 此时误碰开关会让丝杆电机直接转起来
Zero_State zero3_state = ZERO_DONE;
Zero_State zero4_state = ZERO_DONE;

//上位机心跳
static volatile uint32_t Last_Online_Tick = 0; //最近一次收到Online的时刻
static volatile uint8_t Online_Received = 0; //上电后是否收到过Online
static uint8_t Offline_Handled = 0; //本次断线是否已经处理过

//结构初始化(调零+卡紧)的状态
enum Structure_Init_State
{
    STRUCTURE_INIT_IDLE, //空闲
    STRUCTURE_INIT_REQUESTED, //已收到命令 等主循环启动
    STRUCTURE_INIT_ZEROING //调零中 等两个开关都完成
};

static volatile Structure_Init_State structure_init_state = STRUCTURE_INIT_IDLE;
static volatile float Structure_Init_Radius = 0.0f;
static volatile float Structure_Init_Extra = 0.0f;
static uint32_t Structure_Init_Start_Tick = 0;

/**
 * @brief 初始化所有配置
 * 
 */
void Robot_Control_Init(Class_Motor_C620 &motor, Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4){

    motor.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 5000.0f);
    motor.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 3591.0f / 187.0f); 
    //初始化为速度环

    motor2.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor2.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 3000.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 1.0f); 
    //初始化为速度环

    motor3.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor3.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 4000.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA,36.0f);
    //初始化为角度环
    
    motor4.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
    motor4.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 4000.0f);
    motor4.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA,36.0f);
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

    motor2.Set_Target_Omega(Rotate_Target_Speed * RPM_TO_RADS);

}

/**
 * @brief 控制M2006旋转指定圈数
 * 
 * @param Target_Revolutions 圈数 正负表示方向
 * @param motor 
 */

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

    float Rotate_Num = 0.0f;

    if(&motor == &motor3){ //前端电机拟合

        //M2006_Rotate_Specified_Revolutions的输入是圈数 内部会乘2π 这里不能再乘一次
        Rotate_Num = Angle_Round_Transfer(0.5f * Target_Angle , motor3);
        M2006_Rotate_Specified_Revolutions(Rotate_Num * MOTOR3_DERECTION_SYMBOL , motor3);

    }
    else if(&motor == &motor4){

        Rotate_Num = Angle_Round_Transfer(0.5f * Target_Angle , motor4);
        M2006_Rotate_Specified_Revolutions(Rotate_Num * MOTOR4_DERECTION_SYMBOL , motor4);

    }

 }

/**
 * @brief 电机停机函数 速度环下目标速度设0 角度环下目标角度设为当前角度
 *        PID一直在运行 所以停机后电机保持出力 竖直管道里不会下滑
 * 
 * @param name 
 */

void Motor_Stop_Operation(Motor_Name name){

    if(name == M3508){

        if(motor.Get_Control_Method() == Control_Method_ANGLE){

            motor.Set_Target_Angle(motor.Get_Now_Angle());

        }
        else{

            motor.Set_Target_Omega(0.0f);

        }

    }
    else if(name == GM6020){

        if(motor2.Get_Control_Method() == Control_Method_ANGLE){

            motor2.Set_Target_Angle(motor2.Get_Now_Angle());

        }
        else{

            motor2.Set_Target_Omega(0.0f);

        }

    }
    else if(name == M2006_Front){

        if(motor3.Get_Control_Method() == Control_Method_ANGLE){

            motor3.Set_Target_Angle(motor3.Get_Now_Angle());

        }
        else{

            motor3.Set_Target_Omega(0.0f);

        }

    }
    else if(name == M2006_Rear){

        if(motor4.Get_Control_Method() == Control_Method_ANGLE){

            motor4.Set_Target_Angle(motor4.Get_Now_Angle());

        }
        else{

            motor4.Set_Target_Omega(0.0f);

        }

    }

 }

/**
 * @brief 切换电机控制模式
 *        切到角度环: 目标角度设为当前角度 否则会朝着上一次遗留的目标角度转过去
 *        切到速度环: 目标速度设0 否则会沿用角度环算出来的中间速度
 *        同时清掉积分 防止带着旧积分进新模式
 * 
 * @param name 
 * @param method 
 */

void Motor_Set_Control_Method(Motor_Name name , Enum_Control_Method method){

    if(name == M3508){

        motor.Set_Control_Method(method);
        motor.Set_Target_Angle(motor.Get_Now_Angle());
        motor.Set_Target_Omega(0.0f);
        motor.PID_Angle.Set_Integral_Error(0.0f);
        motor.PID_Omega.Set_Integral_Error(0.0f);

    }
    else if(name == GM6020){

        motor2.Set_Control_Method(method);
        motor2.Set_Target_Angle(motor2.Get_Now_Angle());
        motor2.Set_Target_Omega(0.0f);
        motor2.PID_Angle.Set_Integral_Error(0.0f);
        motor2.PID_Omega.Set_Integral_Error(0.0f);

    }
    else if(name == M2006_Front){

        motor3.Set_Control_Method(method);
        motor3.Set_Target_Angle(motor3.Get_Now_Angle());
        motor3.Set_Target_Omega(0.0f);
        motor3.PID_Angle.Set_Integral_Error(0.0f);
        motor3.PID_Omega.Set_Integral_Error(0.0f);

    }
    else if(name == M2006_Rear){

        motor4.Set_Control_Method(method);
        motor4.Set_Target_Angle(motor4.Get_Now_Angle());
        motor4.Set_Target_Omega(0.0f);
        motor4.PID_Angle.Set_Integral_Error(0.0f);
        motor4.PID_Omega.Set_Integral_Error(0.0f);

    }

}


/**
 * @brief 前端丝杆调整 输入单位RPM
 * 
 */
void Front_Screw_Adjust(float RPM){

    motor3.Set_Target_Omega(MOTOR3_DERECTION_SYMBOL * RPM * RPM_TO_RADS);

}

/**
 * @brief 后端丝杆调整
 * 
 */
void Rear_Screw_Adjust(float RPM){

    motor4.Set_Target_Omega(MOTOR4_DERECTION_SYMBOL * RPM * RPM_TO_RADS);

}

/**
 * @brief 使能测距部分与机器人里程数同步
 * 
 */


void Start_Synchornize(void){

    static uint8_t command[] = "START=1#"; //L432KC字典定义

    UART_Send_Data(&huart6, command, sizeof(command) - 1);

}

/**
 * @brief 失能测距部分与机器人里程数同步
 * 
 */

void Stop_Synchornize(void){

    static uint8_t command[] = "STOP=1#"; //L432KC字典定义

    UART_Send_Data(&huart6, command, sizeof(command) - 1);


}
 


/**
 * @brief 驱动电机M3508的PID计算
 *        原来目标为0时不计算也不发CAN 电机处于无力状态 现在始终闭环
 *        原来每个电机算完都发一次CAN 一个周期最多发8帧 1ms内发不完 现在只算不发 由Motor_CAN_Send_Processing统一发
 * 
 */


void Forward_PID_Processing(Class_Motor_C620 &motor){

    motor.TIM_PID_PeriodElapsedCallback(); //PID计算 结果写入CAN发送缓冲区

}

/**
 * @brief 旋转电机GM6020的PID计算
 */

void Rotate_PID_Processing(Class_Motor_GM6020 &motor2){

    motor2.TIM_PID_PeriodElapsedCallback();

}

/**
 * @brief 丝杆调整电机M2006的PID计算
 * 
 */


void Angle_Adjusting_PID_Processing(Class_Motor_C610 &motor){

    motor.TIM_PID_PeriodElapsedCallback();

}

/**
 * @brief 把四个电机的控制量发到CAN总线 每个控制周期调用一次 放在所有PID计算之后
 *        0x200一帧带M3508和两个M2006 0x1fe一帧带GM6020 共两帧
 * 
 */

void Motor_CAN_Send_Processing(void){

    TIM_CAN_PeriodElapsedCallback();

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
 * @brief 获取机器人目前目标速度
 * 
 */

float Get_Robot_Target_Speed(){

    return (motor.Get_Target_Omega() * RADIUS / MOTOR_WHEEL_OMEGA_RATIO) ;

}

float Get_Robot_Target_Distance(){

    return (motor.TotalDistance + Robot_Move_Target_Delta_Distance);


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

            //测距舱转角每更新一次 就连同当时的里程存一条历史 供测距数据到达后回查
            Scan_Sync_Push_Pose(Robot_Get_Time_us(), (uint16_t)((Rx_Buffer->Data[0] << 8) | Rx_Buffer->Data[1]), motor.TotalDistance);

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

//PC0 为后端丝杆结构的调零开关
else if (GPIO_Pin == GPIO_PIN_0)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if (level == GPIO_PIN_SET && zero4_state == ZERO_WAIT_PRESS)
    {
        zero4_state = ZERO_BACKING;

        Motor_Back(motor4);

        // motor4 设置退出开关方向的速度
    }
    else if (level == GPIO_PIN_RESET && zero4_state == ZERO_BACKING) //原来这里判的是zero3_state 后端调零结束不了
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

    Motor_Set_Control_Method(M2006_Front , Control_Method_ANGLE); //切换回角度环并停在当前位置

    motor3.Front_Structure_Now_Angle = FRONT_ANGLE_MINIMUM; //设置零点
    motor3.Front_Structure_Now_Coordinate = 0;

    }
    else if(&motor == &motor4){

    Motor_Set_Control_Method(M2006_Rear , Control_Method_ANGLE); //切换回角度环并停在当前位置

    motor4.Rear_Structure_Now_Angle = REAR_ANGLE_MINIMUM; //设置零点 原来写的是Front_ 而Angle_Round_Transfer读的是Rear_
    motor4.Rear_Structure_Now_Coordinate = 0;


    }

}


/**
 * @brief 电机触点后退回 即前端电机运动负方向
 * 
 */

void Motor_Back(Class_Motor_C610 &motor){

    if(&motor == &motor3){

    motor3.Set_Target_Omega(-1.0f * MOTOR3_DERECTION_SYMBOL * BACKSPEED); //负方向速度 原来没有负号 触点后会继续往开关方向顶

    }
    else if(&motor == &motor4){

    motor4.Set_Target_Omega(-1.0f * MOTOR4_DERECTION_SYMBOL * BACKSPEED);

    }

}



/**
 * @brief 电机调零程序启动 执行程序
 * 
 */

void Operate_Motor_Set_Zero(Class_Motor_C610 &motor){

    if(&motor == &motor3){

    if(zero3_state == ZERO_DONE){ //判断是否为非调零中状态 调零时向下运动 为正方向

    Motor_Set_Control_Method(M2006_Front , Control_Method_OMEGA);
    motor3.Set_Target_Omega(MOTOR3_DERECTION_SYMBOL * SETZEROSPEED);

    zero3_state = ZERO_WAIT_PRESS;

         }
    }
    else if(&motor == &motor4){

    if(zero4_state == ZERO_DONE){ //判断是否为非调零中状态

    Motor_Set_Control_Method(M2006_Rear , Control_Method_OMEGA);
    motor4.Set_Target_Omega(MOTOR4_DERECTION_SYMBOL * SETZEROSPEED);

    zero4_state = ZERO_WAIT_PRESS;

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

    float Span_Angle = Angle_Caculate_From_Radius(Radius);

    if(Span_Angle < 0.0f){

        return; //管径超出机构能撑到的范围 不动作

    }

    if(&motor == &motor3){

        Screw_Structure_Angle_Set( Span_Angle + Extra , motor3);

    }
    else if(&motor == &motor4){

        Screw_Structure_Angle_Set( Span_Angle + Extra , motor4);

    }

}

/**
 * @brief 初始化机械结构 调零后卡紧管壁
 *        这里只登记请求 真正的动作在主循环的Structure_Init_Processing里做
 *        原来在这里HAL_Delay(15000) 而本函数是从串口中断里调用的
 *        SysTick优先级比串口中断低 中断里计时不走 HAL_Delay永远等不完 整机卡死
 * 
 * @param Radius 管道半径 单位米
 */
 
void Structure_Init(float Radius , float Extra){

    if(structure_init_state != STRUCTURE_INIT_IDLE){

        return; //上一次还没做完

    }

    Structure_Init_Radius = Radius;
    Structure_Init_Extra = Extra;
    structure_init_state = STRUCTURE_INIT_REQUESTED;

}

/**
 * @brief 结构初始化的执行 放在主循环
 *        启动调零 -> 等两个调零开关都完成 -> 按管径卡紧   超时则停机放弃
 * 
 */

void Structure_Init_Processing(void){

    if(structure_init_state == STRUCTURE_INIT_REQUESTED){

        Operate_Motor_Set_Zero(motor3);
        Operate_Motor_Set_Zero(motor4);

        Structure_Init_Start_Tick = HAL_GetTick();
        structure_init_state = STRUCTURE_INIT_ZEROING;

    }
    else if(structure_init_state == STRUCTURE_INIT_ZEROING){

        if(zero3_state == ZERO_DONE && zero4_state == ZERO_DONE){

            Auto_Fit_Operation(motor3 , Structure_Init_Radius , Structure_Init_Extra);
            Auto_Fit_Operation(motor4 , Structure_Init_Radius , Structure_Init_Extra);

            structure_init_state = STRUCTURE_INIT_IDLE;

        }
        else if(HAL_GetTick() - Structure_Init_Start_Tick > ZERO_TIMEOUT_MS){

            //开关一直没触发 停下丝杆电机 状态复位以便重试
            Motor_Stop_Operation(M2006_Front);
            Motor_Stop_Operation(M2006_Rear);

            zero3_state = ZERO_DONE;
            zero4_state = ZERO_DONE;

            structure_init_state = STRUCTURE_INIT_IDLE;

        }

    }

}


/**
 * @brief 转换角度与旋转圈数
 * 
 */

float Angle_Round_Transfer(float Angle , Class_Motor_C610 &motor){

    float Absolute_Coordinate = 0.0f , Distance , Round , Under_Root;

    if(&motor == &motor3){ //前端电机

    Under_Root = (FRONT_NODE_LENGTH)*(FRONT_NODE_LENGTH) - (SPRING_ORIGINAL_LENGTH * sinf(0.5f * Angle))*(SPRING_ORIGINAL_LENGTH * sinf(0.5f * Angle));

    if(Under_Root < 0.0f) return 0.0f; //该角度下几何无解 根号里为负会算出NaN并传给电机 这里直接不动

    Absolute_Coordinate = sqrtf(Under_Root) - SPRING_ORIGINAL_LENGTH * cosf(0.5f * Angle) - FRONT_ZERO_POINT;

    Distance = Absolute_Coordinate - motor.Front_Structure_Now_Coordinate; //表示螺母还要运动的距离

    motor.Front_Structure_Now_Coordinate += Distance; //累加坐标

    Round = Distance * DISTANCE_TO_ROUND;

    Distance = 0;

    return Round;

    }
    else if(&motor == &motor4){ //后端电机

    Under_Root = (REAR_NODE_LENGTH)*(REAR_NODE_LENGTH) - (SPRING_ORIGINAL_LENGTH * sinf(0.5f * Angle))*(SPRING_ORIGINAL_LENGTH * sinf(0.5f * Angle));

    if(Under_Root < 0.0f) return 0.0f;

    Absolute_Coordinate = sqrtf(Under_Root) - SPRING_ORIGINAL_LENGTH * cosf(0.5f * Angle) - REAR_ZERO_POINT;

    Distance = Absolute_Coordinate - motor.Rear_Structure_Now_Coordinate; //表示螺母还要运动的距离

    motor.Rear_Structure_Now_Coordinate += Distance; //累加坐标

    Round = Distance * DISTANCE_TO_ROUND;

    Distance = 0;

    return Round;


    }
    else{

        return 0.0f;

    }
    

}

/**
 * @brief 半径为Radius的管子张开角度
 * 
 * @param Radius 管子半径 RADIUS是轮子半径
 * @return float 弧度制 管径不合法时返回-1
 */

float Angle_Caculate_From_Radius(float Radius){

    float Span_Angle = 0.0f;
    float Ratio;

    if(Radius <= RADIUS){

        return -1.0f; //管道半径不大于轮子半径 分母为0或负

    }

    Ratio = TOTAL_LENGTH / (Radius - RADIUS);

    if(Ratio > 1.0f){

        return -1.0f; //asinf的输入超过1会算出NaN

    }

    Span_Angle = asinf(Ratio) * 2.0f;

    return Span_Angle;

}

/**
 * @brief 统计机器人总路程函数 放在systick中 速度积分
 * 
 */

// void Caculate_Total_Distance(){

//     float TICK = 0.001f;

//     if(motor.Get_Target_Omega() > 1e-5 | motor.Get_Target_Omega() < 1e-5){
    
//     motor.TotalDistance += Get_Robot_Now_Speed(motor,MOTOR_WHEEL_OMEGA_RATIO,RADIUS) * TICK;

//     }

// }

/**
 * @brief 收到上位机Online心跳时调用 在串口中断里
 * 
 */

void Robot_Online_Feed(void){

    Last_Online_Tick = HAL_GetTick();
    Online_Received = 1;

}

/**
 * @brief 机器人掉线判定 放在主循环
 *        超过OFFLINE_TIMEOUT_MS没收到心跳判定为断线 每次断线只处理一次 重新收到心跳后才允许再次触发
 *        原来断线后每3个循环(约3ms)就触发一次返程 每次都重新取起点 返程目标一直在变
 *        上电后从没收到过心跳时不判断线 这样台架上不连上位机调试时电机不会自己动
 * 
 */

void Robot_Offline_Detection(void){

    if(!Online_Received){

        return;

    }

    if(HAL_GetTick() - Last_Online_Tick < OFFLINE_TIMEOUT_MS){

        Offline_Handled = 0; //在线

    }
    else if(!Offline_Handled){

        Offline_Handled = 1;
        Handle_Robot_Offline(); //机器人断线处理

    }

}

/**
 * @brief 机器人掉线处理
 * 
 */

void Handle_Robot_Offline(void){

    Robot_Back(); //暂定原路返回

}

/**
 * @brief 机器人按原来里程返回(仅针对直管)
 * 
 */

void Robot_Back(void){

    Motor_Set_Control_Method(M3508 , Control_Method_ANGLE); //切到角度环并停在当前位置

    Robot_Move_Distance_Control(motor.TotalDistance * (-1.0f), MOTOR_WHEEL_OMEGA_RATIO, RADIUS, motor);

}

/**
 * @brief 机器人控制板复位
 * 
 */

void Robot_Control_Panel_Reset(void){

    HAL_NVIC_SystemReset();

}

/**
 * @brief 机器人打滑处理
 * 
 */

void Robot_Control_Deal_Slip(void){



}

/**
 * @brief 上电以来的微秒数 由SysTick的毫秒计数加上它当前的倒计数值得到
 *        约71分钟溢出回0 使用时只做减法求时间差即可
 *        可以在中断里调用: 如果SysTick中断已经挂起但还没来得及把毫秒数加1 这里会补上
 * 
 */

uint32_t Robot_Get_Time_us(void){

    uint32_t primask = __get_PRIMASK();
    uint32_t ms , value , load;

    __disable_irq();

    load = SysTick->LOAD;
    ms = HAL_GetTick();
    value = SysTick->VAL;

    if(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk){

        //倒计数刚刚重装 毫秒数还没加
        value = SysTick->VAL;
        ms++;

    }

    if(!primask){

        __enable_irq();

    }

    return ms * 1000U + (load - value) * 1000U / (load + 1U);

}
