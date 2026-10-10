#include "Robot_test.h"

/**
 * @brief serialplot字典定义
 * 
 */

void Dictionary_Define(Class_Serialplot &serialplot){

    static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    //M3508电机调PID
    "po",
    "io",
    "do",
    "fo",
    "pa",
    "ia",
    "da",
    "to",
    "ta",

    //GM6020电机调PID
    "po2",
    "io2",
    "do2",
    "fo2",
    "pa2",
    "ia2",
    "da2",
    "to2",
    "ta2",
};

    serialplot.Init(&huart2, 18, (char **)Variable_Assignment_List);
    

}

/**
 * @brief serialplot回调函数配置
 * 
 * @param Buffer 串口接收数据缓存区
 * @param Length 串口接收数据长度
 * @param serialplot 串口绘图对象
 * @param motor M3508电机对象
 * @param motor2 GM6020电机对象
 */

void Robot_Test_Serialplot_Call_Back_Configuration(uint8_t *Buffer,
    uint16_t Length,
    Class_Serialplot &serialplot,
    Class_Motor_C620 &motor,
    Class_Motor_GM6020 &motor2,
    float *Robot_Target_Speed,
    float *Robot_Target_Distance,
    float *Rotate_Target_Speed,
    float *Rotate_Target_Angle){

    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        // M3508电机调PID

        case(0):
        {
            motor.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(1):
        {
            motor.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(2):
        {
            motor.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(3):
        {
            motor.PID_Omega.Set_K_F(serialplot.Get_Variable_Value());
        }
        break;
        case(4):
        {
            motor.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(5):
        {
            motor.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(6):
        {
            motor.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(7):
        {
            *Robot_Target_Speed = serialplot.Get_Variable_Value();
        }
        break;
        case(8):
        {
            *Robot_Target_Distance = serialplot.Get_Variable_Value();
        }
        break;



        //GM6020电机调PID
        case(9):
        {
            motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(10):
        {
            motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(11):
        {
            motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(12):
        {
            motor2.PID_Omega.Set_K_F(serialplot.Get_Variable_Value());
        }
        break;
        case(13):
        {
            motor2.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(14):
        {
            motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(15):
        {
            motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(16):
        {
            *Rotate_Target_Speed = serialplot.Get_Variable_Value();
        }
        break;
        case(17):
        {
            *Rotate_Target_Angle = serialplot.Get_Variable_Value();
        }
        break;
    }

}

/**
 * @brief 开启serilaplot绘图
 * 
 */

void Robot_Test_Serialplot_Plot(Class_Serialplot &serialplot , Class_Motor_C620 &motor , Class_Motor_GM6020 &motor2 , float Wheel_Omega_Ratio , float Wheel_Radius){

    float Robot_Target_Speed, Robot_Now_Speed,Rotate_Target_Speed, Rotate_Now_Speed;

    Robot_Target_Speed = motor.Get_Target_Omega() * Wheel_Radius / Wheel_Omega_Ratio;
    Rotate_Target_Speed = motor2.Get_Target_Omega() * RADS_TO_RPM;

    Robot_Now_Speed = Get_Robot_Now_Speed(motor, Wheel_Omega_Ratio , Wheel_Radius);
    Rotate_Now_Speed = Get_Rotate_Now_Speed(motor2);

    serialplot.Set_Data(4,&Robot_Target_Speed, &Robot_Now_Speed, &Rotate_Target_Speed, &Rotate_Now_Speed);
    serialplot.TIM_Write_PeriodElapsedCallback();

    TIM_UART_PeriodElapsedCallback();

}
