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

    //修改PID模式
     "3508modechange",
     "6020modechange",

     //M2006电机调PID
    "po3",
    "io3",
    "do3",
    "fo3",
    "pa3",
    "ia3",
    "da3",
    "to3",
    "ta3",

    //修改PID模式
    "M2006modechange",

    //停机
    "STOP",
    "stop3508",
    "stop6020",
    "stop2006front",
    "stop2006rear",

    //丝杆设置
    "initialize",
    "setzero",

    //串口发送控制
    "OperateSending",
    "DisableSending",

    //断连检测
    "Online",
    "FrontIncreaseForce",
    "RearIncreaseForce",

    //使能与失能L432KC测距
    "StartSynchronize",
    "StopSynchronize",

    //使能与使能实时建模 由于波特率限制 同时会关闭其他数据传回
    "StartSynchroDataSending",
    "StopSynchroDataSending",

};

    //命令条数由数组大小自动算出 原来手写46 增删命令时容易忘改
    serialplot.Init(&huart2, sizeof(Variable_Assignment_List) / sizeof(Variable_Assignment_List[0]), (char **)Variable_Assignment_List);
    

}

/**
 * @brief serialplot回调函数配置
 * 
 * @param Buffer 串口接收数据缓存区
 * @param Length 串口接收数据长度
 * @param serialplot 串口绘图对象
 * @param motor M3508电机对象
 * @param motor2 GM6020电机对象
 * @param motor3 M2006电机对象 虽然有两个 但写一个就行
 */

void Robot_Test_Serialplot_Call_Back_Configuration(uint8_t *Buffer,
    uint16_t Length,
    Class_Serialplot &serialplot,
    Class_Motor_C620 &motor,
    Class_Motor_GM6020 &motor2,
    Class_Motor_C610 &motor3,
    float *Robot_Target_Speed,
    float *Robot_Target_Distance,
    float *Rotate_Target_Speed,
    float *Rotate_Target_Angle){

    //在本次收到的数据后面补0 解析时就不会读到上一条指令留在缓冲区里的内容
    //否则一个噪声字节盖掉旧指令的第一个字符 剩下的部分可能被当成新指令再执行一遍
    if(Length < UART_BUFFER_SIZE){

        Buffer[Length] = 0;

    }

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
            // *Robot_Target_Speed = serialplot.Get_Variable_Value();
            Move_Forward_Speed_Control(serialplot.Get_Variable_Value(),MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);
        }
        break;
        case(8):
        {
            //*Robot_Target_Distance = serialplot.Get_Variable_Value();
            Robot_Move_Distance_Control(serialplot.Get_Variable_Value(),MOTOR_WHEEL_OMEGA_RATIO,RADIUS,motor);
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
            Rotate_Distance_Measurement_Speed_Control(serialplot.Get_Variable_Value() , motor2);
        }
        break;
        case(17):
        {
           //useless
        }
        break;
        case(18): // 3508modechange
        {
            if(serialplot.Get_Variable_Value() == 0){
                Motor_Set_Control_Method(M3508 , Control_Method_OMEGA);
            }
            else if(serialplot.Get_Variable_Value() == 1){
                Motor_Set_Control_Method(M3508 , Control_Method_ANGLE);
            }
        }
        break;
        case(19): // 6020modechange
        {
            if(serialplot.Get_Variable_Value() == 0){
                Motor_Set_Control_Method(GM6020 , Control_Method_OMEGA);
            }
            else if(serialplot.Get_Variable_Value() == 1){
                Motor_Set_Control_Method(GM6020 , Control_Method_ANGLE);
            }
        }
        break;
        case(20): // po3
        {
            motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(21): // io3
        {
            motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(22): // do3
        {
            motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(23): // fo3
        {
            motor3.PID_Omega.Set_K_F(serialplot.Get_Variable_Value());
        }
        break;
        case(24): // pa3
        {
            motor3.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(25): // ia3
        {
            motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(26): // da3
        {
            motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(27): // to3, rad/s
        {
            motor3.Set_Target_Omega(serialplot.Get_Variable_Value() * RPM_TO_RADS);
        }
        break;
        case(28): // ta3, rad
        {
            M2006_Rotate_Specified_Revolutions(serialplot.Get_Variable_Value(), motor3);
        }
        break;
        case(29): // M2006modechange
        {
            if(serialplot.Get_Variable_Value() == 0){
                Motor_Set_Control_Method(M2006_Front , Control_Method_OMEGA);
            }
            else if(serialplot.Get_Variable_Value() == 1){
                Motor_Set_Control_Method(M2006_Front , Control_Method_ANGLE);
            }
        }
        break;
        case(30):
        {

            Motor_Stop_Operation(M3508);
            Motor_Stop_Operation(GM6020);
            Motor_Stop_Operation(M2006_Front);
            Motor_Stop_Operation(M2006_Rear);

        }
        break;
        case(31):{

            Motor_Stop_Operation(M3508);

        }
        break;
        case(32):{

            Motor_Stop_Operation(GM6020);

        }
        break;
        case(33):{

             Motor_Stop_Operation(M2006_Front);

        }
        break;
        case(34):{

            Motor_Stop_Operation(M2006_Rear);

        }
        break;
        case(35): //initialize 已作废: 机械调零与管径自适应方案取消 字典里保留这条是为了不改变后面命令的编号
        case(36): //setzero 已作废 同上
        {

            //收到也不做任何事

        }
        break;
        case(37):{

            IfPlotSending = 1;

        }
        break;
        case(38):{

             IfPlotSending = 0;

        }
        break;
        case(39):{

            Robot_Online_Feed(); //上位机每5秒发一次 刷新心跳时刻

        }
        break;
        case(40):{

            Front_Screw_Adjust(serialplot.Get_Variable_Value());

        }
        break;
        case(41):{

            Rear_Screw_Adjust(serialplot.Get_Variable_Value());

        }
        break;
        case(42):{

            Start_Synchornize(); //开始测距
            IfPlotSending = 0;

        }
        break;
        case(43):{

            Stop_Synchornize(); //停止测距

        }
        break;
        case(44):{

            IfsychroDataSending = 1; //开启实时建模
            IfPlotSending = 0;

        }
        break;
        case(45):{

            IfsychroDataSending = 0; //关闭实时建模
            IfPlotSending = 1;

        }
        break;
    }

}

/**
 * @brief 开启serilaplot绘图
 *        每PLOT_PERIOD_MS发一帧 且只在上一帧发完后才填下一帧
 *        原来每个循环都重填发送缓冲区 而上一帧还在用DMA往外发 会把正在发的数据改掉
 * 
 */

void Robot_Test_Serialplot_Plot(Class_Serialplot &serialplot , Class_Motor_C620 &motor , Class_Motor_GM6020 &motor2 , Class_Motor_C610 &motor3 , Class_Motor_C610 &motor4 , float Wheel_Omega_Ratio , float Wheel_Radius){

    static uint32_t Last_Plot_Tick = 0;

    if(IfPlotSending && !IfsychroDataSending){

    if(HAL_GetTick() - Last_Plot_Tick < PLOT_PERIOD_MS || huart2.gState != HAL_UART_STATE_READY){

        return;

    }

    Last_Plot_Tick = HAL_GetTick();

    float Robot_Target_Speed, Robot_Now_Speed,Rotate_Target_Speed, Rotate_Now_Speed;

    Robot_Target_Speed = motor.Get_Target_Omega() * Wheel_Radius / Wheel_Omega_Ratio;
    Rotate_Target_Speed = motor2.Get_Target_Omega() * RADS_TO_RPM;

    Robot_Now_Speed = Get_Robot_Now_Speed(motor, Wheel_Omega_Ratio , Wheel_Radius);
    Rotate_Now_Speed = Get_Rotate_Now_Speed(motor2);

    //测距链路质量 每秒统计一次 用来判断433M模块带不带得动当前的数据量
    //CH6 每秒收到的采样数 正常应接近采样率1000
    //CH7 每秒丢失的采样数 由序号跳变算出 正常应接近0
    //CH8 累计校验错误次数
    static uint32_t Last_Link_Tick = 0 , Last_Samples_Ok = 0 , Last_Lost_Samples = 0;
    static float Link_Samples_Per_Second = 0.0f , Link_Lost_Per_Second = 0.0f , Link_Checksum_Errors = 0.0f;

    if(HAL_GetTick() - Last_Link_Tick >= 1000U){

        Struct_Scan_Statistics Statistics;

        Scan_Sync_Get_Statistics(&Statistics);

        Link_Samples_Per_Second = (float)(Statistics.Samples_Ok - Last_Samples_Ok);
        Link_Lost_Per_Second = (float)(Statistics.Lost_Samples - Last_Lost_Samples);
        Link_Checksum_Errors = (float)Statistics.Checksum_Errors;

        Last_Samples_Ok = Statistics.Samples_Ok;
        Last_Lost_Samples = Statistics.Lost_Samples;
        Last_Link_Tick = HAL_GetTick();

    }

    serialplot.Set_Data(8,&Robot_Target_Speed, &Robot_Now_Speed, &motor.TotalDistance , &Rotate_Target_Speed, &Rotate_Now_Speed , &Link_Samples_Per_Second , &Link_Lost_Per_Second , &Link_Checksum_Errors);
    serialplot.TIM_Write_PeriodElapsedCallback();

    TIM_UART_PeriodElapsedCallback();

    }

}

/**
 * @brief 测点处理 放在主循环 每个循环都要调用
 *        把同步模块配好的测点(序号, θ, ρ, x)取出来 开启实时建模时抽稀后回传上位机
 *
 *        回传帧格式(小端):
 *          字节0      帧头 0xAC
 *          字节1      本帧测点数 N
 *          之后N×12字节  每个测点: 序号uint32 θ uint16(0~65535为一圈) ρ uint16(微米 0为无效) x float32(米)
 *          最后1字节  校验 = 前面所有字节求和后按位取反
 *
 *        完整数据应写SD卡 这里的无线回传只用于实时预览
 * 
 */

void Robot_Test_Scan_Data_Processing(void){

    static uint8_t Pending[SCAN_UPLINK_RECORDS_PER_FRAME * sizeof(Struct_Scan_Record)]; //攒够一帧再发
    static uint8_t Pending_Count = 0;
    static uint8_t Decimation_Count = 0;

    Struct_Scan_Record Record;
    uint8_t Sending = (IfsychroDataSending && !IfPlotSending);

    //测点必须一直取走 否则缓冲区满了之后新的测点进不来
    while(Scan_Sync_Pop_Record(&Record)){

        if(!Sending || Pending_Count >= SCAN_UPLINK_RECORDS_PER_FRAME){

            continue; //没开回传 或上一帧还没发出去 丢弃

        }

        Decimation_Count++;

        if(Decimation_Count < SCAN_UPLINK_DECIMATION){

            continue;

        }

        Decimation_Count = 0;

        memcpy(&Pending[Pending_Count * sizeof(Struct_Scan_Record)], &Record, sizeof(Struct_Scan_Record));
        Pending_Count++;

    }

    if(!Sending){

        Pending_Count = 0;
        return;

    }

    if(Pending_Count >= SCAN_UPLINK_RECORDS_PER_FRAME && huart2.gState == HAL_UART_STATE_READY){

        uint8_t *Tx = UART2_Manage_Object.Tx_Buffer;
        uint16_t Length = 2 + Pending_Count * sizeof(Struct_Scan_Record);
        uint8_t Sum = 0;
        uint16_t i;

        Tx[0] = SCAN_UPLINK_FRAME_HEADER;
        Tx[1] = Pending_Count;
        memcpy(&Tx[2], Pending, Pending_Count * sizeof(Struct_Scan_Record));

        for(i = 0; i < Length; i++){

            Sum = (uint8_t)(Sum + Tx[i]);

        }

        Tx[Length] = (uint8_t)(~Sum);

        UART_Send_Data(&huart2, Tx, Length + 1);

        Pending_Count = 0;

    }

}
