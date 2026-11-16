#include "Dic_config.h"


/**
 * @brief 字典的定义
 * 
 */
    static char Variable_Assignment_List[][RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = { //二维数组字典定义 每条指令最多100字节

        "START",
        "STOP",

    };



/**
 * @brief 分析对应字典索引
 * 
 * @param size buffer缓冲区长度
 * 
 */

int Dictionary_Process_Index(int size){

    int nameflag = 0;

    int index = 0;

    char Command_String[RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];

    for(nameflag = 0; (USART1_Manage_Object->Rx_Buffer[nameflag] != '=') && (nameflag < size); nameflag++){

        Command_String[nameflag] = USART1_Manage_Object->Rx_Buffer[nameflag];
        
    }

    if(nameflag >= size) return -1; //没找到等号

    Command_String[nameflag] = '\0';

    for(index = 0; index < (sizeof(Variable_Assignment_List) / sizeof(Variable_Assignment_List[0])); ++index){ //与字典比对

        if(strcmp(Command_String ,Variable_Assignment_List[index]) == 0){

        return index;

        }

    }

    return -1; //未找到匹配的指令
}

/**
 * @brief 分离命令对应数值
 * 
 */

float Dictionary_Process_Value(int size){

    int nameflag = 0;

    int ValueStart = 0;

    int ValueFlag = 0;

    char Command_Value[RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {0};

    for(nameflag = 0; (USART1_Manage_Object->Rx_Buffer[nameflag] != '=') && (nameflag < size); nameflag++){
        

    }

    for(ValueStart = nameflag + 1; (USART1_Manage_Object->Rx_Buffer[ValueStart] != '#') && (ValueStart < size) ; ValueStart++){ //找出数值部分

        if(ValueFlag >= RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1) return -1;

        Command_Value[ValueFlag] = USART1_Manage_Object->Rx_Buffer[ValueStart];

        ValueFlag++;

    }

    if((ValueStart >= size) || ValueFlag == 0) return -1; //溢出或者没找到数值

    return strtof(Command_Value, NULL);

}


/**
 * @brief 对每种指令进行分析
 * 
 */

void Command_Processing(int size){

    int Index_Of_Command = 0;

    float Value_Get_From_Command = 0;

    Index_Of_Command = Dictionary_Process_Index(size);

    Value_Get_From_Command = Dictionary_Process_Value(size);

    switch (Index_Of_Command)
    {
    case (0):{

         HAL_TIM_Base_Start(&htim2);
         ifsend = 1;
         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

    }
    break;

    case (1):{

        HAL_TIM_Base_Stop(&htim2);
        ifsend = 0;

    }
    break;

    default:
    break;
    }

}
