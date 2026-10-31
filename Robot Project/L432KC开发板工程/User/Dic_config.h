#ifndef __DIC_CONFIG_H__
#define __DIC_CONFIG_H__

#include "string.h"
#include "stdlib.h"
#include "UART_config.h"

#define RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 100


int Dictionary_Process_Index(int size);

float Dictionary_Process_Value(int size);

void Command_Processing(int size);

#endif
