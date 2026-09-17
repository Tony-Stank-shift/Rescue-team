#ifndef COMMAND_H
#define COMMAND_H

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef Command_Init(UART_HandleTypeDef *uart);
void Command_Update(void);

#endif /* COMMAND_H */
