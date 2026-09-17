#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

HAL_StatusTypeDef Encoder_Init(TIM_HandleTypeDef *left_timer,
                               TIM_HandleTypeDef *right_timer);

/* 每 10 ms 更新一次；产生新样本时返回 true。 */
bool Encoder_Update(void);

/* 以当前计数器位置为新零点，同时清零累计值和速度样本。 */
void Encoder_Reset(void);

/* 所有速度、增量和累计值均统一为“车轮向前为正”。 */
float Encoder_GetLeftRpm(void);
float Encoder_GetRightRpm(void);
float Encoder_GetLeftCountsPerSecond(void);
float Encoder_GetRightCountsPerSecond(void);
float Encoder_GetSamplePeriodSeconds(void);
int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);
int32_t Encoder_GetLeftTotal(void);
int32_t Encoder_GetRightTotal(void);
uint16_t Encoder_GetLeftCounter(void);
uint16_t Encoder_GetRightCounter(void);

#endif /* ENCODER_H */
