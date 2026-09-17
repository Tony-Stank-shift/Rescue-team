#include "encoder.h"

#include "chassis_config.h"

#define ENCODER_SAMPLE_MS                  10U
#define LEFT_ENCODER_FORWARD_SIGN          1
#define RIGHT_ENCODER_FORWARD_SIGN         (-1)

static TIM_HandleTypeDef *left_encoder_timer;
static TIM_HandleTypeDef *right_encoder_timer;

static uint16_t left_last_count;
static uint16_t right_last_count;
static uint32_t encoder_last_tick;
static uint16_t left_encoder_count;
static uint16_t right_encoder_count;
static int16_t left_encoder_delta;
static int16_t right_encoder_delta;
static int32_t left_encoder_total;
static int32_t right_encoder_total;
static float left_counts_per_second;
static float right_counts_per_second;
static float left_rpm;
static float right_rpm;
static float sample_period_seconds = 0.010f;

HAL_StatusTypeDef Encoder_Init(TIM_HandleTypeDef *left_timer,
                               TIM_HandleTypeDef *right_timer)
{
    if ((left_timer == NULL) || (right_timer == NULL))
    {
        return HAL_ERROR;
    }

    left_encoder_timer = left_timer;
    right_encoder_timer = right_timer;
    if (HAL_TIM_Encoder_Start(left_encoder_timer, TIM_CHANNEL_ALL) != HAL_OK)
    {
        left_encoder_timer = NULL;
        right_encoder_timer = NULL;
        return HAL_ERROR;
    }
    if (HAL_TIM_Encoder_Start(right_encoder_timer, TIM_CHANNEL_ALL) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(left_encoder_timer, TIM_CHANNEL_ALL);
        left_encoder_timer = NULL;
        right_encoder_timer = NULL;
        return HAL_ERROR;
    }

    left_last_count = (uint16_t)__HAL_TIM_GET_COUNTER(left_encoder_timer);
    right_last_count = (uint16_t)__HAL_TIM_GET_COUNTER(right_encoder_timer);
    left_encoder_count = left_last_count;
    right_encoder_count = right_last_count;
    left_encoder_delta = 0;
    right_encoder_delta = 0;
    left_encoder_total = 0;
    right_encoder_total = 0;
    left_counts_per_second = 0.0f;
    right_counts_per_second = 0.0f;
    left_rpm = 0.0f;
    right_rpm = 0.0f;
    sample_period_seconds = (float)ENCODER_SAMPLE_MS / 1000.0f;
    encoder_last_tick = HAL_GetTick();
    return HAL_OK;
}

bool Encoder_Update(void)
{
    uint32_t now;
    uint32_t elapsed_ms;
    int16_t left_raw_delta;
    int16_t right_raw_delta;

    if ((left_encoder_timer == NULL) || (right_encoder_timer == NULL))
    {
        return false;
    }

    now = HAL_GetTick();
    elapsed_ms = now - encoder_last_tick;
    if (elapsed_ms < ENCODER_SAMPLE_MS)
    {
        return false;
    }
    encoder_last_tick = now;
    sample_period_seconds = (float)elapsed_ms / 1000.0f;

    left_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(left_encoder_timer);
    right_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(right_encoder_timer);

    /* int16_t 差值自动处理 16 位定时器在 0/65535 处的回绕。 */
    left_raw_delta =
        (int16_t)((uint16_t)(left_encoder_count - left_last_count));
    right_raw_delta =
        (int16_t)((uint16_t)(right_encoder_count - right_last_count));
    left_last_count = left_encoder_count;
    right_last_count = right_encoder_count;

    left_encoder_delta =
        (int16_t)(left_raw_delta * LEFT_ENCODER_FORWARD_SIGN);
    right_encoder_delta =
        (int16_t)(right_raw_delta * RIGHT_ENCODER_FORWARD_SIGN);
    left_encoder_total += left_encoder_delta;
    right_encoder_total += right_encoder_delta;

    left_counts_per_second =
        (float)left_encoder_delta / sample_period_seconds;
    right_counts_per_second =
        (float)right_encoder_delta / sample_period_seconds;
    left_rpm = left_counts_per_second * 60.0f /
        CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
    right_rpm = right_counts_per_second * 60.0f /
        CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
    return true;
}

void Encoder_Reset(void)
{
    if ((left_encoder_timer != NULL) && (right_encoder_timer != NULL))
    {
        left_last_count =
            (uint16_t)__HAL_TIM_GET_COUNTER(left_encoder_timer);
        right_last_count =
            (uint16_t)__HAL_TIM_GET_COUNTER(right_encoder_timer);
        left_encoder_count = left_last_count;
        right_encoder_count = right_last_count;
    }

    left_encoder_delta = 0;
    right_encoder_delta = 0;
    left_encoder_total = 0;
    right_encoder_total = 0;
    left_counts_per_second = 0.0f;
    right_counts_per_second = 0.0f;
    left_rpm = 0.0f;
    right_rpm = 0.0f;
    sample_period_seconds = (float)ENCODER_SAMPLE_MS / 1000.0f;
    encoder_last_tick = HAL_GetTick();
}

float Encoder_GetLeftRpm(void) { return left_rpm; }
float Encoder_GetRightRpm(void) { return right_rpm; }
float Encoder_GetLeftCountsPerSecond(void) { return left_counts_per_second; }
float Encoder_GetRightCountsPerSecond(void) { return right_counts_per_second; }
float Encoder_GetSamplePeriodSeconds(void) { return sample_period_seconds; }
int16_t Encoder_GetLeftDelta(void) { return left_encoder_delta; }
int16_t Encoder_GetRightDelta(void) { return right_encoder_delta; }
int32_t Encoder_GetLeftTotal(void) { return left_encoder_total; }
int32_t Encoder_GetRightTotal(void) { return right_encoder_total; }
uint16_t Encoder_GetLeftCounter(void) { return left_encoder_count; }
uint16_t Encoder_GetRightCounter(void) { return right_encoder_count; }
