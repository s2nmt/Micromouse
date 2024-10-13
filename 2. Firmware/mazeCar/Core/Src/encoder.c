
#include "encoder.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx.h"

#define ENCODER_COUNT 620 // number of encoder steps per wheel revolution

/*reset encoder count is halt of max uint16. this helps to prevent faulty vlaue on overflow*/
const int32_t reset_encoder_count = 65535 / 2;

int32_t interval;
int32_t count_difference;

uint64_t counterTim2 = 0;
uint64_t counterTim3 = 0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static float calculate_speed(int32_t count, int32_t interval);

uint8_t init_encoder(encoder_t *enc, wheel_t wheel)
{
	enc->wheel = wheel;
    enc->speed = 0;
    enc->position = 0;
    enc->previous_millis = 0;
    return 0;
}

static float calculate_speed(int32_t count, int32_t interval)
{
    // Calculate speed in units per minute
    double speed = (count / (double)ENCODER_COUNT) * (60000.0 / interval);
    return speed;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        counterTim2 = __HAL_TIM_GET_COUNTER(htim);
    }
    else if (htim->Instance == TIM3)
    {
        counterTim3 = __HAL_TIM_GET_COUNTER(htim);
    }
}
void update_encoder(encoder_t *enc)
{
    uint32_t current_millis;
    current_millis = HAL_GetTick();
    if (current_millis > enc->previous_millis)
    {
        interval = current_millis - enc->previous_millis;
    }
    if (enc->wheel == LEFT_WHEEL){
        count_difference = (int32_t)counterTim3 - reset_encoder_count;
        __HAL_TIM_SET_COUNTER(&htim3, 32767);
    }
    else {
        count_difference = (int32_t)counterTim2 - reset_encoder_count;
        __HAL_TIM_SET_COUNTER(&htim2, 32767);
    }
    /*poston sums driven distance */
    enc->position += (count_difference);
    if (count_difference == 0)
        enc->speed = 0.0;

    else
        enc->speed = calculate_speed(count_difference, interval);

    enc->previous_millis = current_millis;
}

float get_speed_enc(encoder_t *enc)
{
    return enc->speed;
}

int32_t get_position_enc(encoder_t *enc)
{
    return enc->position;
}

void reset_position_enc(encoder_t *enc)
{
    enc->position = 0;
}

void print_info_enc(encoder_t *enc)
{
    printf("speed: %5.1f Pos: %7ld  \t", enc->speed, enc->position);
}

