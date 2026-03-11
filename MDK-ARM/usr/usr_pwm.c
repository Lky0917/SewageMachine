#include "usr_pwm.h"

extern TIM_HandleTypeDef htim1;

// ccr:0~1000
void PWM_SetDutyCCR(uint32_t channel, uint32_t ccr)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    if (ccr > arr) ccr = arr;

    __HAL_TIM_SET_COMPARE(&htim1, channel, ccr);
}