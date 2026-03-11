#ifndef __USR_PWM_H
#define __USR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
}
#endif     

#define MOTOL_PWM TIM_CHANNEL_4
#define MOTOR_PWM TIM_CHANNEL_2

void PWM_SetDutyCCR(uint32_t channel, uint32_t ccr);
	
#endif 
