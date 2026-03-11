#ifndef __USR_ADC_H
#define __USR_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>

#define ADC_CH_NUM 6
#define ADC_SAMPLE_GROUPS 50
#define ADC_DMA_LEN (ADC_CH_NUM * ADC_SAMPLE_GROUPS)
#define ADC_MAX_VALUE 4095
#define ADC_REF_MV 3300

#ifdef __cplusplus
}
#endif

extern uint16_t adc_dma_buf[];

void adc_average_thread(void);
/* 电机电流读取接口（单位：A） */
float Adc_GetLeftMotorCurrent(void);
float Adc_GetRightMotorCurrent(void);
float Adc_GetMotor3Current(void);
float Adc_GetMotor4Current(void);
/* 电池电压读取接口（单位：V） */
float Adc_GetBatteryVoltage(void);

#endif
