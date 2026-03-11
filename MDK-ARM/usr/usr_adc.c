#include "usr_adc.h"

uint16_t adc_dma_buf[ADC_DMA_LEN];                /* ADC DMA 原始数据缓存 */
uint16_t adc_10x6[ADC_SAMPLE_GROUPS][ADC_CH_NUM]; /* 解包后的 ADC 数据缓存 */
uint16_t adc_avg[ADC_CH_NUM];                     /* 各通道平均值缓存 */
uint16_t adc_voltage_mv[6];                       /* 各通道电压值缓存（单位：毫伏） */
uint8_t ADC_on_off = 0;                           /* ADC 打印开关 */

volatile uint8_t adc_frame_ready = 0; /* ADC DMA 完成标志 */

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/**
 * @brief ADC 转换完成回调函数
 *
 * @param hadc ADC句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_frame_ready = 1;
    }
}

/**
 * @brief 解包并计算 ADC 数据的平均值
 *
 */
static void adc_unpack_and_average(void)
{
    for (uint8_t i = 0; i < ADC_SAMPLE_GROUPS; i++)
    {
        for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++)
        {
            adc_10x6[i][ch] = adc_dma_buf[i * ADC_CH_NUM + ch];
        }
    }

    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++)
    {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < ADC_SAMPLE_GROUPS; i++)
        {
            sum += adc_10x6[i][ch];
        }
        adc_avg[ch] = (uint16_t)(sum / ADC_SAMPLE_GROUPS);
    }
}

/**
 * @brief 计算 ADC 电压值（单位：毫伏）
 *
 */
static void adc_calc_voltage_mv(void)
{
    for (uint8_t ch = 0; ch < 6; ch++)
    {
        adc_voltage_mv[ch] =
            (uint16_t)((uint32_t)adc_avg[ch] * ADC_REF_MV / ADC_MAX_VALUE);
    }
}
/**
 * @brief 从输出电压计算霍尔传感器电流
 *
 * @param vout_mV 输出电压（毫伏）
 * @param hall_range_A 霍尔传感器量程（安培）
 * @return float 计算出的电流值（安培）
 */
static float hall_current_from_mV(float vout_mV, float hall_range_A)
{
    const float sens_mV_per_A = 1650.0f / hall_range_A;

    return (vout_mV - 1650.0f) / sens_mV_per_A;
}

/**
 * @brief 从输出电压计算分流电阻电流
 *
 * @param vout_mV   输出电压（毫伏）
 * @return float    计算出的电流值（安培）
 */
static float current_from_shunt_mV(float vout_mV)
{
    // R = 39 mO, Gain = 15
    // I = V / (R * Gain)
    return (vout_mV / 1000.0f) / 0.585f;
}

/* 获取左行走电机电流 */
float Adc_GetLeftMotorCurrent(void)
{
    return hall_current_from_mV(adc_voltage_mv[5], 5.0f);
}
/* 获取右行走电机电流 */
float Adc_GetRightMotorCurrent(void)
{
    return hall_current_from_mV(adc_voltage_mv[0], 5.0f);
}
/* 获取抽水电机3电流 */
float Adc_GetMotor3Current(void)
{
    return current_from_shunt_mV(adc_voltage_mv[4]);
}
/* 获取抽水电机4电流 */
float Adc_GetMotor4Current(void)
{
    return current_from_shunt_mV(adc_voltage_mv[1]);
}

/* 获取电池电压 */
float Adc_GetBatteryVoltage(void)
{
    return adc_voltage_mv[2] * 11.0f / 1000.0f;
}

/**
 * @brief 打印 ADC 电压值
 *
 */
static void adc_printf_voltage(void)
{
    printf("ADC:");
    printf("VbatV:%2.2fV\t", adc_voltage_mv[2] * 11.0f / 1000.0f);            // 电池电压
    printf("VbatC:%2.2fA\t", hall_current_from_mV(adc_voltage_mv[3], 20.0f)); // 电池电流
    printf("LmotoC:%2.2fA\t", hall_current_from_mV(adc_voltage_mv[5], 5.0f)); // 左电机电流
    printf("RmotoC:%2.2fA\t", hall_current_from_mV(adc_voltage_mv[0], 5.0f)); // 右电机电流
    printf("Moto3:%2.2fA\t", current_from_shunt_mV(adc_voltage_mv[4]));       // 电机3电流
    printf("Moto4:%2.2fA\t", current_from_shunt_mV(adc_voltage_mv[1]));       // 电机4电流
    printf("\r\n");
}

void adc_average_thread(void)
{
    if (adc_frame_ready)
    {
        adc_frame_ready = 0;
        adc_unpack_and_average();
        adc_calc_voltage_mv();
        if (ADC_on_off == 1)
            adc_printf_voltage();
    }
}
