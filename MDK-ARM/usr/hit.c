#include "hit.h"
#include "bt_tuya.h"
#include "usr_motor_ctrl.h"

extern uint8_t g_work_mode;

/* 预留的历史电平变量（当前未使用） */
// static uint8_t last_level = 0x0F;

/**
 * @brief 读取 HIT 输入脚并决定工作模式。
 *
 * @details
 * 优先级：IN1 -> IN2 -> IN3 -> IN4，否则为默认模式 4。
 * 当模式变化时更新 g_work_mode 并触发全量 DP 上报。
 */
void gpio_work_mode_scan(void)
{
    static uint8_t last_work_mode = 0xFF;
    uint8_t new_work_mode;

    /* 读取输入脚电平（低电平表示触发）。 */
    uint8_t in1 = (HAL_GPIO_ReadPin(HIT_IN1_GPIO_Port, HIT_IN1_Pin) == GPIO_PIN_SET);
    uint8_t in2 = (HAL_GPIO_ReadPin(HIT_IN2_GPIO_Port, HIT_IN2_Pin) == GPIO_PIN_SET);
    uint8_t in3 = (HAL_GPIO_ReadPin(HIT_IN3_GPIO_Port, HIT_IN3_Pin) == GPIO_PIN_SET);
    uint8_t in4 = (HAL_GPIO_ReadPin(HIT_IN4_GPIO_Port, HIT_IN4_Pin) == GPIO_PIN_SET);

    if (in1 == 0)
    {
        new_work_mode = 0; /* 模式 0 */
    }
    else if (in2 == 0)
    {
        new_work_mode = 1; /* 模式 1 */
    }
    else if (in3 == 0)
    {
        new_work_mode = 2; /* 模式 2 */
    }
    else if (in4 == 0)
    {
        new_work_mode = 3; /* 模式 3 */
    }
    else
    {
        new_work_mode = 4; /* 默认模式 4 */
    }

    /* 模式变化时才更新并上报，避免重复发送。 */
    if (new_work_mode != last_work_mode)
    {
        last_work_mode = new_work_mode; /* 更新上次模式 */
        g_work_mode = new_work_mode;    /* 更新全局工作模式 */
        all_data_update();              /* 触发全量 DP 上报 */
    }
}
