#include "bt_tuya.h"
#include "usr_motor_ctrl.h"

uint8_t g_power_switch = 0;         // 初始电源开关
uint8_t g_clean_switch = 0;         // 初始清扫开关
uint8_t g_work_mode = 0;            // 初始工作模式
uint8_t g_work_status = 0;          // 初始工作状态
uint8_t g_battery_percentage = 100; // 初始剩余电量
uint8_t g_clean_time = 0;           // 初始清扫时间
uint8_t g_forward = 0;              // 初始前进状态
uint8_t g_backward = 0;             // 初始后退状态
uint8_t g_left = 0;                 // 初始向左清扫状态
uint8_t g_right = 0;                // 初始向右清扫状态
uint8_t g_warning_info = 0;         // 初始报警信息
uint8_t g_auto_enable = 0;          // 自动模式使能标志
uint8_t g_wall_enable = 0;          // 爬墙模式使能标志

void bt_state_poll_task(void)
{
    static unsigned char last_state = 0xFF;
    unsigned char cur = mcu_get_bt_work_state();

    if (cur != last_state)
    {
        last_state = cur;

        // printf("bt statu : %d\r\n",last_state);
        if (cur == 2)
        {
            printf("bt phone on-line\r\n");
        }
        else if (cur == 1)
        {
            printf("bt phone off-line\r\n");
        }
    }
}
