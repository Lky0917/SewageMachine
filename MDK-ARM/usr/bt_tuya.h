#ifndef __BT_TUYA_H
#define __BT_TUYA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "mcu_api.h"
#include "protocol.h"
#include "system.h"

#ifdef __cplusplus
}
#endif

extern uint8_t g_power_switch;
extern uint8_t g_clean_switch;
extern uint8_t g_work_mode;
extern uint8_t g_work_status;
extern uint8_t g_battery_percentage;
extern uint8_t g_clean_time;
extern uint8_t g_forward;
extern uint8_t g_backward;
extern uint8_t g_left;
extern uint8_t g_right;
extern uint8_t g_warning_info;
extern unsigned long g_dev_worning;
extern uint8_t g_auto_enable;
extern uint8_t g_wall_enable;

/* 工作状态枚举值（需与涂鸦面板枚举顺序一致） */
#define STATUS_STANDBY 0     // standby 待机中
#define STATUS_SMART_CLEAN 1 // smart_clean 自动清扫中
#define STATUS_WALL_CLEAN 2  // wall_clean 沿墙清扫中
#define STATUS_PAUSED 7      // paused 暂停

void bt_state_poll_task(void);

#endif
