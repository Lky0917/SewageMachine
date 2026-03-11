#ifndef __USR_TIME_H__
#define __USR_TIME_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
}
#endif
/* user code begin here */

typedef enum
{
    /**< 无任务 */
    USR_TIME_TASK_NONE = 0,
    /**< 1 分钟测试任务 */
    USR_TIME_TASK_1_MIN_TEST,
    /**< 30 分钟任务 */
    USR_TIME_TASK_30_MIN,
    /**< 1 小时任务 */
    USR_TIME_TASK_1_HOUR,
    /**< 2 小时任务 */
    USR_TIME_TASK_2_HOUR,
    /**< 3 小时任务 */
    USR_TIME_TASK_3_HOUR,
    /**< 5 小时任务 */
    USR_TIME_TASK_5_HOUR,
    /**< 8 小时任务 */
    USR_TIME_TASK_8_HOUR,
    /**< 自定义秒任务 */
    USR_TIME_TASK_CUSTOM
} UsrTimeTask_t;

/**
 * @brief 初始化定时任务模块状态（不启动计时）。
 *
 * @note 仅初始化软件状态，不包含外设定时器初始化。
 */
void UsrTime_Init(void);

/**
 * @brief 1ms 软定时节拍。
 *
 * @note 需在 1ms 周期中断或等价节拍中调用一次。
 */
void UsrTime_Tick(void);

/**
 * @brief 启动 1 分钟测试定时任务。
 */
void UsrTime_Start1MinTest(void);
/**
 * @brief 启动 30 分钟定时任务。
 */
void UsrTime_Start30Min(void);
/**
 * @brief 启动 1 小时定时任务。
 */
void UsrTime_Start1Hour(void);
/**
 * @brief 启动 2 小时定时任务。
 */
void UsrTime_Start2Hour(void);
/**
 * @brief 启动 3 小时定时任务。
 */
void UsrTime_Start3Hour(void);
/**
 * @brief 启动 5 小时定时任务。
 */
void UsrTime_Start5Hour(void);
/**
 * @brief 启动 8 小时定时任务。
 */
void UsrTime_Start8Hour(void);
/**
 * @brief 启动自定义秒数定时任务。
 *
 * @param seconds 定时时长（秒）。
 */
void UsrTime_StartSeconds(uint32_t seconds);

/**
 * @brief 停止当前定时任务并清空状态。
 */
void UsrTime_Stop(void);
/**
 * @brief 是否存在正在运行的定时任务。
 *
 * @return 1 表示正在计时，0 表示空闲。
 */
uint8_t UsrTime_IsActive(void);
/**
 * @brief 定时任务是否已到期（到期后会保持为 1，直到重启/停止）。
 *
 * @return 1 表示已到期，0 表示未到期。
 */
uint8_t UsrTime_IsExpired(void);
/**
 * @brief 获取并清除到期标志。
 *
 * @return 1 表示本次检测到到期，0 表示未到期。
 */
uint8_t UsrTime_PopExpired(void);
/**
 * @brief 获取当前任务类型。
 *
 * @return 当前任务枚举。
 */
UsrTimeTask_t UsrTime_GetTask(void);
/**
 * @brief 获取剩余秒数。
 *
 * @return 剩余秒数；未运行返回 0。
 */
uint32_t UsrTime_GetRemainingSeconds(void);
/**
 * @brief 设置到期回调函数。
 *
 * @param cb 回调函数指针。传 NULL 表示清除回调。
 */
void UsrTime_SetExpiredCallback(void (*cb)(UsrTimeTask_t task));
/**
 * @brief 主循环轮询函数：检测到期标志并在主循环上下文中执行回调。
 * @note  必须在主循环中周期性调用；禁止在中断中调用。
 */
void UsrTime_Poll(void);

/* user code end here */

#endif
