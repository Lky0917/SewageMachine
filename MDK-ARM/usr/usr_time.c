#include "usr_time.h"
#include "usr_motor_ctrl.h"

#define USR_TIME_MS_PER_SEC 1000u /* 1 秒 = 1000 ms */

static volatile uint8_t s_active = 0;                      /* 是否正在计时 */
static volatile uint8_t s_expired = 0;                     /* 是否已到期 */
static volatile UsrTimeTask_t s_task = USR_TIME_TASK_NONE; /* 当前任务类型 */
static volatile uint32_t s_tick_ms = 0;                    /* 1ms 软计数器（由 TIM2 中断驱动） */
static uint32_t s_deadline = 0;                            /* 绝对到期时间（ms 计数） */
static void UsrTime_DefaultExpiredCallback(UsrTimeTask_t task);
static void (*s_expire_cb)(UsrTimeTask_t task) = UsrTime_DefaultExpiredCallback; /* 到期回调 */

static void UsrTime_DefaultExpiredCallback(UsrTimeTask_t task)
{
    (void)task;
    /* 默认行为：所有定时结束后停止电机 */
    MotorCtrl_StopAll();
}

static void UsrTime_StartMs(UsrTimeTask_t task, uint32_t duration_ms)
{
    /* 设置任务类型、激活并计算到期时间 */
    s_task = task;
    s_active = 1;
    s_expired = 0;
    s_deadline = s_tick_ms + duration_ms;
}

void UsrTime_Init(void)
{
    /* 清空软件定时器状态 */
    s_active = 0;
    s_expired = 0;
    s_task = USR_TIME_TASK_NONE;
    s_tick_ms = 0;
    s_deadline = 0;
    s_expire_cb = UsrTime_DefaultExpiredCallback;
}

void UsrTime_Tick(void)
{
    /* 1ms 节拍，必须由定时中断调用 */
    s_tick_ms++;
    if (!s_active)
    {
        return;
    }

    /* 到期判断：当前 tick 超过或等于 deadline */
    if ((int32_t)(s_tick_ms - s_deadline) >= 0)
    {
        s_active = 0;
        s_expired = 1;
        /* 仅置位标志，禁止在 ISR 中执行业务逻辑；
           回调由主循环 UsrTime_Poll() 负责调用，避免竞态条件 */
    }
}

void UsrTime_Start1MinTest(void)
{
    /* 60 秒测试任务 */
    UsrTime_StartMs(USR_TIME_TASK_1_MIN_TEST, 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start30Min(void)
{
    /* 30 分钟任务 */
    UsrTime_StartMs(USR_TIME_TASK_30_MIN, 30u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start1Hour(void)
{
    /* 1 小时任务 */
    UsrTime_StartMs(USR_TIME_TASK_1_HOUR, 60u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start2Hour(void)
{
    /* 2 小时任务 */
    UsrTime_StartMs(USR_TIME_TASK_2_HOUR, 2u * 60u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start3Hour(void)
{
    /* 3 小时任务 */
    UsrTime_StartMs(USR_TIME_TASK_3_HOUR, 3u * 60u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start5Hour(void)
{
    /* 5 小时任务 */
    UsrTime_StartMs(USR_TIME_TASK_5_HOUR, 5u * 60u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_Start8Hour(void)
{
    /* 8 小时任务 */
    UsrTime_StartMs(USR_TIME_TASK_8_HOUR, 8u * 60u * 60u * USR_TIME_MS_PER_SEC);
}

void UsrTime_StartSeconds(uint32_t seconds)
{
    /* 自定义秒数任务 */
    UsrTime_StartMs(USR_TIME_TASK_CUSTOM, seconds * USR_TIME_MS_PER_SEC);
}

void UsrTime_Stop(void)
{
    /* 停止并清空状态 */
    s_active = 0;
    s_expired = 0;
    s_task = USR_TIME_TASK_NONE;
    s_deadline = 0;
}

uint8_t UsrTime_IsActive(void)
{
    /* 是否存在运行中的定时任务 */
    return s_active;
}

uint8_t UsrTime_IsExpired(void)
{
    /* 是否已到期 */
    return s_expired;
}

uint8_t UsrTime_PopExpired(void)
{
    /* 获取并清除到期标志 */
    if (s_expired)
    {
        s_expired = 0;
        return 1;
    }
    return 0;
}

UsrTimeTask_t UsrTime_GetTask(void)
{
    /* 获取当前任务类型 */
    return s_task;
}

uint32_t UsrTime_GetRemainingSeconds(void)
{
    /* 获取剩余秒数（向上取整） */
    if (!s_active)
    {
        return 0;
    }

    uint32_t now = s_tick_ms;
    if ((int32_t)(s_deadline - now) <= 0)
    {
        return 0;
    }

    return (s_deadline - now + (USR_TIME_MS_PER_SEC - 1u)) / USR_TIME_MS_PER_SEC;
}

void UsrTime_SetExpiredCallback(void (*cb)(UsrTimeTask_t task))
{
    /* 设置到期回调；允许传 NULL 以清空回调 */
    s_expire_cb = cb;
}

void UsrTime_Poll(void)
{
    /* 在主循环中轮询到期标志，将回调执行移出 ISR 上下文，消除竞态条件 */
    if (s_expired != 0u)
    {
        UsrTimeTask_t task = s_task;
        s_expired = 0u; /* 先清标志，再执行回调，防止重入 */
        if (s_expire_cb != NULL)
        {
            s_expire_cb(task);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* TIM2 1ms 中断驱动软计时 */
        UsrTime_Tick();
    }
}
