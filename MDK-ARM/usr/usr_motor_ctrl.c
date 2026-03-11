#include "usr_motor_ctrl.h"
#include "usr_pwm.h"
#include "main.h"
#include "usr_adc.h"
#include "usr_system.h"
#include "usr_time.h"
#include "bt_tuya.h"
#include <float.h>
#include "qmi8658.h"
#include "imu.h"
#include "hit.h"
#include <stdio.h>

/*****************************碰撞逻辑*************************************************
 *  左右行走电机：正转=前进，反转=后退（如果你硬件相反，方向反过来即可）
 *  退避统一 2.0s
 *  转向时间可先用 0.8s 试跑，再按实际角度微调
 *  触发后执行“模式0~3”（你定义的四个碰撞模式）
 *  模式0：前碰撞（HIT_IN1，前）

    动作：后退 2.0s（左右电机同向“后退”）
    转向：向左转 0.8s（右轮前进、左轮后退，原地左转）
    结束：恢复正常模式
    模式1：后碰撞（HIT_IN2，后）

    动作：前进 2.0s（左右电机同向“前进”）
    转向：向右转 0.8s（右轮后退、左轮前进，原地右转）
    结束：恢复正常模式
    模式2：左碰撞（HIT_IN3，左）

    动作：后退 1.5s（左右电机同向“后退”）
    转向：向右转 0.6s（右轮后退、左轮前进）
    结束：恢复正常模式
    模式3：右碰撞（HIT_IN4，右）

    动作：后退 1.5s（左右电机同向“后退”）
    转向：向左转 0.6s（右轮前进、左轮后退）
    结束：恢复正常模式
    可选优化（后续再加）：

    连续触发同侧时，转向时间逐步加大（如 +0.2s）
    前碰撞后转向方向交替（左/右轮换），避免陷在死角
    退避期间屏蔽其他外部指令
 *****************************************************************************/
static float s_acc[3] = {0};
static float s_gyro[3] = {0};
static float s_rpy[3] = {0};
static uint8_t s_hit_front = 0;
static uint8_t s_hit_back = 0;
static uint8_t s_hit_left = 0;
static uint8_t s_hit_right = 0;

volatile uint8_t g_motor_selftest_busy = 0; // 1=自检中，0=空闲

extern uint8_t g_auto_enable;                   // 自动模式使能标志
extern uint8_t g_wall_enable;                   // 爬壁模式使能标志
extern volatile unsigned char system_flag_bool; // 系统开关状态

/* 电机控制内部状态 */
typedef struct
{
    MotorCtrlMode_t mode; /* 当前模式 */
    uint16_t pwm;         /* 当前 PWM 占空比 */
    uint32_t end_tick;    /* 动作结束时间 */
} MotorCtrlState_t;

/* 自检状态 */
typedef enum
{
    SELFTEST_IDLE = 0,
    SELFTEST_M34,       /* MOTO3/MOTO4 使能 15s */
    SELFTEST_LR_STAGE1, /* 左正右反 10s */
    SELFTEST_LR_STAGE2  /* 左反右正 10s */
} MotorSelfTestState_t;
/* 自检控制结构体 */
typedef struct
{
    uint8_t active;             // 是否在自检中
    MotorSelfTestState_t state; // 自检状态
    uint16_t pwm;               // 自检使用的 PWM 占空比
    uint32_t deadline;          // 自检截止时间
} MotorSelfTestCtrl_t;

/* 自检统计 */
typedef struct
{
    float left_min;     // 左电机最小值
    float left_max;     // 左电机最大值
    float right_min;    // 右电机最小值
    float right_max;    // 右电机最大值
    float m3_min;       // MOTO3最小值
    float m3_max;       // MOTO3最大值
    float m4_min;       // MOTO4最小值
    float m4_max;       // MOTO4最大值
} MotorSelfTestStats_t; // 自检统计

/* 自检统计信息 */
static MotorSelfTestStats_t s_selftest_stats;
static MotorCtrlState_t s_ctrl = {MOTOR_CTRL_STOP, 0, 0};
static MotorSelfTestCtrl_t s_selftest = {0, SELFTEST_IDLE, 0, 0};

/* 碰撞检测状态 */
typedef enum
{
    COLLISION_IDLE = 0,
    COLLISION_BACKOFF, // 退避阶段
    COLLISION_TURN     // 转向阶段
} MotorCollisionState_t;

/* 碰撞检测控制结构体 */
typedef struct
{
    uint8_t active;
    uint8_t mode; // 0~3 对应前/后/左/右
    MotorCollisionState_t state;
    uint16_t pwm;
    uint32_t deadline;
} MotorCollisionCtrl_t;

/* 自动行走状态 */
typedef enum
{
    AUTO_IDLE = 0,
    AUTO_DRIVE,
    AUTO_TURN_1,
    AUTO_FORWARD_SHORT,
    AUTO_TURN_2
} MotorAutoState_t;
/* 自动行走控制结构体 */
typedef struct
{
    uint8_t active;
    uint8_t turn_dir; /* 0=左转，1=右转 */
    MotorAutoState_t state;
    uint16_t pwm;
    uint32_t deadline;
    float start_yaw;
    uint8_t climbing;
} MotorAutoCtrl_t;

static MotorAutoCtrl_t s_auto = {0, 0, AUTO_IDLE, 0, 0, 0};

/* 自动序列状态 */
typedef enum
{
    AUTO_SEQ_WAIT_SELFTEST = 0,
    AUTO_SEQ_WAIT_AFTER_SELFTEST,
    AUTO_SEQ_FORWARD_LONG,
    AUTO_SEQ_BACKOFF,
    AUTO_SEQ_TURN_1,
    AUTO_SEQ_FORWARD_SHORT,
    AUTO_SEQ_TURN_2,
} AutoSeqState_t;

typedef struct
{
    AutoSeqState_t state;
    uint32_t deadline;
} AutoSeqCtrl_t;

static AutoSeqCtrl_t s_auto_seq = {AUTO_SEQ_WAIT_SELFTEST, 0};

/* 一个身位后退和 90 度转向时间（ms，按实机调） */
#define AUTO_TURN_MS 800
#define AUTO_FORWARD_SHORT_MS 1000
#define AUTO_DRIVE_MAX_MS (180u * 1000u)

/* 碰撞检测控制 */
static MotorCollisionCtrl_t s_collision = {0, 0, COLLISION_IDLE, 0, 0};
static uint8_t s_wall_climb = 0; // 墙体爬升标志

/**
 * @brief 设置左右轮方向。
 *
 * @details
 * 方向电平与实际正反向的对应关系与硬件有关，可后续调整。
 */
static void MotorCtrl_SetDir(uint8_t right_dir, uint8_t left_dir)
{
    HAL_GPIO_WritePin(MOTO1_DIR_GPIO_Port, MOTO1_DIR_Pin, (GPIO_PinState)right_dir); // 右轮方向
    HAL_GPIO_WritePin(MOTO2_DIR_GPIO_Port, MOTO2_DIR_Pin, (GPIO_PinState)left_dir);  // 左轮方向
}

/**
 * @brief MOTO3/MOTO4 使能控制（低电平有效）。
 */

static void MotorCtrl_SetAuxEnable(uint8_t enable)
{
    GPIO_PinState level = enable ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(MOTO3_EN_GPIO_Port, MOTO3_EN_Pin, level);
    HAL_GPIO_WritePin(MOTO4_EN_GPIO_Port, MOTO4_EN_Pin, level);
}

/**
 * @brief 更新最小值和最大值。(自检)
 *
 * @param value
 * @param min_v
 * @param max_v
 */
static void MotorSelfTest_UpdateMinMax(float value, float *min_v, float *max_v)
{
    if (value < *min_v)
        *min_v = value;
    if (value > *max_v)
        *max_v = value;
}

/**
 * @brief 重置自检统计信息。
 *
 */
static void MotorSelfTest_ResetStats(void)
{
    s_selftest_stats.left_min = FLT_MAX;
    s_selftest_stats.left_max = -FLT_MAX;
    s_selftest_stats.right_min = FLT_MAX;
    s_selftest_stats.right_max = -FLT_MAX;
    s_selftest_stats.m3_min = FLT_MAX;
    s_selftest_stats.m3_max = -FLT_MAX;
    s_selftest_stats.m4_min = FLT_MAX;
    s_selftest_stats.m4_max = -FLT_MAX;
}
/**
 * @brief 更新左右轮电流统计信息。
 *
 */
static void MotorSelfTest_UpdateStats_LR(void)
{
    float left = Adc_GetLeftMotorCurrent();
    float right = Adc_GetRightMotorCurrent();

    MotorSelfTest_UpdateMinMax(left, &s_selftest_stats.left_min, &s_selftest_stats.left_max);
    MotorSelfTest_UpdateMinMax(right, &s_selftest_stats.right_min, &s_selftest_stats.right_max);
}

/**
 * @brief  更新 MOTO3/MOTO4 电流统计信息。
 *
 */
static void MotorSelfTest_UpdateStats_M34(void)
{
    float m3 = Adc_GetMotor3Current();
    float m4 = Adc_GetMotor4Current();

    MotorSelfTest_UpdateMinMax(m3, &s_selftest_stats.m3_min, &s_selftest_stats.m3_max);
    MotorSelfTest_UpdateMinMax(m4, &s_selftest_stats.m4_min, &s_selftest_stats.m4_max);
}

/**
 * @brief 报告自检结果。
 *
 */
static void MotorSelfTest_ReportResult(void)
{
    printf("自检结果:\r\n");
    if (s_selftest_stats.m3_min != FLT_MAX)
    {
        if (s_selftest_stats.m3_max >= 5.5f)
            printf("报警:电机3过载,max=%.2fA\r\n", s_selftest_stats.m3_max);
        if (s_selftest_stats.m3_min <= 1.0f)
            printf("报警:电机3出水,min=%.2fA\r\n", s_selftest_stats.m3_min);
    }

    if (s_selftest_stats.m4_min != FLT_MAX)
    {
        if (s_selftest_stats.m4_max >= 5.5f)
            printf("报警:电机4过载,max=%.2fA\r\n", s_selftest_stats.m4_max);
        if (s_selftest_stats.m4_min <= 1.0f)
            printf("报警:电机4出水,min=%.2fA\r\n", s_selftest_stats.m4_min);
    }

    if (s_selftest_stats.left_min != FLT_MAX)
    {
        if (s_selftest_stats.left_max >= 1.2f)
            printf("报警:左行走电机过载,max=%.2fA\r\n", s_selftest_stats.left_max);
        if (s_selftest_stats.left_min <= 0.5f)
            printf("报警:左行走电机出水,min=%.2fA\r\n", s_selftest_stats.left_min);
    }

    if (s_selftest_stats.right_min != FLT_MAX)
    {
        if (s_selftest_stats.right_max >= 1.2f)
            printf("报警:右行走电机过载,max=%.2fA\r\n", s_selftest_stats.right_max);
        if (s_selftest_stats.right_min <= 0.5f)
            printf("报警:右行走电机出水,min=%.2fA\r\n", s_selftest_stats.right_min);
    }
}

/*
 * @brief 应用当前模式到电机输出。
 *
 * @param mode 模式
 * @param pwm  占空比
 */
static void MotorCtrl_Apply(MotorCtrlMode_t mode, uint16_t pwm)
{
    if ((mode == MOTOR_CTRL_STOP) || (pwm == 0))
    {
        PWM_SetDutyCCR(MOTOR_PWM, 0);
        PWM_SetDutyCCR(MOTOL_PWM, 0);
        return;
    }

    switch (mode)
    {
    case MOTOR_CTRL_FORWARD:            /* 前进 */
        MotorCtrl_SetDir(0, 1);         // 前进方向
        PWM_SetDutyCCR(MOTOR_PWM, pwm); // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, pwm); // 左轮
        break;
    case MOTOR_CTRL_BACKWARD:           /* 后退 */
        MotorCtrl_SetDir(1, 0);         // 后退方向
        PWM_SetDutyCCR(MOTOR_PWM, pwm); // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, pwm); // 左轮
        break;
    case MOTOR_CTRL_TURN_LEFT:          /* 左转 */
        MotorCtrl_SetDir(0, 0);         // 前进方向
        PWM_SetDutyCCR(MOTOR_PWM, pwm); // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, 0);   // 左轮
        break;
    case MOTOR_CTRL_TURN_RIGHT:         /* 右转 */
        MotorCtrl_SetDir(0, 0);         // 前进方向
        PWM_SetDutyCCR(MOTOR_PWM, 0);   // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, pwm); // 左轮
        break;
    case MOTOR_CTRL_SELFTEST:         /* 自检（占位） */
        PWM_SetDutyCCR(MOTOR_PWM, 0); // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, 0); // 左轮
        break;
    case MOTOR_CTRL_STOP:             /* 停止 */
    default:                          // 防止未定义模式
        PWM_SetDutyCCR(MOTOR_PWM, 0); // 右轮
        PWM_SetDutyCCR(MOTOL_PWM, 0); // 左轮
        break;
    }
}

/**
 * @brief 获取碰撞退避时间（毫秒）。
 *
 * @param mode 碰撞方向（0=前，1=后，2=左，3=右）
 * @return uint16_t 退避时间（毫秒）
 */
static uint16_t Collision_BackoffMs(uint8_t mode)
{
    if (mode == 0 || mode == 1)
        return 2000; // 前/后碰撞
    return 1500;     // 左/右碰撞
}

static uint16_t Collision_TurnMs(uint8_t mode)
{
    if (mode == 0 || mode == 1)
        return 800; // 前/后碰撞
    return 600;     // 左/右碰撞
}

static MotorCtrlMode_t Collision_BackoffMode(uint8_t mode)
{
    return (mode == 1) ? MOTOR_CTRL_FORWARD : MOTOR_CTRL_BACKWARD;
}

static MotorCtrlMode_t Collision_TurnMode(uint8_t mode)
{
    // 前碰撞/右碰撞：左转；后碰撞/左碰撞：右转
    if (mode == 0 || mode == 3)
        return MOTOR_CTRL_TURN_LEFT;
    return MOTOR_CTRL_TURN_RIGHT;
}

static MotorCtrlMode_t AutoSeq_TurnMode(void)
{
#if AUTO_SEQ_TURN_DIR_RIGHT
    return MOTOR_CTRL_TURN_RIGHT;
#else
    return MOTOR_CTRL_TURN_LEFT;
#endif
}
/**
 * @brief 计算角度差值，结果范围 -180 ~ +180 度。
 *
 * @param cur 当前角度
 * @param ref 参考角度
 * @return float 角度差值
 */
static float MotorCtrl_AngleDiff(float cur, float ref)
{
    float d = cur - ref;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

/**
 * @brief 初始化电机控制模块。
 */
void MotorCtrl_Init(void)
{
    s_ctrl.mode = MOTOR_CTRL_STOP; // 停止
    s_ctrl.pwm = 0;
    s_ctrl.end_tick = 0;
    s_selftest.active = 0;
    s_selftest.state = SELFTEST_IDLE;
    g_motor_selftest_busy = 0;           // 初始空闲
    MotorCtrl_Apply(MOTOR_CTRL_STOP, 0); // 停止电机
    MotorCtrl_SetAuxEnable(0);           // 关闭 MOTO3/MOTO4
}

/**
 * @brief 电机控制任务（非阻塞轮询）。
 */
void MotorCtrl_Task(void)
{

    uint32_t now = HAL_GetTick();
    static uint8_t last_power_on = 0;

    /* 定时任务到期：关闭模式并停机（避免自动/爬壁重新拉起电机） */
    if (UsrTime_IsExpired()) /* 仅查看到期标志，不消耗；UsrTime_Poll() 的回调负责清标志并向 App 上报状态 */
    {
        g_auto_enable = 0;
        g_wall_enable = 0;
        MotorCtrl_StopAll();
        return;
    }

    /* 开机自检入口暂时停用，仅保留开机沿记录，避免影响其他模式逻辑 */
    // if (system_flag_bool && !last_power_on)
    // {
    //     MotorCtrl_SelfTestStart(MOTOR_COLLISION_PWM_DEFAULT);
    // }
    last_power_on = system_flag_bool ? 1u : 0u;

    /* 自检流程优先 */
    if (s_selftest.active) // 自检
    {
        if (s_selftest.state == SELFTEST_M34)
        {
            MotorSelfTest_UpdateStats_M34();
        }
        else if ((s_selftest.state == SELFTEST_LR_STAGE1) || (s_selftest.state == SELFTEST_LR_STAGE2))
        {
            MotorSelfTest_UpdateStats_LR();
        }

        // printf("------------------电机自检中------------------\r\n");
        if ((int32_t)(now - s_selftest.deadline) >= 0) // 到达阶段截止时间
        {
            switch (s_selftest.state)
            {
            case SELFTEST_M34:
                MotorCtrl_SetAuxEnable(0);                 // 关闭 MOTO3/MOTO4
                MotorCtrl_SetDir(1, 0);                    // 左正、右反
                PWM_SetDutyCCR(MOTOR_PWM, s_selftest.pwm); // 右轮
                PWM_SetDutyCCR(MOTOL_PWM, s_selftest.pwm); // 左轮
                s_selftest.state = SELFTEST_LR_STAGE1;     // 左正右反 10s
                s_selftest.deadline = now + 10000u;        // 10 秒
                break;

            case SELFTEST_LR_STAGE1:                       // 左正右反 10s
                MotorCtrl_SetDir(0, 1);                    // 左反、右正
                PWM_SetDutyCCR(MOTOR_PWM, s_selftest.pwm); // 右轮
                PWM_SetDutyCCR(MOTOL_PWM, s_selftest.pwm); // 左轮
                s_selftest.state = SELFTEST_LR_STAGE2;     // 左反右正 10s
                s_selftest.deadline = now + 10000u;        // 10 秒
                break;

            case SELFTEST_LR_STAGE2: // 左反右正 10s
            default:
                MotorCtrl_SetAuxEnable(0);           // 关闭 MOTO3/MOTO4
                MotorSelfTest_ReportResult();        // 报告自检结果
                MotorCtrl_Apply(MOTOR_CTRL_STOP, 0); // 停止左右轮
                s_selftest.active = 0;               // 自检结束
                g_motor_selftest_busy = 0;           // 标记空闲
                s_selftest.state = SELFTEST_IDLE;    // 空闲状态
                s_ctrl.mode = MOTOR_CTRL_STOP;       // 停止
                s_ctrl.pwm = 0;                      // 停止
                s_ctrl.end_tick = 0;                 // 无结束时间
                break;
            }
        }

        return; /* 自检期间不处理其他动作 */
    }

    if (g_auto_enable)
    {
        MotorCtrl_SetAuxEnable(1);                                    /* 自动/爬墙模式下启用抽水电机 */
        MotorCtrl_SensorUpdate(0.02f);                                /* 更新传感器数据 */
        MotorCtrl_AutoModeUpdate(0.02f, MOTOR_COLLISION_PWM_DEFAULT); /* 自动模式更新 */
    }
    else if (g_wall_enable)
    {
        MotorCtrl_SetAuxEnable(1);       /* 自动/爬墙模式下启用抽水电机 */
        MotorCtrl_WallClimbUpdate(1000); /* 爬墙模式更新 */
    }

    if (!g_auto_enable && !g_wall_enable)
    {
        if (s_ctrl.end_tick != 0) // 有动作结束时间
        {
            if ((int32_t)(now - s_ctrl.end_tick) >= 0) // 到达动作结束时间
            {
                MotorCtrl_Stop(); // 停止动作
            }
        }
    }
    return;
    // printf("------------------电机自检结束------------------\r\n");
}

/**
 * @brief 设置电机动作并可选定时停止。
 * @param mode        动作模式
 * @param pwm         PWM 占空比（0~ARR）
 * @param duration_ms 动作持续时间（毫秒）
 */
void MotorCtrl_SetAction(MotorCtrlMode_t mode, uint16_t pwm, uint32_t duration_ms)
{
    if (s_collision.active)
    {
        return; /* 碰撞流程中忽略外部动作 */
    }

    if (s_selftest.active)
    {
        return; /* 自检期间忽略外部动作指令 */
    }

    s_ctrl.mode = mode;
    s_ctrl.pwm = pwm;
    if (duration_ms > 0)
    {
        s_ctrl.end_tick = HAL_GetTick() + duration_ms;
    }
    else
    {
        s_ctrl.end_tick = 0;
    }

    MotorCtrl_Apply(mode, pwm);
}

/**
 * @brief 启动电机自检流程（非阻塞）。
 * @param pwm 左右轮自检使用的 PWM 占空比
 */
void MotorCtrl_SelfTestStart(uint16_t pwm)
{
    if (s_selftest.active)
    {
        return; // 已在自检中
    }

    g_motor_selftest_busy = 1;                    // 标记自检中
    s_selftest.active = 1;                        // 标记为自检中
    s_selftest.state = SELFTEST_M34;              // MOTO3/MOTO4 使能 15s
    s_selftest.pwm = pwm;                         // 保存自检 PWM
    s_selftest.deadline = HAL_GetTick() + 15000u; // 15 秒
    MotorSelfTest_ResetStats();                   // 重置统计信息
    s_ctrl.mode = MOTOR_CTRL_SELFTEST;            // 设置自检模式
    s_ctrl.pwm = pwm;                             // 保存 PWM
    s_ctrl.end_tick = 0;                          // 无结束时间

    MotorCtrl_Apply(MOTOR_CTRL_STOP, 0); /* 先停止左右轮 */
    MotorCtrl_SetAuxEnable(1);           /* MOTO3/MOTO4 使能 15 秒 */
}

/**
 * @brief 查询电机自检是否正在进行。
 */
uint8_t MotorCtrl_SelfTestActive(void)
{
    return s_selftest.active; // 1 正在自检，0 未自检
}

/**
 * @brief 立即停止电机。
 */
void MotorCtrl_Stop(void)
{
    if (s_selftest.active)
    {
        return; /* 自检期间不响应外部停止 */
    }

    s_ctrl.mode = MOTOR_CTRL_STOP;
    s_ctrl.pwm = 0;
    s_ctrl.end_tick = 0;
    MotorCtrl_Apply(MOTOR_CTRL_STOP, 0);
}

/**
 * @brief 立即停止左右行走电机并关闭 MOTO3/MOTO4。
 */
void MotorCtrl_StopAll(void)
{
    s_auto.active = 0;
    s_auto.state = AUTO_IDLE;
    s_auto.deadline = 0;

    s_selftest.active = 0;
    s_selftest.state = SELFTEST_IDLE;
    s_selftest.deadline = 0;
    g_motor_selftest_busy = 0;

    s_collision.active = 0;
    s_collision.state = COLLISION_IDLE;
    s_collision.deadline = 0;

    s_ctrl.mode = MOTOR_CTRL_STOP;
    s_ctrl.pwm = 0;
    s_ctrl.end_tick = 0;

    PWM_SetDutyCCR(MOTOR_PWM, 0);
    PWM_SetDutyCCR(MOTOL_PWM, 0);
    MotorCtrl_SetAuxEnable(0);
}

/**
 * @brief 控制抽水电机（MOTO3/MOTO4）使能。
 * @param enable 1=使能，0=关闭
 */
void MotorCtrl_WaterPumpEnable(uint8_t enable)
{
    MotorCtrl_SetAuxEnable(enable);
}

/**
 * @brief 获取当前电机模式。
 * @return MOTOR_CTRL_STOP——停止
 * MOTOR_CTRL_FORWARD——前进
 * MOTOR_CTRL_BACKWARD——后退
 * MOTOR_CTRL_TURN_LEFT——左转
 * MOTOR_CTRL_TURN_RIGHT——右转)
 */
MotorCtrlMode_t MotorCtrl_GetMode(void)
{
    return s_ctrl.mode;
}

/**
 * @brief 启动碰撞处理流程。
 *
 * @param hit_mode
 * @param pwm
 */
void MotorCtrl_CollisionStart(uint8_t hit_mode, uint16_t pwm)
{
    if (s_selftest.active || s_collision.active)
    {
        return;
    }

    s_collision.active = 1;
    s_collision.mode = hit_mode;
    s_collision.state = COLLISION_BACKOFF;
    s_collision.pwm = pwm;
    s_collision.deadline = HAL_GetTick() + Collision_BackoffMs(hit_mode);

    MotorCtrl_Apply(Collision_BackoffMode(hit_mode), pwm);
}
/**
 * @brief 查询碰撞处理是否正在进行。
 *
 * @return uint8_t 1 表示正在进行，0 表示未进行。
 */
uint8_t MotorCtrl_CollisionActive(void)
{
    return s_collision.active;
}
/**
 * @brief 自动模式执行任务
 *
 * @param hit_front
 * @param hit_back
 * @param hit_left
 * @param hit_right
 * @param pitch_deg
 * @param yaw_deg
 * @param pwm
 */
void MotorCtrl_AutoModeTask(uint8_t hit_front,
                            uint8_t hit_back,
                            uint8_t hit_left,
                            uint8_t hit_right,
                            float pitch_deg,
                            float yaw_deg,
                            uint16_t pwm)

{
    uint32_t now = HAL_GetTick();
    uint8_t hit = (hit_front || hit_back) ? 1u : 0u;

    /* 自检或碰撞流程时不介入 */
    if (s_selftest.active || s_collision.active)
    {
        return;
    }

    /* 仅保留前后碰撞，忽略左右传感与姿态输入 */
    (void)hit_left;
    (void)hit_right;
    (void)pitch_deg;
    (void)yaw_deg;

    /* 首次进入自动模式，直接直线运行，并设置最长直行时间 */
    if (!s_auto.active)
    {
        s_auto.active = 1;
        s_auto.state = AUTO_DRIVE;
        s_auto.pwm = pwm;
        s_auto.deadline = now + AUTO_DRIVE_MAX_MS;
        MotorCtrl_Apply(MOTOR_CTRL_FORWARD, pwm);
    }

    switch (s_auto.state)
    {
    case AUTO_DRIVE:
        /* 前/后碰撞或直行超时，进入 S 型转向序列 */
        if (hit || ((int32_t)(now - s_auto.deadline) >= 0))
        {
            s_auto.turn_dir ^= 1; /* 每次触发交替转向方向，形成 S 型路线 */
            s_auto.state = AUTO_TURN_1;
            s_auto.deadline = now + AUTO_TURN_MS;
            if (s_auto.turn_dir == 0)
                MotorCtrl_Apply(MOTOR_CTRL_TURN_LEFT, pwm);
            else
                MotorCtrl_Apply(MOTOR_CTRL_TURN_RIGHT, pwm);
        }
        break;

    case AUTO_TURN_1:
        if ((int32_t)(now - s_auto.deadline) >= 0)
        {
            s_auto.state = AUTO_FORWARD_SHORT;
            s_auto.deadline = now + AUTO_FORWARD_SHORT_MS;
            MotorCtrl_Apply(MOTOR_CTRL_FORWARD, pwm);
        }
        break;

    case AUTO_FORWARD_SHORT:
        if ((int32_t)(now - s_auto.deadline) >= 0)
        {
            s_auto.state = AUTO_TURN_2;
            s_auto.deadline = now + AUTO_TURN_MS;
            if (s_auto.turn_dir == 0)
                MotorCtrl_Apply(MOTOR_CTRL_TURN_LEFT, pwm);
            else
                MotorCtrl_Apply(MOTOR_CTRL_TURN_RIGHT, pwm);
        }
        break;

    case AUTO_TURN_2:
        if ((int32_t)(now - s_auto.deadline) >= 0)
        {
            s_auto.state = AUTO_DRIVE;
            s_auto.deadline = now + AUTO_DRIVE_MAX_MS;
            MotorCtrl_Apply(MOTOR_CTRL_FORWARD, pwm);
        }
        break;

    case AUTO_IDLE:
    default:
        s_auto.state = AUTO_DRIVE;
        s_auto.deadline = now + AUTO_DRIVE_MAX_MS;
        MotorCtrl_Apply(MOTOR_CTRL_FORWARD, pwm);
        break;
    }
}

/**
 * @brief 查询当前是否处于墙体爬升状态。
 *
 * @return uint8_t 1 表示正在爬升，0 表示未爬升。
 */
uint8_t MotorCtrl_WallClimbUpdate(uint16_t pwm)
{
    /* 0=直行找墙 1=已碰壁等待爬壁 2=爬壁直行30s 3=后退下墙 4=转向1 5=直行1s 6=转向2 */
    static uint8_t state = 0;
    static uint32_t deadline = 0;
    static uint8_t turn_dir = 0; /* 0=左转，1=右转，交替形成S型路线 */

    float rpy[3] = {0};
    float roll_deg = 0.0f;
    float ar = 0.0f;
    uint8_t hit_front = 0;
    uint8_t hit_back = 0;
    uint8_t hit = 0;

    if (s_selftest.active || s_collision.active)
        return s_wall_climb;

    hit_front = (HAL_GPIO_ReadPin(HIT_IN1_GPIO_Port, HIT_IN1_Pin) == GPIO_PIN_RESET);
    hit_back = (HAL_GPIO_ReadPin(HIT_IN2_GPIO_Port, HIT_IN2_Pin) == GPIO_PIN_RESET);
    hit = (hit_front || hit_back) ? 1u : 0u;

    if (!IMU_GetRPY(rpy))
        return s_wall_climb;

    roll_deg = rpy[0];
    ar = fabsf(roll_deg);

    /* 打印当前角度（1s一次） */
    {
        static uint32_t pitch_log_tick = 0;
        uint32_t now = HAL_GetTick();
        if ((now - pitch_log_tick) >= 100u)
        {
            pitch_log_tick = now;
            printf("ar=%.2f\r\n", ar);
        }
    }

    switch (state)
    {
    case 0: /* 直行找墙 */
        s_wall_climb = 0;
        MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
        if (hit)
        {
            state = 1;
            deadline = HAL_GetTick() + 30000u; /* 碰壁后等待爬壁最多30s */
        }
        break;

    case 1: /* 已碰壁，继续直行，等待陀螺仪检测爬壁或超时 */
        MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
        if (ar >= MOTOR_PITCH_CLIMB_MIN && ar <= MOTOR_PITCH_CLIMB_MAX)
        {
            s_wall_climb = 1;
            state = 2;
            deadline = HAL_GetTick() + 30000u; /* 爬壁直行30s */
        }
        else if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        {
            /* 未检测到爬壁，超时转向 */
            turn_dir ^= 1;
            state = 4;
            deadline = HAL_GetTick() + AUTO_TURN_MS;
            MotorCtrl_SetAction(turn_dir ? MOTOR_CTRL_TURN_RIGHT : MOTOR_CTRL_TURN_LEFT, pwm, 0);
        }
        break;

    case 2: /* 爬壁直行30s */
        MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
        if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        {
            state = 3;
            MotorCtrl_SetAction(MOTOR_CTRL_BACKWARD, pwm, 0);
        }
        break;

    case 3: /* 后退下墙，直到角度恢复 */
        MotorCtrl_SetAction(MOTOR_CTRL_BACKWARD, pwm, 0);
        if (ar <= MOTOR_PITCH_CLIMB_EXIT)
        {
            s_wall_climb = 0;
            turn_dir ^= 1;
            state = 4;
            deadline = HAL_GetTick() + AUTO_TURN_MS;
            MotorCtrl_SetAction(turn_dir ? MOTOR_CTRL_TURN_RIGHT : MOTOR_CTRL_TURN_LEFT, pwm, 0);
        }
        break;

    case 4: /* 转向90度 */
        if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        {
            state = 5;
            deadline = HAL_GetTick() + AUTO_FORWARD_SHORT_MS;
            MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
        }
        break;

    case 5: /* 直行1秒 */
        if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        {
            state = 6;
            deadline = HAL_GetTick() + AUTO_TURN_MS;
            MotorCtrl_SetAction(turn_dir ? MOTOR_CTRL_TURN_RIGHT : MOTOR_CTRL_TURN_LEFT, pwm, 0);
        }
        break;

    case 6: /* 再转向90度，进入下一段直行找墙 */
        if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        {
            state = 0;
        }
        break;

    default:
        state = 0;
        s_wall_climb = 0;
        break;
    }

    return s_wall_climb;
}

void MotorCtrl_AutoModeUpdate(float dt, uint16_t pwm)
{
    (void)dt; // dt 不再用
    MotorCtrl_AutoModeTask(s_hit_front, s_hit_back, s_hit_left, s_hit_right,
                           s_rpy[1], s_rpy[2], pwm);
}

void MotorCtrl_AutoSeqUpdate(uint16_t pwm)
{
    uint32_t now = HAL_GetTick();

    if (MotorCtrl_SelfTestActive())
    {
        s_auto_seq.state = AUTO_SEQ_WAIT_SELFTEST;
        s_auto_seq.deadline = 0;
        return;
    }

    if (MotorCtrl_CollisionActive())
    {
        return;
    }

    switch (s_auto_seq.state)
    {
    case AUTO_SEQ_WAIT_SELFTEST:
        MotorCtrl_SetAction(MOTOR_CTRL_STOP, 0, 0);
        s_auto_seq.state = AUTO_SEQ_WAIT_AFTER_SELFTEST;
        s_auto_seq.deadline = now + AUTO_SEQ_PAUSE_AFTER_SELFTEST_MS;
        break;

    case AUTO_SEQ_WAIT_AFTER_SELFTEST:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
            s_auto_seq.state = AUTO_SEQ_FORWARD_LONG;
            s_auto_seq.deadline = now + AUTO_SEQ_FORWARD_LONG_MS;
        }
        break;

    case AUTO_SEQ_FORWARD_LONG:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(MOTOR_CTRL_BACKWARD, pwm, 0);
            s_auto_seq.state = AUTO_SEQ_BACKOFF;
            s_auto_seq.deadline = now + AUTO_SEQ_BACKOFF_MS;
        }
        break;

    case AUTO_SEQ_BACKOFF:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(AutoSeq_TurnMode(), pwm, 0);
            s_auto_seq.state = AUTO_SEQ_TURN_1;
            s_auto_seq.deadline = now + AUTO_SEQ_TURN_90_MS;
        }
        break;

    case AUTO_SEQ_TURN_1:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
            s_auto_seq.state = AUTO_SEQ_FORWARD_SHORT;
            s_auto_seq.deadline = now + AUTO_SEQ_FORWARD_SHORT_MS;
        }
        break;

    case AUTO_SEQ_FORWARD_SHORT:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(AutoSeq_TurnMode(), pwm, 0);
            s_auto_seq.state = AUTO_SEQ_TURN_2;
            s_auto_seq.deadline = now + AUTO_SEQ_TURN_90_MS;
        }
        break;

    case AUTO_SEQ_TURN_2:
        if ((int32_t)(now - s_auto_seq.deadline) >= 0)
        {
            MotorCtrl_SetAction(MOTOR_CTRL_FORWARD, pwm, 0);
            s_auto_seq.state = AUTO_SEQ_FORWARD_LONG;
            s_auto_seq.deadline = now + AUTO_SEQ_FORWARD_LONG_MS;
        }
        break;

    default:
        s_auto_seq.state = AUTO_SEQ_WAIT_SELFTEST;
        s_auto_seq.deadline = 0;
        break;
    }
}

void MotorCtrl_SensorUpdate(float dt)
{
    qmi8658_read_xyz(s_acc, s_gyro);
    imu_get_eulerian_angles(s_acc, s_gyro, s_rpy, dt);

    s_hit_front = (HAL_GPIO_ReadPin(HIT_IN1_GPIO_Port, HIT_IN1_Pin) == GPIO_PIN_RESET);
    s_hit_back = (HAL_GPIO_ReadPin(HIT_IN2_GPIO_Port, HIT_IN2_Pin) == GPIO_PIN_RESET);
    s_hit_left = (HAL_GPIO_ReadPin(HIT_IN3_GPIO_Port, HIT_IN3_Pin) == GPIO_PIN_RESET);
    s_hit_right = (HAL_GPIO_ReadPin(HIT_IN4_GPIO_Port, HIT_IN4_Pin) == GPIO_PIN_RESET);
}
