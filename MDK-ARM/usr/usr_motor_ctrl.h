#ifndef __USR_MOTOR_CTRL_H
#define __USR_MOTOR_CTRL_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* 默认碰撞处理 PWM（可根据需要调） */
#define MOTOR_COLLISION_PWM_DEFAULT 999

/* Wall climb thresholds (deg) */
#define MOTOR_PITCH_CLIMB_MIN 50.0f
#define MOTOR_PITCH_CLIMB_MAX 120.0f
#define MOTOR_PITCH_CLIMB_EXIT 20.0f
#define AUTO_YAW_LEFT_POSITIVE 1

/* 自动序列（两分钟前进+转向）参数 */
#define AUTO_SEQ_PAUSE_AFTER_SELFTEST_MS 2000u
#define AUTO_SEQ_FORWARD_LONG_MS 120000u
#define AUTO_SEQ_BACKOFF_MS 2000u
#define AUTO_SEQ_TURN_90_MS 800u
#define AUTO_SEQ_FORWARD_SHORT_MS 3000u
/* 1=右转，0=左转，后续只需改这个宏 */
#define AUTO_SEQ_TURN_DIR_RIGHT 0

/**
 * @brief 自动模式执行任务
 * @param hit_front 前碰撞
 * @param hit_back  后碰撞（当前逻辑未用，可后续扩展）
 * @param hit_left  左碰撞
 * @param hit_right 右碰撞
 * @param pitch_deg 当前 pitch 角度（度）
 * @param yaw_deg   当前 yaw 角度（度）
 * @param pwm       自动模式 PWM
 */
void MotorCtrl_AutoModeTask(uint8_t hit_front,
                            uint8_t hit_back,
                            uint8_t hit_left,
                            uint8_t hit_right,
                            float pitch_deg,
                            float yaw_deg,
                            uint16_t pwm);

void MotorCtrl_SensorUpdate(float dt); // 姿态+碰撞采样（一直跑）

/**
 * @brief 自动模式总入口（内部完成IMU与碰撞采样）
 * @param dt 采样周期（秒）
 * @param pwm 自动模式 PWM
 */
void MotorCtrl_AutoModeUpdate(float dt, uint16_t pwm);

/**
 * @brief 自动序列任务（自检结束后暂停2s -> 前进2min -> 后退2s -> 转90 -> 前进3s -> 再转90 -> 循环）
 * @param pwm 自动序列 PWM
 */
void MotorCtrl_AutoSeqUpdate(uint16_t pwm);

/**
 * @brief Wall climb update based on pitch.
 * @param pitch_deg Pitch angle in deg
 * @param pwm PWM used during climb
 * @return uint8_t 1=climbing,0=not
 */
uint8_t MotorCtrl_WallClimbUpdate(uint16_t pwm);

/**
 * @brief 电机控制模式定义（用于左右轮）。
 */
typedef enum
{
    MOTOR_CTRL_STOP = 0,   /* 停止 */
    MOTOR_CTRL_FORWARD,    /* 前进 */
    MOTOR_CTRL_BACKWARD,   /* 后退 */
    MOTOR_CTRL_TURN_LEFT,  /* 左转 */
    MOTOR_CTRL_TURN_RIGHT, /* 右转 */
    MOTOR_CTRL_SELFTEST    /* 自检（占位） */
} MotorCtrlMode_t;

/* 自检全局标志：1 自检中，0 空闲 */
extern volatile uint8_t g_motor_selftest_busy;

/**
 * @brief 初始化电机控制模块。
 */
void MotorCtrl_Init(void);

/**
 * @brief 电机控制任务（非阻塞轮询）。
 *
 * @details
 * 在主循环中周期调用，用于处理定时动作的结束和状态维护。
 */
void MotorCtrl_Task(void);

/**
 * @brief 设置电机动作并可选定时停止。
 *
 * @param mode        动作模式
 * @param pwm         PWM 占空比（0~ARR）
 * @param duration_ms 持续时间（毫秒），为 0 表示一直保持
 */
void MotorCtrl_SetAction(MotorCtrlMode_t mode, uint16_t pwm, uint32_t duration_ms);

/**
 * @brief 启动电机自检流程（非阻塞）。
 *
 * @param pwm 左右轮自检使用的 PWM 占空比
 *
 * @note 自检期间不响应外部动作指令，直到流程结束。
 */
void MotorCtrl_SelfTestStart(uint16_t pwm);

/**
 * @brief 查询电机自检是否正在进行。
 *
 * @retval 1 正在自检，0 未自检
 */
uint8_t MotorCtrl_SelfTestActive(void);

/**
 * @brief 立即停止电机。
 */
void MotorCtrl_Stop(void);

/**
 * @brief 立即停止左右行走电机并关闭 MOTO3/MOTO4。
 */
void MotorCtrl_StopAll(void);

/**
 * @brief 控制抽水电机（MOTO3/MOTO4）使能。
 * @note  协议层通过此接口控制水泵，无需直接操作 GPIO。
 * @param enable 1=使能水泵，0=关闭水泵
 */
void MotorCtrl_WaterPumpEnable(uint8_t enable);

/**
 * @brief 获取当前电机模式。
 */
MotorCtrlMode_t MotorCtrl_GetMode(void);

/**
 * @brief 碰撞触发时序：退避 + 转向。
 *
 * @param hit_mode 0=前碰撞，1=后碰撞，2=左碰撞，3=右碰撞
 * @param pwm      退避/转向使用的 PWM
 */
void MotorCtrl_CollisionStart(uint8_t hit_mode, uint16_t pwm);

/**
 * @brief 当前是否在碰撞处理流程中。
 */
uint8_t MotorCtrl_CollisionActive(void);

#endif
