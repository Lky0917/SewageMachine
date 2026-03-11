#include "usr_key.h"
#include <stdio.h>
#include "multi_button.h"
#include "bt_tuya.h"

/*******************************************************************************************
    1.短按，系统开机，进入自检 -> 工作，电源指示灯常亮(未做)
    2.长按 2S左右，电源指示灯闪烁3下，系统关机(未做)
    3.长按 5S左右，电源指示灯快闪，进入设备搜索模式(未做)
 ********************************************************************************************/

/* 按键 GPIO 端口与引脚定义 */
#define KEY_GPIO_PORT KEY_IN_OFF_GPIO_Port
#define KEY_GPIO_PIN KEY_IN_OFF_Pin

/* 按键按下电平（低电平有效） */
#define KEY_PRESSED_LEVEL GPIO_PIN_RESET

/* multiButton 轮询周期（ms） */
#define KEY_TICK_MS TICKS_INTERVAL

/* 按键句柄 */
static struct Button g_keyHandle;
/* 最近一次产生的按键事件 */
static volatile KeyEvent_t g_keyEvent = KEY_EVENT_NONE;
/* multiButton 节拍时间戳 */
static uint32_t t_tick = 0;
extern volatile unsigned char system_flag_bool;

extern uint8_t g_power_switch; // 初始电源开关

static uint8_t Key_ReadLevel(void);
static void Key_OnSingle(void *btn);
static void Key_OnLongStart(void *btn);
static void Key_OnRepeat(void *btn);

/**
 * @brief 短按按键回调。
 *
 * @note 事件消费函数入口
 */
static void OnShortPress(void)
{
    printf("Short press\r\n");
    if (system_flag_bool == 0)
    {
        system_flag_bool = 1; /* 短按开机 */
        g_power_switch = 1;
        mcu_dp_bool_update(DPID_SWITCH, g_power_switch);
        printf("Power on by short press\r\n");
    }
}

/**
 * @brief 三连击按键回调。
 *
 * @note 事件消费函数入口
 */
static void OnTriplePress(void)
{
    printf("Triple click\r\n");
}

/**
 * @brief 长按按键回调。
 *
 * @note 事件消费函数入口
 */
static void OnLongPress(void)
{
    system_flag_bool = 0; /* 长按2秒关机 */
    g_power_switch = 0;
    mcu_dp_bool_update(DPID_SWITCH, g_power_switch);
    printf("Long press 2s\r\n");
}

/**
 * @brief 读取当前按键电平。
 *
 * @retval GPIO 电平
 */
static uint8_t Key_ReadLevel(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN);
}

/**
 * @brief 单击事件回调。
 */
static void Key_OnSingle(void *btn)
{
    (void)btn;
    g_keyEvent = KEY_EVENT_SHORT;
    // printf("Key single click\r\n");
}

/**
 * @brief 长按（2s）起始回调。
 */
static void Key_OnLongStart(void *btn)
{
    (void)btn;
    g_keyEvent = KEY_EVENT_LONG;
    // printf("Key long press 2s\r\n");
}

/**
 * @brief 连击计数回调，用于识别三连击。
 */
static void Key_OnRepeat(void *btn)
{
    Button *handle = (Button *)btn;
    if (handle->repeat == 3u)
    {
        g_keyEvent = KEY_EVENT_TRIPLE;
        // printf("Key triple click\r\n");
    }
}

/**
 * @brief 按键模块初始化。
 */
void Key_Init(void)
{
    g_keyEvent = KEY_EVENT_NONE;
    t_tick = 0;

    button_init(&g_keyHandle, Key_ReadLevel, KEY_PRESSED_LEVEL);
    button_attach(&g_keyHandle, SINGLE_CLICK, Key_OnSingle);
    button_attach(&g_keyHandle, LONG_RRESS_START, Key_OnLongStart);
    button_attach(&g_keyHandle, PRESS_REPEAT, Key_OnRepeat);
    button_start(&g_keyHandle);
}

/**
 * @brief 按键任务（轮询调用）。
 *
 * @details 按 multiButton 设定节拍调用 driver。
 */
void Key_Task(void)
{
    uint32_t now = HAL_GetTick();

    while ((now - t_tick) >= KEY_TICK_MS)
    {
        t_tick += KEY_TICK_MS;
        button_ticks();
    }
}

/**
 * @brief 获取并清除一次按键事件。
 *
 * @retval 当前缓存的按键事件
 */
KeyEvent_t Key_GetEvent(void)
{
    KeyEvent_t ev = g_keyEvent;
    g_keyEvent = KEY_EVENT_NONE;
    return ev;
}

/**
 * @brief 按键扫描函数
 *
 */
void Key_Scan(void)
{
    KeyEvent_t ev;       // 按键事件变量
    ev = Key_GetEvent(); /* 获取按键事件 */
    switch (ev)
    {
    case KEY_EVENT_SHORT:
        OnShortPress();
        break;
    case KEY_EVENT_TRIPLE:
        OnTriplePress();
        break;
    case KEY_EVENT_LONG:
        OnLongPress();
        break;
    default:
        break;
    }
}
