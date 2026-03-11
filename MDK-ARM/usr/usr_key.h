#ifndef __USR_KEY_H
#define __USR_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

    typedef enum
    {
        KEY_EVENT_NONE = 0, /* 无事件 */
        KEY_EVENT_SHORT,    /* 单击事件 */
        KEY_EVENT_TRIPLE,   /* 三连击事件 */
        KEY_EVENT_LONG      /* 长按（2s）事件 */
    } KeyEvent_t;           /* 按键事件枚举 */

    void Key_Init(void);           /* 按键初始化 */
    void Key_Task(void);           /* 按键状态机任务 */
    KeyEvent_t Key_GetEvent(void); /* 获取按键事件 */
    void Key_Scan(void);           /* 按键扫描处理函数 */

#ifdef __cplusplus
}
#endif

#endif
