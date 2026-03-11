#ifndef __HIT_H
#define __HIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
}
#endif

/**
 * @brief 扫描工作模式输入并更新全局工作模式。
 *
 * @details
 * 根据 HIT_IN1~HIT_IN4 的电平组合判定模式，
 * 当检测到模式变化时更新 g_work_mode 并上报。
 */
void gpio_work_mode_scan(void);
	
#endif
