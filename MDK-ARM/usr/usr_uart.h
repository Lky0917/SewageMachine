#ifndef __USR_UART_H
#define __USR_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
}
#endif

/** @brief UART1 单字节中断接收缓存。 */
extern uint8_t uart1_rx_byte;
/** @brief UART3 单字节中断接收缓存。 */
extern uint8_t uart3_rx_byte;

/**
 * @brief 轮询并处理 UART1 的一行文本命令。
 *
 * @details
 * 该函数应在主循环中周期调用。当中断接收拼出一行（以 '\n' 结尾）
 * 后，本函数消费该缓冲并分发命令。
 */
void uart1_rx_cmd(void);

#endif
