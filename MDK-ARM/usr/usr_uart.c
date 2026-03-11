#include "usr_uart.h"
#include "main.h"
#include "usr_pwm.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "bt_tuya.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern uint8_t ADC_on_off;

#define UART1_FRAME_MAX 256

/* UART 单字节中断接收缓存。 */
uint8_t uart1_rx_byte;
uint8_t uart3_rx_byte;
/* UART1 文本命令行缓存。 */
static char uart1_frame[UART1_FRAME_MAX];
static uint16_t uart1_len = 0;
static volatile uint8_t uart1_frame_ready = 0;

/**
 * @brief UART 接收完成回调（HAL）。
 *
 * @details
 * - USART1：累计一行文本，直到收到 '\n'。
 * - USART3：把字节流交给涂鸦 MCU SDK 解析器。
 * 中断中始终重新开启单字节接收。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t b;

  if (huart->Instance == USART1)
  {
    b = uart1_rx_byte;

    if (!uart1_frame_ready)
    {
      if (b == '\n')
      {
        if (uart1_len > 0 && uart1_frame[uart1_len - 1] == '\r')
        {
          uart1_len--;
        }

        if (uart1_len < UART1_FRAME_MAX)
          uart1_frame[uart1_len] = '\0';
        else
          uart1_frame[UART1_FRAME_MAX - 1] = '\0';

        uart1_frame_ready = 1;
      }
      else
      {
        if (uart1_len < (UART1_FRAME_MAX - 1))
        {
          uart1_frame[uart1_len++] = b;
        }
        else
        {
          uart1_len = 0;
        }
      }
    }

    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }

  if (huart->Instance == USART3)
  {
    /* 把原始字节流送入涂鸦协议解析。 */
    uart_receive_input(uart3_rx_byte);
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

/**
 * @brief 从 "MOTOR=" 或 "MOTOL=" 命令行解析 PWM 值。
 *
 * @param[out] out_val 解析后的整数值。
 * @retval 1 成功，0 失败。
 */
static uint8_t parse_motor_value_from_frame_ok(int32_t *out_val)
{
  const char *p = strstr(uart1_frame, "MOTOL=");
  if (p)
  {
    p += 6;
  }
  else
  {
    p = strstr(uart1_frame, "MOTOR=");
    if (!p)
      return 0;
    p += 6;
  }

  while (*p == ' ' || *p == '\t')
    p++;

  if (*p != '-' && !isdigit((unsigned char)*p))
    return 0;

  *out_val = (int32_t)atoi(p);
  return 1;
}

/**
 * @brief 解析 '=' 后的整数（允许前导空格）。
 *
 * @param[in]  s   '=' 后的字符串指针。
 * @param[out] out 解析后的整数值。
 * @retval 1 成功，0 失败。
 */
static uint8_t parse_int_after_equal(const char *s, int32_t *out)
{
  while (*s == ' ' || *s == '\t')
    s++;

  if (*s != '-' && !isdigit((unsigned char)*s))
    return 0;

  *out = (int32_t)atoi(s);
  return 1;
}

/**
 * @brief 分发一条已接收完成的 UART1 命令行。
 *
 * @details
 * 支持电机使能/方向/PWM、蓝牙功能、ADC 打印等命令。
 * 命令为以 '\n' 结束的 ASCII 字符串。
 */
static void uart1_handle_line(void)
{
  int moto_pwm = 0;

  if (uart1_frame[0] == '\0')
    return;

  if (strcmp(uart1_frame, "MOTO3=ON") == 0)
  {
    /* 使能 MOTO3（低电平有效）。 */
    HAL_GPIO_WritePin(MOTO3_EN_GPIO_Port, MOTO3_EN_Pin, GPIO_PIN_RESET);
    return;
  }
  if (strcmp(uart1_frame, "MOTO3=OFF") == 0)
  {
    /* 关闭 MOTO3。 */
    HAL_GPIO_WritePin(MOTO3_EN_GPIO_Port, MOTO3_EN_Pin, GPIO_PIN_SET);
    return;
  }
  if (strcmp(uart1_frame, "MOTO4=ON") == 0)
  {
    /* 使能 MOTO4（低电平有效）。 */
    HAL_GPIO_WritePin(MOTO4_EN_GPIO_Port, MOTO4_EN_Pin, GPIO_PIN_RESET);
    return;
  }
  if (strcmp(uart1_frame, "MOTO4=OFF") == 0)
  {
    /* 关闭 MOTO4。 */
    HAL_GPIO_WritePin(MOTO4_EN_GPIO_Port, MOTO4_EN_Pin, GPIO_PIN_SET);
    return;
  }
  if (strncmp(uart1_frame, "MOTOR=", 6) == 0)
  {
    moto_pwm = 0;
    parse_motor_value_from_frame_ok(&moto_pwm);
    /* 右电机 PWM（TIM1 CH2）。 */
    PWM_SetDutyCCR(MOTOR_PWM, moto_pwm);
    return;
  }
  if (strncmp(uart1_frame, "MOTOL=", 6) == 0)
  {
    moto_pwm = 0;
    parse_motor_value_from_frame_ok(&moto_pwm);
    /* 左电机 PWM（TIM1 CH4）。 */
    PWM_SetDutyCCR(MOTOL_PWM, moto_pwm);
    return;
  }
  if (strncmp(uart1_frame, "MOTOLD=", 7) == 0)
  {
    int32_t v;
    if (!parse_int_after_equal(uart1_frame + 7, &v))
      return;
    if (!(v == 0 || v == 1))
      return;
    /* 左电机方向 GPIO。 */
    HAL_GPIO_WritePin(MOTO2_DIR_GPIO_Port, MOTO2_DIR_Pin, v);

    return;
  }
  if (strncmp(uart1_frame, "MOTORD=", 7) == 0)
  {
    int32_t v;
    if (!parse_int_after_equal(uart1_frame + 7, &v))
      return;
    if (!(v == 0 || v == 1))
      return;
    /* 右电机方向 GPIO。 */
    HAL_GPIO_WritePin(MOTO1_DIR_GPIO_Port, MOTO1_DIR_Pin, v);

    return;
  }

  if (strcmp(uart1_frame, "BT_MODE_UP") == 0)
  {
    /* 在模式 4/5 间切换，并上报 DP。 */
    if (g_work_mode == 4)
      g_work_mode = 5;
    else if (g_work_mode == 5)
      g_work_mode = 4;
    else
      g_work_mode = 4;
    all_data_update();
    return;
  }
  if (strcmp(uart1_frame, "BT_UNBOUND") == 0)
  {
    /* 请求涂鸦模块解绑。 */
    bt_unbound_req();
    return;
  }

  if (strcmp(uart1_frame, "ADC_ON") == 0)
  {
    /* 打开 ADC 调试打印。 */
    ADC_on_off = 1;
    return;
  }
  if (strcmp(uart1_frame, "ADC_OFF") == 0)
  {
    /* 关闭 ADC 调试打印。 */
    ADC_on_off = 0;
    return;
  }
}

/**
 * @brief 轮询 UART1 行缓冲，准备好就执行命令。
 *
 * @details
 * 该函数应在主循环中周期调用；
 * 清除就绪标志、分发命令并重置缓冲。
 */
void uart1_rx_cmd(void)
{
  if (uart1_frame_ready)
  {
    uart1_frame_ready = 0;
    uart1_handle_line();
    printf("RX:%s\r\n", uart1_frame);
    uart1_len = 0;
  }
}
