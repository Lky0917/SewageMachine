#include "usr_system.h"

extern UART_HandleTypeDef huart1;

void USR_SystemInit(void)
{

}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

