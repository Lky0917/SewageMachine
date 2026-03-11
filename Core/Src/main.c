/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usr_system.h"
#include "qmi8658.h"
#include "imu.h"
#include "usr_uart.h"
#include "usr_pwm.h"
#include "bt_tuya.h"
#include "hit.h"
#include "usr_adc.h"
#include "usr_key.h"
#include "usr_motor_ctrl.h"
#include "usr_time.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern volatile unsigned char system_flag_bool; // 系统开关状态记录变量
extern uint8_t g_battery_percentage;            // 系统当前电量百分比

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void battery_soc_report_task(void);
static void Led_SocAllOff(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief USART1 发送单字节（轮询方式）。
 *
 * @param c 要发送的字节
 */
void usart1_send_char(uint8_t c)
{
  // HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1, HAL_MAX_DELAY);
  while ((USART1->SR & USART_SR_TXE) == 0)
    ;
  USART1->DR = c;
}

/**
 * @brief 通过 USART1 发送自定义“匿名上报”帧。
 *
 * @param fun 功能码
 * @param data 数据指针
 * @param len 数据长度（最大 28）
 */
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
  uint8_t send_buf[32];
  uint8_t i;

  if (len > 28)
  {
    return;
  }

  send_buf[len + 4] = 0;
  send_buf[0] = 0XAA;
  send_buf[1] = 0XAA;
  send_buf[2] = fun;
  send_buf[3] = len;

  for (i = 0; i < len; i++)
  {
    send_buf[4 + i] = data[i];
  }

  for (i = 0; i < len + 4; i++)
  {
    send_buf[len + 4] += send_buf[i];
  }

  for (i = 0; i < len + 5; i++)
  {
    usart1_send_char(send_buf[i]);
  }
}

/**
 * @brief 打包并发送 QMI8658 采样数据（加速度/陀螺仪）。
 *
 * @param aacx 加速度 X
 * @param aacy 加速度 Y
 * @param aacz 加速度 Z
 * @param gyrox 角速度 X
 * @param gyroy 角速度 Y
 * @param gyroz 角速度 Z
 */
void qmi8658_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
  uint8_t tbuf[18];
  tbuf[0] = (aacx >> 8) & 0XFF;
  tbuf[1] = aacx & 0XFF;
  tbuf[2] = (aacy >> 8) & 0XFF;
  tbuf[3] = aacy & 0XFF;
  tbuf[4] = (aacz >> 8) & 0XFF;
  tbuf[5] = aacz & 0XFF;
  tbuf[6] = (gyrox >> 8) & 0XFF;
  tbuf[7] = gyrox & 0XFF;
  tbuf[8] = (gyroy >> 8) & 0XFF;
  tbuf[9] = gyroy & 0XFF;
  tbuf[10] = (gyroz >> 8) & 0XFF;
  tbuf[11] = gyroz & 0XFF;

  tbuf[12] = 0;
  tbuf[13] = 0;
  tbuf[14] = 0;
  tbuf[15] = 0;
  tbuf[16] = 0;
  tbuf[17] = 0;
  usart1_niming_report(0X02, tbuf, 18);
}

/**
 * @brief 发送 IMU 姿态角度与状态信息。
 *
 * @param roll  横滚角（放大后）
 * @param pitch 俯仰角（放大后）
 * @param yaw   航向角（放大后）
 * @param prs   压力/高度等扩展值（当前为 0）
 * @param fly_mode 飞行/工作模式
 * @param armed 解锁标志
 */
void usart1_report_imu(short roll, short pitch, short yaw, int prs, uint8_t fly_mode, uint8_t armed)
{
  uint8_t tbuf[12];

  tbuf[0] = (roll >> 8) & 0XFF;
  tbuf[1] = roll & 0XFF;
  tbuf[2] = (pitch >> 8) & 0XFF;
  tbuf[3] = pitch & 0XFF;
  tbuf[4] = (yaw >> 8) & 0XFF;
  tbuf[5] = yaw & 0XFF;
  tbuf[6] = (prs >> 24) & 0XFF;
  tbuf[7] = (prs >> 16) & 0XFF;
  tbuf[8] = (prs >> 8) & 0XFF;
  tbuf[9] = prs & 0XFF;
  tbuf[10] = fly_mode;
  tbuf[11] = armed;
  usart1_niming_report(0X01, tbuf, 12); /* 设备号，0X01 */
}

/**
 * @brief 读取 IMU 数据并通过串口上报（调试用）。
 */
void qmi8658_test()
{
  float gyro[3];
  float accel[3];
  float euler_angle[3] = {0, 0, 0};

  qmi8658_read_xyz(accel, gyro);
  if (g_imu_init)
  {
    imu_get_eulerian_angles(accel, gyro, euler_angle, IMU_DELTA_T);
    // printf("%f\t%f\t%f\r\n", euler_angle[0],euler_angle[1],euler_angle[2]);

    qmi8658_send_data(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
    usart1_report_imu((int)(euler_angle[1] * 100), (int)(euler_angle[0] * 100), (int)(euler_angle[2] * 100), 0, 0, 0);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); /* 右电机 PWM 输出 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* 左电机 PWM 输出 */
  HAL_TIM_Base_Start_IT(&htim2);            /* TIM2 1ms 定时中断 */
  UsrTime_Init();                           /* 用户时间模块初始化 */
  MotorCtrl_Init();                         /* 电机控制模块初始化 */

  printf("SewageMachine-v0.1\r\n"); /* 启动信息打印 */
  bt_protocol_init();               /* 涂鸦蓝牙协议初始化 */

  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1); /* 启动 USART3 单字节接收中断 */
  Key_Init();                                      /* 按键模块初始化 */

  uint32_t wait_log_tick = HAL_GetTick();
  while (!system_flag_bool)
  {
    // 等待系统开关打开
    bt_uart_service();    /* 处理涂鸦协议串口数据 */
    bt_state_poll_task(); /* 轮询蓝牙连接状态 */
    Key_Task();           /* 按键状态机 */
    Key_Scan();           /* 按键扫描处理 */
    MotorCtrl_StopAll();  /* 关机态保持所有电机关闭 */
    Led_SocAllOff();      /* 关机态关闭电量指示灯 */

    if ((HAL_GetTick() - wait_log_tick) >= 200u)
    {
      wait_log_tick = HAL_GetTick();
      printf("System is off, waiting to turn on...\r\n");
    }
    HAL_Delay(5);
  }

  while (qmi8658_init() != 0) /* QMI8658A初始化 */
  {
    printf("QMI8658A Error!\r\n");
  }
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);                 /* 启动 USART1 单字节接收中断 */
  HAL_ADCEx_Calibration_Start(&hadc1);                             /* ADC 校准 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_LEN); /* 启动 ADC DMA */

  g_work_mode = 4; /* 默认工作模式 4 */

  all_data_update(); /* 启动时发送全量 DP 上报 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // qmi8658_test(); /* IMU 数据串口上报（调试用，当前关闭） */

    Key_Task(); /* 按键状态机 */
    Key_Scan(); /* 按键扫描处理 */

    uart1_rx_cmd();       /* 处理 UART1 文本命令 */
    bt_uart_service();    /* 处理涂鸦协议串口数据 */
    bt_state_poll_task(); /* 轮询蓝牙连接状态 */
    MotorCtrl_Task();     /* 电机任务 */
    UsrTime_Poll();       /* 定时到期回调（主循环执行，避免中断竞态） */

    if (!system_flag_bool)
    {
      MotorCtrl_StopAll(); /* 关机后立即停止全部电机 */
      Led_SocAllOff();     /* 关机后关闭电量指示灯 */
      HAL_Delay(50);
      continue;
    }

    gpio_work_mode_scan();     /* 根据 GPIO 判断前后左右碰撞 */
    adc_average_thread();      /* ADC 数据平均与换算 */
    battery_soc_report_task(); /* 电池电量百分比上报任务 */

    // HAL_Delay(20); /* 如需节拍可开启 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // HAL_GPIO_WritePin(GPIOC, MOTO1_BK_Pin | MOTO1_DIR_Pin | MOTO4_BK_Pin | MOTO4_DIR_Pin | MOTO2_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, MOTO1_BK_Pin | MOTO1_DIR_Pin | MOTO4_BK_Pin | MOTO2_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_POWER_GPIO_Port, LED_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_SOC1_Pin | LED_SOC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_SOC3_Pin | LED_SOC4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  // HAL_GPIO_WritePin(GPIOB, MOTO3_BK_Pin | MOTO3_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, MOTO3_BK_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTO4_EN_Pin | MOTO3_EN_Pin | MOTO2_BK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MOTO1_BK_Pin MOTO1_DIR_Pin MOTO4_BK_Pin MOTO4_DIR_Pin
                           MOTO2_DIR_Pin */
  GPIO_InitStruct.Pin = MOTO1_BK_Pin | MOTO1_DIR_Pin | MOTO4_BK_Pin | MOTO4_DIR_Pin | MOTO2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_IN_OFF_Pin */
  GPIO_InitStruct.Pin = KEY_IN_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_IN_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_POWER_Pin */
  GPIO_InitStruct.Pin = LED_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SOC1_Pin LED_SOC2_Pin */
  GPIO_InitStruct.Pin = LED_SOC1_Pin | LED_SOC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SOC3_Pin LED_SOC4_Pin */
  GPIO_InitStruct.Pin = LED_SOC3_Pin | LED_SOC4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QMI8658A_INT_Pin HIT_IN2_Pin HIT_IN3_Pin HIT_IN4_Pin */
  GPIO_InitStruct.Pin = QMI8658A_INT_Pin | HIT_IN2_Pin | HIT_IN3_Pin | HIT_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTO3_BK_Pin MOTO3_DIR_Pin */
  GPIO_InitStruct.Pin = MOTO3_BK_Pin | MOTO3_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTO4_EN_Pin MOTO3_EN_Pin */
  GPIO_InitStruct.Pin = MOTO4_EN_Pin | MOTO3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTO2_BK_Pin */
  GPIO_InitStruct.Pin = MOTO2_BK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTO2_BK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HIT_IN1_Pin */
  GPIO_InitStruct.Pin = HIT_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HIT_IN1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // // 行走电机刹车
  // HAL_GPIO_WritePin(MOTO1_BK_GPIO_Port, MOTO1_BK_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MOTO2_BK_GPIO_Port, MOTO2_BK_Pin, GPIO_PIN_RESET);

  // 抽水电机方向控制
  HAL_GPIO_WritePin(MOTO3_DIR_GPIO_Port, MOTO3_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTO4_DIR_GPIO_Port, MOTO4_DIR_Pin, GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Led_SocAllOff(void)
{
  HAL_GPIO_WritePin(LED_SOC1_GPIO_Port, LED_SOC1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC2_GPIO_Port, LED_SOC2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC3_GPIO_Port, LED_SOC3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC4_GPIO_Port, LED_SOC4_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 实时电量上传函数
 *
 * @param batt_v 电池电压值
 *
 */
static uint8_t BatterySoc_FromVoltage(float batt_v)
{
  float soc_f = ((batt_v - 20.65f) / 1.05f) * 25.0f;

  if (soc_f <= 0.0f)
  {
    return 0;
  }
  if (soc_f >= 100.0f)
  {
    return 100;
  }
  return (uint8_t)(soc_f + 0.5f);
}

/**
 * @brief 更新LED指示灯状态
 *
 * @param soc
 */
static void BatterySoc_UpdateLeds(uint8_t soc)
{
  uint8_t level = 0;

  if (soc > 75)
  {
    level = 4;
    // printf("Battery SOC: %d%% (Full)\r\n", soc);
  }
  else if (soc > 50)
  {
    level = 3;
    // printf("Battery SOC: %d%% (High)\r\n", soc);
  }
  else if (soc > 25)
  {
    level = 2;
    // printf("Battery SOC: %d%% (Medium)\r\n", soc);
  }
  else if (soc > 0)
  {
    level = 1;
    // printf("Battery SOC: %d%% (Low)\r\n", soc);
  }

  HAL_GPIO_WritePin(LED_SOC1_GPIO_Port, LED_SOC1_Pin, (level >= 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC2_GPIO_Port, LED_SOC2_Pin, (level >= 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC3_GPIO_Port, LED_SOC3_Pin, (level >= 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_SOC4_GPIO_Port, LED_SOC4_Pin, (level >= 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 电池电量上报任务
 *
 */
void battery_soc_report_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t last_soc = 0xFF;
  uint32_t now = HAL_GetTick();
  uint8_t soc;

  if ((now - last_tick) < 500u)
  {
    return;
  }
  last_tick = now;

  soc = BatterySoc_FromVoltage(Adc_GetBatteryVoltage());
  g_battery_percentage = soc;
  if (!system_flag_bool)
  {
    Led_SocAllOff();
  }
  else
  {
    BatterySoc_UpdateLeds(soc);
  }

  if (soc != last_soc)
  {
    all_data_update();
    last_soc = soc;
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
