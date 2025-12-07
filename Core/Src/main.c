/* IMPROVED IR CAPTURE CODE FOR DAIKIN160 */
/* Key improvements:
 * 1. Better timer resolution
 * 2. Reduced input filtering
 * 3. Tighter deviation tolerance
 * 4. Pulse width normalization
 * 5. Better bit classification
 * 6. RAW PWM DATA OUTPUT ADDED
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - IMPROVED FOR DAIKIN160
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#define samplecount 250
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint16_t end_width = 40000;
volatile int packet_flag = 0;
volatile int capture_cnt = 0;
volatile uint16_t pwm[2][samplecount];
volatile int diff_stats[3][10];
volatile int diff_count = 0;
volatile int num_packets = 0;

/* snapshot results filled by calibrate() and consumed by main() */
volatile uint8_t stats_ready = 0;
volatile int result_count = 0;
volatile int result_diff_count = 0;
volatile int result_stats[3][10];
volatile uint8_t normal_packet_done = 0;

volatile uint32_t last_irq_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// IMPROVEMENT 1: Better pulse width normalization
uint16_t normalize_pulse(uint16_t raw_width) {
    // Normalize to expected Daikin160 values
    if (raw_width >= 900 && raw_width <= 1200) {
        return 1042;  // Expected "0" bit (342 + 700)
    } else if (raw_width >= 1950 && raw_width <= 2300) {
        return 2128;  // Expected "1" bit (342 + 1786)
    } else if (raw_width >= 6800 && raw_width <= 7500) {
        return 7145;  // Expected header (5000 + 2145)
    } else if (raw_width >= 29000 && raw_width <= 30500) {
        return 29650; // Expected gap
    }
    return raw_width;  // Return as-is if doesn't match known patterns
}

// IMPROVEMENT 2: Tighter matching with normalized values
int match(int p_width, int deviation) {
    if (p_width < 0) {
        p_width *= -1;
    }
    return (p_width < deviation);
}

void calibrate(int count) {
    // Reset previous stats
    diff_count = 0;
    for (int i = 0; i < 10; ++i) {
        diff_stats[0][i] = 0;
        diff_stats[1][i] = 0;
        diff_stats[2][i] = 0;
    }

    // IMPROVEMENT 3: Reduced deviation for better classification
    int dev = 100;  // Reduced from 150 to 100µs

    for (int i = 0; i < count; ++i) {
        int pulse = pwm[1][i];
        int width = pwm[0][i];

        // IMPROVEMENT 4: Normalize before grouping
        width = normalize_pulse(width);

        int found = 0;
        for (int j = 0; j < diff_count; ++j) {
            int avg = (diff_stats[0][j] + (diff_stats[1][j] >> 1)) / diff_stats[1][j];
            if (match(width - avg, dev)) {
                diff_stats[0][j] += width;
                diff_stats[1][j]++;
                diff_stats[2][j] += pulse;
                found = 1;
                break;
            }
        }
        if (!found && diff_count < 10) {
            diff_stats[0][diff_count] = width;
            diff_stats[1][diff_count] = 1;
            diff_stats[2][diff_count] = pulse;
            diff_count++;
        }
    }

    // Publish snapshot
    __disable_irq();
    result_count = count;
    result_diff_count = diff_count;
    for (int i = 0; i < 10; ++i) {
        result_stats[0][i] = diff_stats[0][i];
        result_stats[1][i] = diff_stats[1][i];
        result_stats[2][i] = diff_stats[2][i];
    }
    stats_ready = 1;
    __enable_irq();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        int p_width = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        int p_mark  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if (packet_flag) {
            if (p_width >= end_width || capture_cnt >= samplecount - 2) {
                if (capture_cnt > 4)
                    normal_packet_done = 1;
                packet_flag = 0;
                last_irq_tick = HAL_GetTick();
            } else {
                pwm[0][capture_cnt] = p_width;
                pwm[1][capture_cnt] = p_mark;
                capture_cnt++;
                last_irq_tick = HAL_GetTick();
            }
        } else {
            // IMPROVEMENT 5: Better header detection for Daikin160
            // Header should be around 7145µs (5000 mark + 2145 space)
            if (p_width > 6500 && p_width < 8000) {
                packet_flag = 1;
                capture_cnt = 0;
                pwm[0][capture_cnt] = p_width;
                pwm[1][capture_cnt] = p_mark;
                capture_cnt++;
                last_irq_tick = HAL_GetTick();
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        // Not used
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  for (int i = 0; i < 10; ++i) {
    diff_stats[0][i] = diff_stats[1][i] = diff_stats[2][i] = 0;
  }

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  last_irq_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t tx_buff[1024];
  int local_count;
  int local_diff_count;
  int local_stats[3][10];
  int buff_len;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (normal_packet_done) {
        __disable_irq();
        normal_packet_done = 0;
        __enable_irq();

        if (capture_cnt > 4) {
            calibrate(capture_cnt);
        }
    }

    if (packet_flag && (HAL_GetTick() - last_irq_tick > 150)) {
        __disable_irq();
        if (capture_cnt > 4) {
            packet_flag = 0;
            calibrate(capture_cnt);
        } else {
            packet_flag = 0;
            capture_cnt = 0;
        }
        __enable_irq();
    }

    if (stats_ready) {
        __disable_irq();
        stats_ready = 0;
        local_count = result_count;
        local_diff_count = result_diff_count;
        for (int i = 0; i < 10; ++i) {
            local_stats[0][i] = result_stats[0][i];
            local_stats[1][i] = result_stats[1][i];
            local_stats[2][i] = result_stats[2][i];
        }
        __enable_irq();

        if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY) {
            buff_len = 0;
            buff_len += sprintf((char*)tx_buff + buff_len,
                              "\r\n=== DAIKIN160 CAPTURE (Normalized) ===\r\n");
            buff_len += sprintf((char*)tx_buff + buff_len,
                              "Capture count = %5d, Unique pulses = %d:\r\n",
                              local_count, local_diff_count);

            for (int i = 0; i < local_diff_count; ++i) {
                int cnt = local_stats[1][i];
                if (cnt <= 0) continue;
                int avg_w = (local_stats[0][i] + (cnt >> 1)) / cnt;
                int avg_p = (local_stats[2][i] + (cnt >> 1)) / cnt;

                // IMPROVEMENT 6: Add bit type annotation
                const char* type = "???";
                if (avg_w >= 900 && avg_w <= 1200) type = "BIT 0";
                else if (avg_w >= 1950 && avg_w <= 2300) type = "BIT 1";
                else if (avg_w >= 6800 && avg_w <= 7500) type = "HEADER";
                else if (avg_w >= 29000 && avg_w <= 30500) type = "GAP";

                buff_len += sprintf((char*)tx_buff + buff_len,
                                  "%4d µs [cnt=%2d, mark=%4d] -> %s\r\n",
                                  avg_w, cnt, avg_p, type);
            }

            // NEW: Output raw PWM capture data
            buff_len += sprintf((char*)tx_buff + buff_len,
                              "\r\n--- RAW CAPTURE DATA ---\r\n");
            buff_len += sprintf((char*)tx_buff + buff_len,
                              "Index | Width(µs) | Mark(µs)\r\n");

            // Send summary first
            HAL_UART_Transmit_DMA(&huart2, tx_buff, buff_len);
            while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
                HAL_Delay(1);
            }

            // Now send raw data in chunks
            for (int i = 0; i < local_count && i < samplecount; ++i) {
                buff_len = 0;

                // Read raw PWM values (read directly from volatile arrays)
                uint16_t width_val = pwm[0][i];
                uint16_t mark_val = pwm[1][i];

                buff_len += sprintf((char*)tx_buff + buff_len,
                                  "%5d | %9d | %8d\r\n",
                                  i, width_val, mark_val);

                HAL_UART_Transmit_DMA(&huart2, tx_buff, buff_len);
                while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
                    HAL_Delay(1);
                }
            }

            // Send final newline
            buff_len = sprintf((char*)tx_buff, "\r\n");
            HAL_UART_Transmit_DMA(&huart2, tx_buff, buff_len);
        }
    }
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function - IMPROVED
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;  // 84MHz / 84 = 1MHz = 1µs resolution
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;  // IMPROVEMENT: Max period for better long pulse capture
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;  // IMPROVEMENT: Reduced from 4 to 0 for better accuracy
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;  // IMPROVEMENT: Reduced from 4 to 2 - light filtering only
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;  // No filter on rising edge
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
