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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include "pv_st_f411.h"
#include "stm32f4xx_hal.h"

// --- PORCUPINE INCLUDES ---
#include "pv_porcupine_mcu.h"
#include "keyword_params.h" // Your Alexa/Keyword header
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    IR_IDLE = 0,
    IR_SENDING_MARK,
    IR_SENDING_SPACE
} IR_State_t;

typedef struct {
    uint16_t mark_us;
    uint16_t space_us;
} IR_Pulse_t;

typedef enum {
    CMD_NONE = 0,
    CMD_WAKE_WORD_DETECTED,
    CMD_UART_MSG_READY
} AppCommand_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- AUDIO CONFIG ---
#define SAMPLE_RATE 16000
#define FRAME_SIZE 512
#define CHANNELS 2
#define I2S_BUFFER_SIZE (FRAME_SIZE * CHANNELS * 2)
#define MEMORY_BUFFER_SIZE (1024 * 20)

// --- DAIKIN / HW CONFIG ---
#define DHT11_PORT GPIOB
#define DHT11_PIN  GPIO_PIN_0

#define DAIKIN176_HDR_MARK      5070
#define DAIKIN176_HDR_SPACE     2140
#define DAIKIN176_BIT_MARK      370
#define DAIKIN176_ONE_SPACE     1780
#define DAIKIN176_ZERO_SPACE    710
#define DAIKIN176_GAP           29410
#define DAIKIN176_STATE_LENGTH  22
#define DAIKIN176_SECTION1_LEN  7
#define MAX_PULSES              300

// AC Constants
#define DAIKIN176_MODE_FAN      0
#define DAIKIN176_MODE_HEAT     1
#define DAIKIN176_MODE_COOL     2
#define DAIKIN176_MODE_AUTO     3
#define DAIKIN176_MODE_DRY      7

#define DAIKIN176_FAN_LOW       1
#define DAIKIN176_FAN_MED       2
#define DAIKIN176_FAN_HIGH      3
#define DAIKIN176_SWING_AUTO    0x5
#define DAIKIN176_SWING_OFF     0x6
#define DAIKIN176_DRY_FAN_TEMP  17
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId AudioTaskHandle;
osThreadId ControlTaskHandle;
osMessageQId commandQueueHandle;
/* USER CODE BEGIN PV */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

// --- HARDWARE HANDLES ---
TIM_HandleTypeDef htim3; // Manually added as it was missing in generated code

// --- AUDIO DATA ---
uint16_t i2s_rx_buffer[I2S_BUFFER_SIZE];
int16_t mono_pcm_buffer[FRAME_SIZE];
volatile uint8_t half_transfer_flag = 0;
volatile uint8_t full_transfer_flag = 0;

//static const char* ACCESS_KEY = "1tRtNkoHDZntOJsz9hWww1pEZ7Ox8nzMBa14btCoTyUTSTBFRxgd/A==";
static const char* ACCESS_KEY = "cTKsua4o2+I4XCobOlBW3ujPKjrQAV8W6ZoyLiBfsdZgJbtYkuPqZQ==";
static uint8_t memory_buffer[MEMORY_BUFFER_SIZE] __attribute__((aligned(16)));
const float sensitivity = 0.7f;
pv_porcupine_t *porcupine_handle = NULL;

// --- AC / UART DATA ---
uint8_t rx_buffer[1];
char msg_buffer[50];
int msg_index = 0;
volatile uint8_t msg_ready = 0;
char tx_buffer[200];

// AC Internal State
uint8_t ac_power = 0;
uint8_t ac_mode = DAIKIN176_MODE_COOL;
uint8_t ac_fan = DAIKIN176_FAN_MED;
uint8_t ac_temp = 25;
uint8_t ac_swing = 0;

uint8_t R_H, R_D, T_H, T_D, Sum;

// IR State
uint8_t daikin176_state[DAIKIN176_STATE_LENGTH] = {
    0x11, 0xDA, 0x17, 0x18, 0x04, 0x00, 0xC4,
    0x11, 0xDA, 0x17, 0x18, 0x00, 0x73, 0x00,
    0x20, 0x00, 0x00, 0x32, 0x60, 0x00, 0x00, 0x00
};
volatile IR_State_t ir_state = IR_IDLE;
volatile IR_Pulse_t pulse_buffer[MAX_PULSES];
volatile uint16_t pulse_count = 0;
volatile uint16_t pulse_index = 0;
volatile bool ir_busy = false;

// DHT11
uint8_t temp_int = 0, hum_int = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void StartAudioTask(void const * argument);
void StartControlTask(void const * argument);

/* USER CODE BEGIN PFP */

void AC_SetMode(uint8_t mode);
void AC_SetFan(uint8_t speed);
void AC_SetSwing(uint8_t on);
const char* AC_GetModeString(uint8_t mode);
const char* AC_GetFanString(uint8_t fan);
bool IR_IsBusy(void);
// AC/IR Functions
void AC_SetPower(uint8_t on);
void AC_SetTemp(uint8_t temp);
void AC_SendCommand(void);
void AC_UpdateState(void);
void IR_BuildDaikin176(void);
void IR_StartTransmission(void);
void IR_AddPulse(uint16_t mark_us, uint16_t space_us);
uint8_t IR_CalculateChecksum(uint8_t *data, uint8_t length);
uint8_t DHT11_Read_Data(void);
void DWT_Delay_us(volatile uint32_t microseconds);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//pv_board_init();

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
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  // 1. Initialize Manual Hardware (TIM3 & DWT)


    // 2. RE-CONFIGURE TIM5 for 38kHz (Your generated code set it to MAX)
    // Assuming 72MHz or 84MHz clock. This sets period to ~26us
    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, 2210); // Adjust based on your APB1 clock!
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1105); // 50% Duty
    pv_board_init();
     const uint8_t *board_uuid = pv_get_uuid();
//
//    	char uart_buf[64];
//    	int len;
//
//    	// 1. Print Opening Separator
//    	len = sprintf(uart_buf, "\r\n============================\r\n");
//    	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
//
//    	// 2. Print UUID Label
//    	len = sprintf(uart_buf, "UUID: ");
//    	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
//
//    	// 3. Print UUID Bytes Loop
//    	for (uint32_t i = 0; i < pv_get_uuid_size(); i++) {
//    	    len = sprintf(uart_buf, " %.2x", board_uuid[i]);
//    	    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
//    	}
//
//    	// 4. Print Closing Separator
//    	len = sprintf(uart_buf, "\r\n============================\r\n");
//    	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
    printf("\r\n============================\r\n");

        printf("UUID: ");

        for (uint32_t i = 0; i < pv_get_uuid_size(); i++) {

            printf(" %.2x", board_uuid[i]);

        }

        printf("\r\n============================\r\n");

    // 3. Porcupine Init
    const int32_t keyword_model_sizes = sizeof(KEYWORD_ARRAY);
    const void *keyword_models = KEYWORD_ARRAY;
    pv_status_t status = pv_porcupine_init(
            ACCESS_KEY, MEMORY_BUFFER_SIZE, memory_buffer,
            1, &keyword_model_sizes, &keyword_models,
            &sensitivity, &porcupine_handle);

    if (status != PV_STATUS_SUCCESS) {
    	char err_msg[32];
    	    // Print the specific status code (e.g., 6, 7, 1)
    	    sprintf(err_msg, "PV_ERR: %d\r\n", status);
    	    HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 100);
    }

    // 4. Start Listening
    AC_UpdateState();
    IR_BuildDaikin176();
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    HAL_I2S_Receive_DMA(&hi2s2, i2s_rx_buffer, I2S_BUFFER_SIZE);

//    // 5. Create RTOS Objects
//    osMessageQDef(cmdQ, 5, uint8_t);
//    commandQueueHandle = osMessageCreate(osMessageQ(cmdQ), NULL);
//
//    osThreadDef(AudioTask, StartAudioTask, osPriorityAboveNormal, 0, 4096);
//    AudioTaskHandle = osThreadCreate(osThread(AudioTask), NULL);
//
//    osThreadDef(ControlTask, StartControlTask, osPriorityNormal, 0, 1024);
//    ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

    char msg[] = "\r\nstarting\r\n";
                            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    // Note: We ignore defaultTask created below, it will just yield.
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of commandQueue */
  osMessageQDef(commandQueue, 10, uint8_t);
  commandQueueHandle = osMessageCreate(osMessageQ(commandQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of AudioTask */
  osThreadDef(AudioTask, StartAudioTask, osPriorityNormal, 0, 2048);
  AudioTaskHandle = osThreadCreate(osThread(AudioTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2210;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1105;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  huart1.Init.BaudRate = 115200;
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

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- TASKS ---


// --- CALLBACKS ---

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) { half_transfer_flag = 1; }
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) { full_transfer_flag = 1; }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        char received = (char)rx_buffer[0];

        // Check for Newline or Carriage Return
        if (received == '\n' || received == '\r') {
            // ONLY process if we actually have data in the buffer
            if (msg_index > 0) {
                msg_buffer[msg_index] = '\0'; // Null terminate the string
                msg_index = 0;                // Reset index IMMEDIATELY to prevent double trigger

                // Send to queue
                osMessagePut(commandQueueHandle, CMD_UART_MSG_READY, 0);
            }
            // If msg_index is 0, it means we just processed the \r,
            // so we ignore the following \n.
        }
        else {
            // It's a normal character, add to buffer
            if (msg_index < 49) {
                msg_buffer[msg_index++] = received;
            }
        }

        // Restart Interrupt
        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (ir_state == IR_SENDING_MARK) {
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
            if (pulse_buffer[pulse_index].space_us > 0) {
                ir_state = IR_SENDING_SPACE;
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                __HAL_TIM_SET_AUTORELOAD(&htim3, pulse_buffer[pulse_index].space_us);
            } else {
                HAL_TIM_Base_Stop_IT(&htim3);
                ir_state = IR_IDLE;
                ir_busy = false;
            }
        } else if (ir_state == IR_SENDING_SPACE) {
            pulse_index++;
            if (pulse_index >= pulse_count) {
                HAL_TIM_Base_Stop_IT(&htim3);
                ir_state = IR_IDLE;
                ir_busy = false;
            } else {
                ir_state = IR_SENDING_MARK;
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                __HAL_TIM_SET_AUTORELOAD(&htim3, pulse_buffer[pulse_index].mark_us);
            }
        }
    }
}


void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
// AC Helper Functions
void AC_SetMode(uint8_t mode) {
    ac_mode = mode;
}

void AC_SetFan(uint8_t speed) {
    if (speed >= DAIKIN176_FAN_LOW && speed <= DAIKIN176_FAN_HIGH) {
        ac_fan = speed;
    }
}

void AC_SetSwing(uint8_t on) {
    ac_swing = on ? 1 : 0;
}

const char* AC_GetModeString(uint8_t mode) {
    switch (mode) {
        case DAIKIN176_MODE_COOL: return "cool";
        case DAIKIN176_MODE_HEAT: return "heat";
        case DAIKIN176_MODE_AUTO: return "auto";
        case DAIKIN176_MODE_DRY:  return "dry";
        case DAIKIN176_MODE_FAN:  return "fan";
        default: return "unknown";
    }
}

const char* AC_GetFanString(uint8_t fan) {
    switch (fan) {
        case DAIKIN176_FAN_LOW:  return "low";
        case DAIKIN176_FAN_MED:  return "medium";
        case DAIKIN176_FAN_HIGH: return "high";
        default: return "unknown";
    }
}

bool IR_IsBusy(void) {
    return ir_busy;
}
// --- AC IMPLEMENTATION ---

void AC_SetPower(uint8_t on) { ac_power = on ? 1 : 0; }
void AC_SetTemp(uint8_t temp) { if(temp >= 18 && temp <= 32) ac_temp = temp; }

void AC_UpdateState(void) {
    daikin176_state[14] = (ac_power & 0x01) | ((ac_mode & 0x07) << 4);
    uint8_t temp_val = (ac_mode == DAIKIN176_MODE_DRY || ac_mode == DAIKIN176_MODE_FAN) ? DAIKIN176_DRY_FAN_TEMP : ac_temp;
    daikin176_state[17] = (daikin176_state[17] & 0x81) | ((temp_val & 0x3F) << 1);
    uint8_t swing_val = ac_swing ? DAIKIN176_SWING_AUTO : DAIKIN176_SWING_OFF;
    daikin176_state[18] = (swing_val & 0x0F) | ((ac_fan & 0x0F) << 4);
}

void IR_BuildDaikin176(void) {
    pulse_count = 0;
    // Calculate Checksums
    uint8_t sum1 = 0; for(int i=0; i<6; i++) sum1 += daikin176_state[i]; daikin176_state[6] = sum1;
    uint8_t sum2 = 0; for(int i=7; i<21; i++) sum2 += daikin176_state[i]; daikin176_state[21] = sum2;

    IR_AddPulse(DAIKIN176_HDR_MARK, DAIKIN176_HDR_SPACE);
    for (uint8_t i = 0; i < DAIKIN176_SECTION1_LEN; i++) {
        uint8_t byte = daikin176_state[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            IR_AddPulse(DAIKIN176_BIT_MARK, (byte & 0x01) ? DAIKIN176_ONE_SPACE : DAIKIN176_ZERO_SPACE);
            byte >>= 1;
        }
    }
    IR_AddPulse(DAIKIN176_BIT_MARK, DAIKIN176_GAP);
    IR_AddPulse(DAIKIN176_HDR_MARK, DAIKIN176_HDR_SPACE);
    for (uint8_t i = DAIKIN176_SECTION1_LEN; i < DAIKIN176_STATE_LENGTH; i++) {
        uint8_t byte = daikin176_state[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            IR_AddPulse(DAIKIN176_BIT_MARK, (byte & 0x01) ? DAIKIN176_ONE_SPACE : DAIKIN176_ZERO_SPACE);
            byte >>= 1;
        }
    }
    IR_AddPulse(DAIKIN176_BIT_MARK, 0);
}

void IR_StartTransmission(void) {
    if (ir_busy) return;
    ir_busy = true;
    pulse_index = 0;
    ir_state = IR_SENDING_MARK;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, pulse_buffer[0].mark_us);
    HAL_TIM_Base_Start_IT(&htim3);
}

void IR_AddPulse(uint16_t mark_us, uint16_t space_us) {
    if (pulse_count < MAX_PULSES) {
        pulse_buffer[pulse_count].mark_us = mark_us;
        pulse_buffer[pulse_count].space_us = space_us;
        pulse_count++;
    }
}

void AC_SendCommand(void) { AC_UpdateState(); IR_BuildDaikin176(); IR_StartTransmission(); }


void DWT_Delay_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

// DHT11 Helper (Simple DWT)
void DWT_Delay_us(volatile uint32_t microseconds) {
    uint32_t clk_cycle_start = DWT->CYCCNT;
    microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);

    // CHANGE: Use osDelay instead of HAL_Delay to let AudioTask run during this long wait
    osDelay(18);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
    DWT_Delay_us(20);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

// Helper to read with timeout
// Returns -1 on timeout, 0 or 1 on success
int Wait_For_Pin_State(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state, uint32_t timeout_iters) {
    while (HAL_GPIO_ReadPin(port, pin) != state) {
        if (timeout_iters-- == 0) return -1;
    }
    return 0;
}

uint8_t DHT11_Check_Response(void) {
    uint8_t response = 0;
    DWT_Delay_us(40);

    // Add timeouts to these checks
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        DWT_Delay_us(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) response = 1;
    }

    // Safety timeout prevents infinite loop
    if (Wait_For_Pin_State(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET, 10000) < 0) return 0; // Error

    return response;
}

uint8_t DHT11_Read_Byte(void) {
    uint8_t i = 0, j;
    for (j = 0; j < 8; j++) {
        // Wait for pin to go high (start of bit) with timeout
        if(Wait_For_Pin_State(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET, 10000) < 0) return 0; // Timeout error

        DWT_Delay_us(40);

        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
            i &= ~(1 << (7 - j));
        } else {
            i |= (1 << (7 - j));
        }

        // Wait for pin to go low (end of bit) with timeout
        if(Wait_For_Pin_State(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET, 10000) < 0) return 0; // Timeout error
    }
    return i;
}

uint8_t DHT11_Read_Data(void) {
    // 1. Send Start Signal (Allow context switching here via osDelay)
    DHT11_Start();

    // 2. Enter Critical Section
    // We DISABLE interrupts so AudioTask cannot interrupt the sensitive timing
    taskENTER_CRITICAL();

    if (DHT11_Check_Response()) {
        R_H = DHT11_Read_Byte();
        R_D = DHT11_Read_Byte();
        T_H = DHT11_Read_Byte();
        T_D = DHT11_Read_Byte();
        Sum = DHT11_Read_Byte();

        // 3. Exit Critical Section immediately after reading
        taskEXIT_CRITICAL();

        if (Sum == (R_H + R_D + T_H + T_D)) {
            temp_int = T_H;
            hum_int = R_H;
            return 1;
        }
    } else {
        // Ensure we exit critical section even if response failed
        taskEXIT_CRITICAL();
    }
    return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
* @brief Function implementing the AudioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioTask */
	while(1) {
	        bool process = false;
	        uint16_t* buffer_ptr = NULL;

	        if (half_transfer_flag) {
	            half_transfer_flag = 0;
	            buffer_ptr = &i2s_rx_buffer[0];
	            process = true;
	        } else if (full_transfer_flag) {
	            full_transfer_flag = 0;
	            buffer_ptr = &i2s_rx_buffer[I2S_BUFFER_SIZE / 2];
	            process = true;
	        }

	        if (process && buffer_ptr) {
	            for (int i = 0; i < FRAME_SIZE; i++) {
	                mono_pcm_buffer[i] = (int16_t)buffer_ptr[i * 2];
	            }
	            int32_t keyword_index = -1;
	            pv_status_t status = pv_porcupine_process(porcupine_handle, mono_pcm_buffer, &keyword_index);

	            if (status == PV_STATUS_SUCCESS && keyword_index != -1) {
	                // Wake Word Detected!
	            	char msg[] = "\r\nWord Detected\r\n";
	            	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	                osMessagePut(commandQueueHandle, CMD_WAKE_WORD_DETECTED, 0);
	                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	            }
	        }
	        osDelay(1);
	    }
  /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
bool flagOn = 1;
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
    osEvent evt;
    while(1) {
        evt = osMessageGet(commandQueueHandle, 200); // 200ms wait
        if (evt.status == osEventMessage) {
            uint8_t cmd = (uint8_t)evt.value.v;
            if (cmd == CMD_WAKE_WORD_DETECTED) {
            	if (flagOn) {
            		AC_SetPower(1);
            		                AC_SendCommand();
            		                flagOn = !flagOn;
            		                sprintf(tx_buffer, "{\"ac\":\"wake-word\",\"power\":\"on\"}\r\n");
            	} else {
            		AC_SetPower(0);
            		                AC_SendCommand();
            		                flagOn = !flagOn;
            		                sprintf(tx_buffer, "{\"ac\":\"wake-word\",\"power\":\"off\"}\r\n");
            	}


                HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
                HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
            }
            else if (cmd == CMD_UART_MSG_READY) {
                for (int i = 0; msg_buffer[i]; i++) msg_buffer[i] = tolower((unsigned char)msg_buffer[i]);

                uint8_t send_ir = 0;

                // Power commands
                if (strstr(msg_buffer, "power-on")) {
                    AC_SetPower(1);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"power-on\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "power-off")) {
                    AC_SetPower(0);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"power-off\",\"status\":\"ok\"}\r\n");
                }
                // Fan commands
                else if (strstr(msg_buffer, "fan-low")) {
                    AC_SetFan(DAIKIN176_FAN_LOW);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"fan-low\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "fan-med")) {
                    AC_SetFan(DAIKIN176_FAN_MED);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"fan-med\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "fan-high")) {
                    AC_SetFan(DAIKIN176_FAN_HIGH);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"fan-high\",\"status\":\"ok\"}\r\n");
                }
                // Mode commands
                else if (strstr(msg_buffer, "mode-cool")) {
                    AC_SetMode(DAIKIN176_MODE_COOL);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"mode-cool\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "mode-heat")) {
                    AC_SetMode(DAIKIN176_MODE_HEAT);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"mode-heat\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "mode-auto")) {
                    AC_SetMode(DAIKIN176_MODE_AUTO);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"mode-auto\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "mode-dry")) {
                    AC_SetMode(DAIKIN176_MODE_DRY);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"mode-dry\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "mode-fan")) {
                    AC_SetMode(DAIKIN176_MODE_FAN);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"mode-fan\",\"status\":\"ok\"}\r\n");
                }
                // Temperature command
                else if (strncmp(msg_buffer, "temp-", 5) == 0) {
                    int t = atoi(&msg_buffer[5]);
                    if (t >= 18 && t <= 32) {
                        AC_SetTemp((uint8_t)t);
                        send_ir = 1;
                        sprintf(tx_buffer, "{\"ac\":\"temp-%d\",\"status\":\"ok\"}\r\n", t);
                    } else {
                        sprintf(tx_buffer, "{\"ac\":\"temp\",\"error\":\"range 18-32\"}\r\n");
                    }
                }
                // Swing commands
                else if (strstr(msg_buffer, "swing-on")) {
                    AC_SetSwing(1);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"swing-on\",\"status\":\"ok\"}\r\n");
                }
                else if (strstr(msg_buffer, "swing-off")) {
                    AC_SetSwing(0);
                    send_ir = 1;
                    sprintf(tx_buffer, "{\"ac\":\"swing-off\",\"status\":\"ok\"}\r\n");
                }
                // Status command
                else if (strstr(msg_buffer, "status")) {
                    sprintf(tx_buffer,
                        "{\"power\":\"%s\",\"mode\":\"%s\",\"fan\":\"%s\",\"temp\":%d,"
                        "\"swing\":\"%s\",\"status\":\"ok\"}\r\n",
                        ac_power ? "on" : "off",
                        AC_GetModeString(ac_mode),
                        AC_GetFanString(ac_fan),
                        ac_temp,
                        ac_swing ? "on" : "off");
                }
                // DHT11 Report
                else if (strstr(msg_buffer, "report")) {
                    if (DHT11_Read_Data()) {
                        sprintf(tx_buffer,
                            "{\"temperature\":%d,\"humidity\":%d,\"unit\":\"C\",\"status\":\"ok\"}\r\n",
                            temp_int, hum_int);
                    } else {
                        sprintf(tx_buffer,
                            "{\"temperature\":null,\"humidity\":null,\"status\":\"error\"}\r\n");
                    }
                }
                else {
                    sprintf(tx_buffer, "{\"error\":\"unknown_command\"}\r\n");
                }

                // Send IR command if needed
                if (send_ir && !ir_busy) {
                    AC_SendCommand();
                }

                HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
                HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

//                msg_ready = 0;
//                msg_index = 0;
            }
        }
    }
  /* USER CODE END StartControlTask */
}

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
