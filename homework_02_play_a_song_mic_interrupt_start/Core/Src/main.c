/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DO4   3206
#define DO4_  3032
#define RE4   2857
#define RE4_  2700
#define MI4   2545
#define FA4   2406
#define FA4_  2270
#define SOL4  2142
#define SOL4_ 2024
#define LA4   1909
#define LA4_  1802
#define SI4   1700

#define DO5   1606
#define DO5_  1516
#define RE5   1431
#define RE5_  1350
#define MI5   1274
#define FA5   1203
#define FA5_  1135
#define SOL5  1072
#define SOL5_ 1012
#define LA5   954
#define LA5_  901
#define SI5   851

#define PAUSE 0

#define TEMPO 1000	// time of one bar (battuta del pentagramma) in ms

#define SONG_SELECTOR 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct note {
	int period;
	int delay;
	struct note* next;
}note;

// first note of the song, it is not null when the linked list for the song has been created
struct note* head = NULL;

// represents the activation of the interrupt of the microphone, set to 1 when detected interrupt
volatile int initialize_song = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// microphone interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// routine in case of errors
void error_routine(void);

// receives the period of the node to set
static void set_Note(int);

// receives the period of the node to play and the delay of play
static void play_Note(int, int);

// create the linked list of notes (that composes a song)
static struct note* create_Song(int[], int[], int);

// the parameter is the TEMPO multiplier
static struct note* play_London_Bridge(int);
static struct note* play_Pokemon_Emerald(int);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(initialize_song) {
		  // remove possibilities of multiple interrupts too close to each other
		  HAL_Delay(50);

		  initialize_song = 0;

		  switch(SONG_SELECTOR) {
			  case 0:
				  head = play_London_Bridge(1);
				  break;
			  case 1:
				  head = play_Pokemon_Emerald(1);
				  break;
			  default:
				  head = NULL;
				  error_routine();
				  break;
		  }
	  }

	  if(head != NULL) {
		  play_Note(head->period, head->delay);
		  head = head->next;
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// interrupt of the microphone to START the song
	if(GPIO_Pin == GPIO_PIN_8)
		initialize_song = 1;
}

void error_routine() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

static void set_Note(int period) {

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = period - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK){
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK){
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK){
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((int) period / 2) - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK){
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK){
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

static struct note* create_and_play(int notes[], int delays[], int size_notes, int size_delays) {
	// check dimension of arrays and create structure of song
	if(size_notes != size_delays) {
		error_routine();
	} else {
		return create_Song(notes, delays, size_notes);
	}
	return NULL;
}

static struct note* create_Song(int notes[], int delays[], int size) {
	struct note *head = NULL;
	struct note *tail = NULL;
	int i = 0;

	for(i = 0; i < size; i++) {
		if(i == 0){
			head = malloc(sizeof(note));
			head->period = notes[i];
			head->delay = delays[i];
			head->next = NULL;

			tail = head;
		} else {
			tail->next = malloc(sizeof(note));
			tail = tail->next;
			tail->period = notes[i];
			tail->delay = delays[i];
			tail->next = NULL;
		}
	}

	return head;
}

static void play_Note(int period, int delay) {
	if(period != PAUSE) {
		set_Note(period);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	}

	HAL_Delay(delay);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

static struct note* play_London_Bridge(int multi) {
	// write notes
	int notes[] = {SOL4, LA4, SOL4, FA4, MI4, FA4, SOL4, RE4, MI4, FA4, MI4, FA4, SOL4, SOL4, LA4, SOL4,
					FA4, MI4, FA4, SOL4, RE4, SOL4, MI4, DO4};

	// write tempos
	int delays[] = {(int) TEMPO * multi *  3/8,
					(int) TEMPO * multi * 1/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 3/8,
					(int) TEMPO * multi * 1/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 4/8,
					(int) TEMPO * multi * 2/8,
					(int) TEMPO * multi * 6/8
	};

	return create_and_play(notes, delays, (sizeof(notes)/sizeof(int)), (sizeof(delays)/sizeof(int)));
}

static struct note* play_Pokemon_Emerald(int multi) {
	// write notes
	int notes[] = {(int) PAUSE,
					(int)FA4*2,
					(int)DO4,
					(int)FA4,
					(int)DO4,

					(int)FA4*2,
					(int)DO4,
					(int)FA4,
					(int)DO4,

					(int)LA4*2,
					(int)MI4,
					(int)LA4,
					(int)MI4,

					(int)DO4_,
					(int)LA4,
					(int)DO4_/2,
					(int)LA4,

					(int)RE4,
					(int)LA4,
					(int)RE4/2,
					(int)LA4,

					(int)DO4_,
					(int)LA4,
					(int)DO4_/2,
					(int)LA4,

					(int)DO4,
					(int)LA4_,
					(int)DO4,
					(int)LA4,

					(int)SI4,

					(int)LA4_*2,
					(int)FA4,
					(int)LA4_,
					(int)FA4,

					(int)LA4_,
					(int)FA4,
					(int)LA4,
					(int)FA4,

					(int)SOL4*2,
					(int)RE4,
					(int)SOL4,
					(int)RE4,

					(int)SOL4*2,
					(int)RE4,
					(int)SOL4,
					(int)RE4,

					(int)DO4,
					(int)SOL4,
					(int)DO4/2,
					(int)SOL4,

					(int)MI4/2,
					(int)SOL4,
					(int)DO4/2,
					(int)SOL4,

					(int)DO4,
					(int)SOL4,
					(int)LA4_*2,
					(int)SOL4,

					(int)DO4,
					(int)DO4*2
	};

	// write tempos
	int delays[] = {(int) TEMPO * multi * 6/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 8/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 2/8,

			(int) TEMPO * multi * 2/8,
			(int) TEMPO * multi * 6/8
	};

	return create_and_play(notes, delays, (sizeof(notes)/sizeof(int)), (sizeof(delays)/sizeof(int)));
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

#ifdef  USE_FULL_ASSERT
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
