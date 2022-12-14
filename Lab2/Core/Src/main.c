/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct pinconfig_t {
	GPIO_TypeDef *GPIO_port;
	typeof(GPIO_PIN_All) GPIO_pin;
} pinconfig_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DIG4_PORT GPIOB
#define DIG4_PIN GPIO_PIN_1

#define DIG3_PORT GPIOB
#define DIG3_PIN GPIO_PIN_3

#define DIG2_PORT GPIOA
#define DIG2_PIN GPIO_PIN_10

#define DIG1_PORT GPIOC
#define DIG1_PIN GPIO_PIN_4

#define A_PORT GPIOA
#define A_PIN GPIO_PIN_6

#define B_PORT GPIOA
#define B_PIN GPIO_PIN_12

#define C_PORT GPIOA
#define C_PIN GPIO_PIN_11

#define D_PORT GPIOC
#define D_PIN GPIO_PIN_10

#define E_PORT GPIOC
#define E_PIN GPIO_PIN_11

#define F_PORT GPIOD
#define F_PIN GPIO_PIN_2

#define G_PORT GPIOC
#define G_PIN GPIO_PIN_12

#define DP_PORT GPIOA
#define DP_PIN GPIO_PIN_15

#define CODE_CLEAR 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const pinconfig_t segment_pins[] = { { A_PORT, A_PIN }, { B_PORT, B_PIN }, {
		C_PORT, C_PIN }, { D_PORT, D_PIN }, { E_PORT, E_PIN },
		{ F_PORT, F_PIN }, { G_PORT, G_PIN }, { DP_PORT, DP_PIN }, };

const pinconfig_t digit_pins[] = { { DIG1_PORT, DIG1_PIN }, { DIG2_PORT,
		DIG2_PIN }, { DIG3_PORT, DIG3_PIN }, { DIG4_PORT, DIG4_PIN }, };

const uint8_t segment_codes[] = {
//abcdefgp
		0b11111100,// 0
		0b01100000, // 1
		0b11011010, // 2
		0b11110010, // 3
		0b01100110, // 4
		0b10110110, // 5
		0b00111110, // 6
		0b11100000, // 7
		0b11111110, // 8
		0b11100110, // 9
		0b00000000, // Clear (10)
		0b00000010, // Dash (11)
		};

const GPIO_TypeDef *GPIO_ports[] = {
GPIOA,
GPIOB,
GPIOC,
GPIOD, };

const uint8_t n_ports = sizeof(GPIO_ports) / sizeof(GPIO_ports[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void light_digit2(int digit, int number) {
	uint16_t GPIO_set_masks[n_ports];
	uint16_t GPIO_reset_masks[n_ports];
	memset(GPIO_set_masks, 0, sizeof(GPIO_set_masks));
	memset(GPIO_reset_masks, 0, sizeof(GPIO_reset_masks));
	uint8_t GPIO_port_index;
	for (uint8_t n_segment = 0; n_segment < 7; n_segment++) {
		for (GPIO_port_index = 0;; GPIO_port_index++)
			if (segment_pins[n_segment].GPIO_port == GPIO_ports[GPIO_port_index])
				break;
		(bitRead(segment_codes[number],7-n_segment)?GPIO_set_masks:GPIO_reset_masks)
				[GPIO_port_index] |= segment_pins[n_segment].GPIO_pin;
	}
	for (uint8_t n_digit = 0; n_digit < 3; n_digit++) {
			for (GPIO_port_index = 0;; GPIO_port_index++)
				if (digit_pins[n_digit].GPIO_port == GPIO_ports[GPIO_port_index])
					break;
			(number-1 == n_digit?GPIO_set_masks:GPIO_reset_masks)
					[GPIO_port_index] |= digit_pins[n_digit].GPIO_pin;
	}

	for(uint8_t port = 0; port < n_ports; port++) {
		HAL_GPIO_WritePin(GPIO_ports[port], GPIO_set_masks[port], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_ports[port], GPIO_reset_masks[port], GPIO_PIN_SET);
	}

}

void light_sticks(int a, int b, int c, int d, int e, int f, int g) {
	HAL_GPIO_WritePin(A_PORT, A_PIN, a ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_PORT, B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_PORT, C_PIN, c ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_PORT, D_PIN, d ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_PORT, E_PIN, e ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_PORT, F_PIN, f ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(G_PORT, G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(DP_PORT, DP_PIN, GPIO_PIN_SET);
}

void light_position(int position) {
	HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN,
			(position == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN,
			(position == 2) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN,
			(position == 3) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN,
			(position == 4) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void light_digit(int position, int digit) {

	light_position(position);
	switch (digit) {
	case 0:
		light_sticks(1, 1, 1, 1, 1, 1, 0);
		break;
	case 1:
		light_sticks(0, 1, 1, 0, 0, 0, 0);
		break;
	case 2:
		light_sticks(1, 1, 0, 1, 1, 0, 1);
		break;
	case 3:
		light_sticks(1, 1, 1, 1, 0, 0, 1);
		break;
	case 4:
		light_sticks(0, 1, 1, 0, 0, 1, 1);
		break;
	case 5:
		light_sticks(1, 0, 1, 1, 0, 1, 1);
		break;
	case 6:
		light_sticks(1, 0, 1, 1, 1, 1, 1);
		break;
	case 7:
		light_sticks(1, 1, 1, 0, 0, 0, 0);
		break;
	case 8:
		light_sticks(1, 1, 1, 1, 1, 1, 1);
		break;
	case 9:
		light_sticks(1, 1, 1, 1, 0, 1, 1);
		break;
	default:
		light_sticks(0, 0, 0, 0, 0, 0, 0);
		break;
	}

}

void light_number(int number) {
	if(number < 0) {
		number = -number;
		light_digit(4, 12);
	} else
		light_digit(4, (number / 1000) % 10);
	HAL_Delay(1);
	light_digit(1, number % 10);
	HAL_Delay(1);
	light_digit(2, (number / 10) % 10);
	HAL_Delay(1);
	light_digit(3, (number / 100) % 10);
	HAL_Delay(1);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	int cnt = 0;
  	int red_port_prev = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
  	int blue_port_prev = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
  	int red_port_cur;
  	int blue_port_cur;
	while (1) {
		red_port_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		blue_port_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
		if (red_port_cur != red_port_prev && red_port_cur) {
			cnt++;
		}
		if (blue_port_cur != blue_port_prev && blue_port_cur) {
			cnt--;
		}
		red_port_prev = red_port_cur;
		blue_port_prev = blue_port_cur;

		if (cnt > 9999) cnt = 0;
		if (cnt < 0) cnt = 9999;

		light_number(cnt);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
