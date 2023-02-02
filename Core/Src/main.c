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
#include "math.h"
#include "string.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

struct GPIOState {
	GPIO_PinState Current;
	GPIO_PinState Previous;
};

struct GPIO_PortPin {
	GPIO_TypeDef *Port;
	uint16_t Pin;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int8_t HAL_GPIO_ReadDigit();
GPIO_PinState HAL_GPIO_CheckID(uint64_t CheckID);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct GPIO_PortPin L[4] = {
		{L1_GPIO_Port, L1_Pin},
		{L2_GPIO_Port, L2_Pin},
		{L3_GPIO_Port, L3_Pin},
		{L4_GPIO_Port, L4_Pin}
};

struct GPIO_PortPin R[4] = {
		{R1_GPIO_Port, R1_Pin},
		{R2_GPIO_Port, R2_Pin},
		{R3_GPIO_Port, R3_Pin},
		{R4_GPIO_Port, R4_Pin}
};

struct GPIOState S[16] = {
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET}
};

char TxBuffer[20];
uint64_t ID = 0;

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

  uint32_t TimeStamp = HAL_GetTick() + 20;

  while (1)
  {

	  if (HAL_GetTick() >= TimeStamp) {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, HAL_GPIO_CheckID(64340500062));
		  TimeStamp += 20;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|R2_Pin|R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R4_Pin */
  GPIO_InitStruct.Pin = R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R3_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L1_Pin */
  GPIO_InitStruct.Pin = L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(L1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L2_Pin L4_Pin L3_Pin */
  GPIO_InitStruct.Pin = L2_Pin|L4_Pin|L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int8_t HAL_GPIO_ReadDigit() {
	uint16_t Digit = 0;
	static int x = 0;

	register int y = 0;
	for (y = 0; y < 4; y++) {
		S[(4 * y) + x].Current = HAL_GPIO_ReadPin(L[y].Port, L[y].Pin);
		if (!S[(4 * y) + x].Current && S[(4 * y) + x].Previous) {
			Digit |= 1 << ((4 * y) + x);
			S[(4 * y) + x].Previous = S[(4 * y) + x].Current;
			break;
		}
		S[(4 * y) + x].Previous = S[(4 * y) + x].Current;
	}
	HAL_GPIO_WritePin(R[x].Port, R[x].Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R[(x + 1) % 4].Port, R[(x + 1) % 4].Pin, GPIO_PIN_RESET);
	x = (x + 1) % 4;

	switch (Digit) {
	case 0x0001: return 7;
	case 0x0002: return 8;
	case 0x0004: return 9;
	case 0x0008: return -1; //CLEAR
	case 0x0010: return 4;
	case 0x0020: return 5;
	case 0x0040: return 6;
	case 0x0080: return -2; //BACKSPACE
	case 0x0100: return 1;
	case 0x0200: return 2;
	case 0x0400: return 3;
	case 0x1000: return 0;
	case 0x8000: return -3; //OK
	default: 	 return -4; //NONE
	}

}

GPIO_PinState HAL_GPIO_CheckID(uint64_t CheckID) {
	static int8_t ID_digit = -4;
	static uint8_t count = 0;

	if (ID_digit == -3) {
		ID_digit = HAL_GPIO_ReadDigit();
		if (ID_digit >= 0 || ID_digit == -4) ID_digit = -3;
	} else {
		ID_digit = HAL_GPIO_ReadDigit();
		if (count > 11 && ID_digit >= 0) ID_digit = -4;
	}

	switch (ID_digit) {
	case -1:
		ID = 0;
		count = 0;
		break;
	case -2:
		ID /= 10;
		count--;
		break;
	case -3:
		if (ID == CheckID) return GPIO_PIN_SET;
		break;
	case -4:
		break;
	default :
		ID = ID * 10 + ID_digit;
		count++;
		break;
	}

	return GPIO_PIN_RESET;
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
