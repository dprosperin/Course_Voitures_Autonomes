/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <lidar.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIDAR_HUART huart1
#define PC_HUART huart2

#define BUFFER_SIZE 20
#define DATA_LIDAR_MM_MAIN_SIZE 360 + 1
//#define DO_TESTS 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t flag_reception_uart2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

uint8_t caractere;

command_lidar_t command_requested = LIDAR_UNKNOWN_COMMAND;

uint8_t buffer_scan[10] = {0};

uint8_t buffer[BUFFER_SIZE] = {0};
int16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Programme interface LIDAR\n");
  printf("Compile le %s\n", __DATE__);

    int i = 0;
    char message[40] = "";

    HAL_UART_Receive_IT(&PC_HUART, &caractere, 1); // A laisser proche de la boucle while(1)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (flag_reception_uart2 == 1) {
	  		  if (caractere == '\n') {
	  			  if (strstr(message, "START_SCAN") != NULL)
	  			  {
	  				command_requested = LIDAR_START_SCAN;
	  				printf("Demarrage du scan normal\n");
	  				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_START_SCAN, LIDAR_COMMAND_START_SCAN_LEN, HAL_MAX_DELAY);
	  				HAL_UART_Receive_IT(&LIDAR_HUART, buffer, 7);
	  			  } else if (strstr(message, "STOP") != NULL)
	  			  {
	  				command_requested = LIDAR_STOP;
	  				printf("Arret\n");
	  				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_STOP, LIDAR_COMMAND_STOP_LEN, HAL_MAX_DELAY);
	  			  } else if (strstr(message, "RESET") != NULL)
	  			  {
	  				command_requested = LIDAR_RESET;
	  				printf("Reset\n");
	  				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_RESET, LIDAR_COMMAND_RESET_LEN, HAL_MAX_DELAY);
	  			  } else if (strstr(message, "GET_INFO") != NULL)
	  			  {
	  				printf("RTFM ! <*_*>\n");
	  				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_INFO, LIDAR_COMMAND_GET_INFO_LEN, HAL_MAX_DELAY);
	  			  } else if (strstr(message, "GET_HEALTH") != NULL)
	  			  {
	  				command_requested = LIDAR_GET_HEALTH;
	  				printf("GET_HEALTH\n");
	  				HAL_UART_Receive_IT(&LIDAR_HUART, buffer, 10);
	  				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_HEALTH, LIDAR_COMMAND_GET_HEALTH_LEN, HAL_MAX_DELAY);
	  			  } else {
	  				  command_requested = LIDAR_UNKNOWN_COMMAND;
	  				  printf("Commande non reconnue : %s\n", message);
	  			  }

	  			  message[0] = '\0';
	  			  i = 0;
	  		  }

	  		  message[i++] = caractere;
	  		  flag_reception_uart2 = 0;

	  		  HAL_UART_Receive_IT(&PC_HUART, &caractere, 1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		if (command_requested == LIDAR_SCAN_IN_PROGESS)
		{
			float angle = 0;
			float distance = 0;

			lidar_decode_angle_and_distance(buffer_scan, &angle, &distance);

			if (distance > 0)
			{
				//printf("(%2.3f,%2.3f)\n", angle, distance);
				lidar_print_single_point_teleplot_format(angle, distance);
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		flag_reception_uart2 = 1;
		/*
		 * Relancer la réception dans l'interruption
	     */
		HAL_UART_Receive_IT(&PC_HUART, &caractere, 1);
	}

	if (huart->Instance == USART1) {
		if (command_requested == LIDAR_GET_HEALTH)
		{
			printf("Response descriptor GET_HEALTH\n");

			char status_msg[16] = "";

			uint8_t status = ((rplidar_device_health_data_response_t *)(buffer + 7))->status;

			uint8_t error_code_low_byte = ((rplidar_device_health_data_response_t *)(buffer + 7))->error_code_7_0;

			uint8_t error_code_high_byte = ((rplidar_device_health_data_response_t *)(buffer + 7))->error_code_15_8;

			uint16_t error_code = ((((uint16_t) error_code_high_byte << 8) & 0xFF00 ) | ((uint16_t) error_code_low_byte & 0x00FF));

			    	switch (status) {
						case 0:
							sprintf(status_msg, "Good");
							break;
						case 1:
							sprintf(status_msg, "Warning");
							break;
						case 2:
							sprintf(status_msg, "Error");
							break;
						default:
							sprintf(status_msg, "Unknown status");
							break;
					}
			  printf("=== Health Report ===\n"
			    	 "status : %s\n"
			    	 "error code : 0x%x\n"
			    	 "======================\n", status_msg, error_code);


		} else if (command_requested == LIDAR_START_SCAN)
		{
			printf("Response descriptor SCAN\n");
			command_requested = LIDAR_SCAN_IN_PROGESS;
			HAL_UART_Receive_DMA(&LIDAR_HUART, buffer_scan, 10);
		} else if (command_requested == LIDAR_SCAN_IN_PROGESS)
		{
			if (command_requested == LIDAR_SCAN_IN_PROGESS)
			{
				float angle = 0;
				float distance = 0;

				lidar_decode_angle_and_distance(buffer_scan + 5, &angle, &distance);

				if (distance > 0)
				{
					//printf("(%2.3f,%2.3f)\n", angle, distance);
					lidar_print_single_point_teleplot_format(angle, distance);
				}
			}
		}
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
