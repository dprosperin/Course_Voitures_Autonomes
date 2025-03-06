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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <lidar.h>
#include <ihm.h>
#include "test.h"
#include "deplacement.h"
#include "utils.h"
#include "conduite_autonome.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_START_AUTONOMOUS_DRIVING 0x500
#define CAN_STOP_AUTONOMOUS_DRIVING 0x501
#define CAN_ID_FOURCHE_OPTIQUE 27
#undef  AUTONOMOUS_DRIVING_STARTED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag_reception_uart2 = 0;
uint8_t caractere;
bool is_autonomous_driving_started = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("Programme interface LIDAR\n");
	printf("Compile le %s\n", __DATE__);

	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	LCD_clear();

	HAL_UART_Receive_IT(&PC_HUART, &caractere, 1); // A laisser proche de la boucle while(1)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/**
		 * @todo Les Hal_delays font échouer la reception des commandes par UART
		 */

#ifdef AUTONOMOUS_DRIVING_STARTED
		if (!is_autonomous_driving_started)
		{
			JOG_read();
			COD_read();
			automate_decode_IHM();
			test_composants_voiture();
			printf("COD Value : %d\n", cod_value);
			printf("JOG Value : %d\n", jog_value);
		}
#endif
		lidar_handle_receive_character();


		if (command_requested == LIDAR_SCAN_IN_PROGESS)
		{
			lidar_print_array_distance_teleplot_format(data_lidar_mm_main, 360);

			if (is_autonomous_driving_started)
			{
				conduite_autonome();
			}

			print_angle_herkulex_teleplot();
			print_vitesse_moteur_teleplot();
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

/* USER CODE BEGIN 4 */
/**
 * @attention La migration du projet vers une version plus récente de Cube IDE ne fait plus fonctionner la réception de trames CAN
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
			&buffer_trame_rx[marker1].header,
			&buffer_trame_rx[marker1].data[0]);

	switch (buffer_trame_rx[marker1].header.Identifier)
	{
	case CAN_START_AUTONOMOUS_DRIVING:
		printf("START AUTONOMOUS DRIVING\n");
		is_autonomous_driving_started = 1;
		break;

	case CAN_STOP_AUTONOMOUS_DRIVING:
		printf("STOP AUTONOMOUS DRIVING\n");
		is_autonomous_driving_started = 0;
		break;
	case CAN_ID_FOURCHE_OPTIQUE:
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		float m_per_sec = (float)buffer_trame_rx[marker1].data[0]/1000;
	}

	marker1++;

	if (marker1 == 32) {
		marker1 = 0;
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		if (command_requested == LIDAR_SCAN_IN_PROGESS)
		{
			float angle = 0;
			float distance = 0;
			bool is_first_scan_point = 0;

			lidar_decode_angle_and_distance(buffer_DMA_scan, &angle, &distance, &is_first_scan_point);

			if (distance > 0)
			{
				if (angle >= 0 && angle <= 180)
				{
					data_lidar_mm_main[(uint16_t) angle] = distance;
				}

				if ((uint16_t) angle == 90)
				{
					lidar_half_complete_scan_callback();
				} else if ((uint16_t) angle == 180)
				{
					lidar_complete_scan_callback();
				}
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		flag_reception_uart2 = 1;
		/**
		 * @note Relancer la réception dans l'interruption
		 */
		HAL_UART_Receive_IT(&PC_HUART, &caractere, 1);
	}

	if (huart->Instance == USART1) {
		if (command_requested == LIDAR_GET_INFO)
		{
			printf("Response descriptor GET_INFO\n");
			lidar_decode_get_info(buffer_UART);
		} else if (command_requested == LIDAR_GET_HEALTH)
		{
			printf("Response descriptor GET_HEALTH\n");
			lidar_decode_get_health(buffer_UART);
		} else if (command_requested == LIDAR_START_SCAN)
		{
			printf("Response descriptor SCAN\n");
			command_requested = LIDAR_SCAN_IN_PROGESS;
			HAL_UART_Receive_DMA(&LIDAR_HUART, buffer_DMA_scan, BUFFER_DMA_SIZE);
		} else if (command_requested == LIDAR_GET_SAMPLERATE)
		{
			printf("Response descriptor GET_SAMPLERATE\n");
			/**
			 * @todo Verifer le retour quand le LIDAR est en mode balayage
			 */
			lidar_decode_get_samplerate(buffer_UART);
		} else if (command_requested == LIDAR_SCAN_IN_PROGESS)
		{
			if (command_requested == LIDAR_SCAN_IN_PROGESS)
			{
				float angle = 0;
				float distance = 0;
				bool is_first_scan_point = 0;

				lidar_decode_angle_and_distance(buffer_DMA_scan + 5, &angle, &distance, &is_first_scan_point);

				if (distance > 0 && angle >= 0 && angle <= 180)
				{
					data_lidar_mm_main[(uint16_t) angle] = distance;

					if ((uint16_t) angle == 90)
					{
						lidar_half_complete_scan_callback();
					} else if ((uint16_t) angle == 180)
					{
						lidar_complete_scan_callback();
					}
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
