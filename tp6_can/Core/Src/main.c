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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ident.h"
#include <stdio.h>
#include <stdarg.h>
#include <deplacement.h>
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t marker1 = 0;
T_FDCAN_trame_rx trame_rx;
T_FDCAN_trame_rx buffer_trame_rx[32];
uint8_t curseur = 0;
char tableau_ecran[65];
uint8_t jog_value = 0;
uint8_t cod_value = 0;
uint16_t valeur_afficher;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_FDCAN1_Init();
	/* USER CODE BEGIN 2 */

	printf("En vie\n");

	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	LCD_clear();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		JOG_read();
		COD_read();
		automate_decode_IHM();
		test_composants_voiture () ;


		//TODO : Bloque la réception des trames lidar

		valeur_afficher = valeur_cod & 0x00FF;

		BAR_set(valeur_cod);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
			&buffer_trame_rx[marker1].header,
			&buffer_trame_rx[marker1].data[0]);
	//printf("0x%X marker1 :%d \n", buffer_trame_rx[marker1].data[0], marker1);
	marker1++;

	if (marker1 == 32) {
		marker1 = 0;
	}
}

void automate_decode_IHM(void) {
	static uint16_t marker2 = 0;

	//printf("Id trame : 0x%X\n", buffer_trame_rx[marker2].header.Identifier);

	switch (buffer_trame_rx[marker2].header.Identifier) {
	case JOG_DATA:
		printf("JOG value : 0x%X\n", buffer_trame_rx[marker2].data[0]);
		jog_value = buffer_trame_rx[marker2].data[0];
		break;

	case COD_DATA:
		printf("COD value : %d\n", buffer_trame_rx[marker2].data[0]);
		cod_value = buffer_trame_rx[marker2].data[0];
		break;
	}

	marker2++;

	if (marker2 == 32) {
		marker2 = 0;
	}
}

/**
 * @brief Obtenir la valeur du JOG
 * @note Pas besoin d'attente bloaquante pour cette fonction avec IHM v2
 */
void JOG_read(void) {
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = JOG_REQ;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);

}

/**
 * @brief Obtenir la valeur du COD
 * @note Pas besoin d'attente bloaquante pour cette fonction avec IHM v2
 */
void COD_read(void) {
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = COD_REQ;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);

}

void BAR_set(uint16_t motif_BAR) {
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = BAR_SET;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = 2;

	uint8_t pTxData[2] = { (uint8_t) (motif_BAR >> 8), (uint8_t) (motif_BAR) };
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);

	HAL_Delay(1);
}

void LCD_gotoxy(uint8_t x, uint8_t y) {
	curseur = (y * 16 + x) % 32;
}

void LCD_clear(void) {
	uint8_t i;

	curseur = 0;

	for (i = 0; i < 32; i++)
		tableau_ecran[i] = 32;

	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = LCD_CLEAR;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
	HAL_Delay(1);
}

void LCD_printf(const char *format, ...) {
	uint8_t i, j;
	va_list arg;
	va_start(arg, format);

	curseur = curseur + vsprintf(tableau_ecran + curseur % 64, format, arg);

	if (curseur > 31) {
		for (i = 32; i < curseur; i++) {
			tableau_ecran[i % 32] = tableau_ecran[i];
		}
		curseur = curseur % 32;
	} else {
		for (i = 0; i < 32; i++) {
			if (tableau_ecran[i] == 0)
				tableau_ecran[i] = 20;
		}
	}
	va_end(arg);
	tableau_ecran[32] = '\0';

	for (j = 0; j < 4; j++) {
		FDCAN_TxHeaderTypeDef pTxHeader = { 0 };
		uint8_t pTxData[8];

		pTxHeader.Identifier = LCD_CHAR0 + j;
		pTxHeader.IdType = FDCAN_STANDARD_ID;
		pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
		pTxHeader.DataLength = 8;

		for (i = 0; i < 8; i++)
			pTxData[i] = tableau_ecran[i + j * 8];

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
		HAL_Delay(1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
