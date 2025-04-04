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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "herculex.h"
#include "pwm_api.h"
#define TIM_CLOCK_FOURCHE_OPTIQUE 170000000
#define PRESCALAR_FOURCHE_OPTIQUE 1
#define ARR_FOURCHE_OPTIQUE 0xffffffff
#define DISTANCE_ANGULAIRE_EN_METRE 0.00493
#define MAX_SPEED 4.07
#define FACTEUR_CONVERTION_VITESSE_LINEAIRE_EN_RAPPORT_CYCLIQUE (1.0 / 3.90)
#define KP 1.5
#define CAN_ID_FOURCHE_OPTIQUE 27

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_FOURCHE_OPTIQUE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

//Interruption pour mesurer la vitesse du moteur
float CK_CNT = TIM_CLOCK_FOURCHE_OPTIQUE/(PRESCALAR_FOURCHE_OPTIQUE);
uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint32_t Difference = 0;
char First_rising;
float Frecuency = 0;
float measured_speed_m_par_sec = 0;
float consigne = 0.0;
bool sens_rotation;

//au moment de changement du projet le tim interrupt se désactive
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if(First_rising == 1)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			First_rising = 0;
		}
		else
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			if(IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}
			else if (IC_Val1 > IC_Val2)
			{
				Difference = ( ARR_FOURCHE_OPTIQUE - IC_Val1) + IC_Val2;
			}

			Frecuency = CK_CNT/Difference;
			measured_speed_m_par_sec = DISTANCE_ANGULAIRE_EN_METRE*Frecuency;

			__HAL_TIM_SET_COUNTER(htim, 0);
			First_rising = 1;
		}
	}
}
bool dix_millisecondes_passes = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  	if (htim==&htim3)
  	{
  		dix_millisecondes_passes = 1;
  	}

  }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
T_FDCAN_trame_rx trame_rx;
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
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
	/**
	 * @note Mettre la vitesse à 0 au démarrage du moteur CC
	 */
	PWM_dir_and_cycle(0, &htim1, TIM_CHANNEL_1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float error = 0;
	float pwm = 0;
	float compteur_de_10_mms;
	float compteur_de_vitesse_nulle;
	float measured_speed_m_par_sec_ancien = 0;
	int16_t measured_speed_mm_par_sec = 0;
	int8_t txData[2];
	FDCAN_TxHeaderTypeDef header;
	while (1)
	{
#ifdef DEBUG_FOURCHE_OPTIQUE
		printf(">Vitesse: %f\n", measured_speed_m_par_sec);
		//printf(">Frecuency: %f\n", Frecuency);
#endif

		/**
		 * @note Vitesse asservie de 6.8V à 8.5V pour une consigne de 0.8 m/s
		 */
		error = consigne - measured_speed_m_par_sec;
		printf(">error_vitesse_lineaire: %2.5f m/s\n", error);
		measured_speed_m_par_sec_ancien = measured_speed_m_par_sec;
		error = error * FACTEUR_CONVERTION_VITESSE_LINEAIRE_EN_RAPPORT_CYCLIQUE * KP;
		printf(">error_rapport_cyclique: %2.5f m/s\n", error);
		pwm = consigne * FACTEUR_CONVERTION_VITESSE_LINEAIRE_EN_RAPPORT_CYCLIQUE + error;
		printf(">pwm: %f\n", pwm);
		PWM_dir_and_cycle(sens_rotation, &htim1, TIM_CHANNEL_1, pwm);
		if(dix_millisecondes_passes)
				{
					compteur_de_10_mms++;
					if(measured_speed_m_par_sec == measured_speed_m_par_sec_ancien)
					{
						compteur_de_vitesse_nulle++;
						printf(">compteur_de_vitesse_nulle: %2.5f m/s\n", compteur_de_vitesse_nulle);
					}
					else compteur_de_vitesse_nulle = 0;
					if(compteur_de_vitesse_nulle == 40)
					{
						measured_speed_m_par_sec = 0;
						compteur_de_vitesse_nulle = 0;
					}
					if(compteur_de_10_mms == 50)
					{
						measured_speed_mm_par_sec = measured_speed_m_par_sec*1000;
						txData[0] = measured_speed_mm_par_sec >> 8;
						txData[1] = measured_speed_mm_par_sec & 0x00FF;
						/******************* NE PAS TOUCHER **************************************/
						header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
						header.BitRateSwitch = FDCAN_BRS_OFF;
						header.FDFormat = FDCAN_CLASSIC_CAN;
						header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
						header.MessageMarker = 0;
						/*************************************************************************/

						header.Identifier = CAN_ID_FOURCHE_OPTIQUE; // Set your CAN identifier
						header.IdType = FDCAN_STANDARD_ID; // Standard ID
						header.TxFrameType = FDCAN_DATA_FRAME; // Data frame
						header.DataLength = 2; // Data length

						/*************************************************************************/
						HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, txData);
						compteur_de_10_mms = 0;

					}
					dix_millisecondes_passes = 0;

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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &trame_rx.header, trame_rx.data);

		switch (trame_rx.header.Identifier) {
			case CAN_ID_HERKULEX:

        /**
         * @attention
         * Pas de printf sur l'USART2 car ça fait planter le herkulex
         */
				uint16_t pos_herculex = ((uint16_t) trame_rx.data[0] << 8) | trame_rx.data[1];
        /**
         * @Attention Si le Herkulex n'est pas alimenté en 7.2V alors il clignote en rouge
         */
				send_pos(ID_HERKULEX, pos_herculex);
				break;
			case CAN_ID_MOTEUR://vitesse consigne réçu en mm/s
					consigne = trame_rx.data[0] / 1000.0; //vitesse transformée en m/s
					sens_rotation =  trame_rx.data[1];
				break;
			default:
				break;
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
