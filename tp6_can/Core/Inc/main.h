/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**
 * @brief	Définition de la structure FDCAN_trame_rx pour la réception de trames
 *			CAN
 */
typedef struct
{
	FDCAN_RxHeaderTypeDef header; /* Spécifie l'en-tête de la trame CAN reçue
	 (ID, IDE, RTR, DLC) */
	uint8_t data[8];  /* Spécifie les données de la trame CAN reçue */
} T_FDCAN_trame_rx;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void automate_decode_IHM(void);
void LCD_gotoxy (uint8_t x, uint8_t y);
void LCD_printf(const char* format, ...);
void LCD_clear(void);
void JOG_read(void);
void COD_read(void);
void BARGRAPHE_set(uint16_t motif_BAR);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t jog_value;
extern uint8_t cod_value;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
