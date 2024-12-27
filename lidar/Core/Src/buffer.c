/**
  ******************************************************************************
  * @file buffer.c
  * @brief Fonctions utilitaire pour le buffer circulaire
  * @author David PROSPÉRIN
  ******************************************************************************
  */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"

size_t index_read = 0;
size_t buffer_size = 0;
uint8_t buffer[2048] = {0};

extern UART_HandleTypeDef huart1;

/**
 * @brief Vérifie si le buffer est plein
 *
 * @retval Retourne True si le buffer est plein False sinon
 */
bool bufferIsFull()
{
	return buffer_size == 2048;
}

/**
 * @brief Vérifie si le buffer est vide
 *
 * @retval Retourne True si le buffer est vide False si le buffer contient un ou plusieurs élément(s)
 */
bool bufferIsEmpty()
{
	return buffer_size == 0;
}

/**
 * @brief Retire un élément du buffer circulaire
 *
 * @param Variable
 * @retval Retourne True si élément est bien retiré du buffer False sinon
 */

bool dequeue(uint8_t *value) {
    uint32_t remainingByte = 2048 - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (remainingByte > index_read)
    {
    	*value = buffer[index_read];

    	index_read = (index_read + 1) % 2048;  // Boucle autour

    	return true;
    } else {
    	return false;
    }

}



