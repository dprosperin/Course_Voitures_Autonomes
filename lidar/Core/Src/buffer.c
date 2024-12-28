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
#include "buffer.h"

size_t index_read = 0;
size_t buffer_size = 0;
uint8_t buffer[BUFFER_SIZE] = {0};

extern UART_HandleTypeDef huart1;

/**
 * @brief Retire un élément du buffer circulaire
 *
 * @param Variable
 * @retval Retourne True si élément est bien retiré du buffer False sinon
 */

bool dequeue(uint8_t *value) {
    uint32_t remainingByte = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (remainingByte > index_read)
    {
    	*value = buffer[index_read];

    	index_read = (index_read + 1) % BUFFER_SIZE;  // Boucle autour

    	return true;
    } else {
    	return false;
    }

}
