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
int16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];

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

/**
 * @brief Initialise le tableau data_lidar_mm_main
 *
 * @retval void
 */
void init_data_lidar_mm_main()
{
	for (size_t i = 0; i < DATA_LIDAR_MM_MAIN_SIZE; i++)
	{
		data_lidar_mm_main[i] = -1;
	}
}

/**
 * @brief Affiche le contenu du tableau data_lidar_mm_main
 *
 * @retval void
 */
void print_init_data_lidar_mm_main()
{
	for (size_t i = 0; i < DATA_LIDAR_MM_MAIN_SIZE; i++)
	{
		printf("(%d, %d)\n", i, data_lidar_mm_main[i]);
	}
}

/**
 * @brief  Ajout une mesure au tableau data_lidar_mm_main
 *
 * @retval Revoie true si élément à bien été ajouté au tableau, false sinon
 */
bool ajout_mesure(float angle, float distance)
{
	if (!(angle >= 0 && angle <= DATA_LIDAR_MM_MAIN_SIZE))
	{
		return false;
	}

	if (data_lidar_mm_main[(size_t) angle] == -1)
	{
		data_lidar_mm_main[(size_t) angle] = (int16_t) distance;
		return true;
	} else {

		float somme = data_lidar_mm_main[(size_t) angle] + distance;
		// Moyenne
		int16_t moyenne = somme / 2.0;

		data_lidar_mm_main[(size_t) angle] = moyenne;
		return true;
	}
}

