/*
 * mymain.c
 *
 *  Created on: Jan 6, 2025
 *      Author: david.prosperin
 */
#include "conduite_autonome.h"

typedef enum
{
	INIT,
	MIN_LOCAL,
	MAX_LOCAL,
	AUGMENTER,
	DIMINUER
} etat_discontuinuite;
static etat_discontuinuite etat = INIT;

int tab_discontuinuite[180][2] = {0};
int min_locaux[180][2] = {0};

int cpt_discontuinuiter = 0;
int angle_1_discontinuite = 0;

void conduite_autonome(void)
{

	discontinuite() ;
	recherches_locaux();
	autonomous () ;
	clear ();

	float angle_mapped = mapf(angle, 0, 180, ANGLE_HERKULEX_MIN, ANGLE_HERKULEX_MAX);

	set_angle(angle_mapped);
	LCD_gotoxy(0,0);
	LCD_printf("Nb disc %d", cpt_discontuinuiter) ;
	LCD_gotoxy(0,1);
    LCD_printf("angle %4.4f", angle_mapped) ;

    cpt_discontuinuiter = 0;
	angle_1_discontinuite = 0;
}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 180°.
 * @todo Tester et valider la fonction void lidar_complete_scan_callback()
 */
void lidar_complete_scan_callback()
{
	//printf(">demi_tour:%lu|xy\n", HAL_GetTick());
}

void discontinuite()
{
	int distance_courante = 0  ;
	int distance_suivante = 0 ;
	int diff = 0 ;
	int seuil_discontinuite = 50;


	for (int i = 0; i < 179; i++)
	{
		distance_courante = data_lidar_mm_main[i];
		distance_suivante = data_lidar_mm_main[i + 1];
		diff = (int)fabs(distance_suivante - distance_courante);

		if (diff >= seuil_discontinuite)
		{
			tab_discontuinuite[i][0] = distance_courante;
			tab_discontuinuite[i][1] = i;
			cpt_discontuinuiter++;
			angle_1_discontinuite = i ;
		}
	}
}


void recherches_locaux()
{
	int distance_actuelle = 0;
	int distance_apres = 0;

	for (int i = 0; i < 179; i++)
	{
		distance_actuelle = data_lidar_mm_main[i];
		distance_apres = data_lidar_mm_main[i + 1];
		switch (etat)
		{
		case INIT:
			if (distance_actuelle < distance_apres)
			{
				etat = AUGMENTER;
			}
			else if (distance_actuelle > distance_apres)
			{
				etat = DIMINUER;
			}
			break;

		case AUGMENTER:
			if (distance_actuelle < distance_apres)
			{
				etat = AUGMENTER;
			}
			else if (distance_actuelle > distance_apres)
			{
				etat = MAX_LOCAL;
			}
			break;

		case DIMINUER:
			if (distance_actuelle > distance_apres)
			{
				etat = DIMINUER;
			}
			else if (distance_actuelle < distance_apres)
			{
				etat = MIN_LOCAL;
			}
			break;

		case MIN_LOCAL:
			min_locaux[i][0] = distance_actuelle;
			min_locaux[i][1] = i;
			etat = INIT;
			break;

		case MAX_LOCAL:
			etat = INIT;
			break;

		default:
			break;
		}
	}

}

void autonomous()
{
	//Variables pour 2 ou plus discontinuiter
	int max_discontinuite_2_distance_1 = 0;
	int max_discontinuite_2_distance_2 = 0;
	int angle_discontinuite_2_1 = 0;
	int angle_discontinuite_2_2 = 0;
	//Variables pour 1 discontinuiter
	int max_distance_min_locaux_1 = 0;
	int angle_min_locaux_1 = 0;
	int max_distance_min_locaux_2 = 0;
	int angle_min_locaux_2 = 0;
	int max_distance_1_discontinuite = 0;
	int angle_1_discontinuiter  = 0 ;

	//Variables pour 0 discontinuiter
	int max_distance_0_discontinuite = 0;
	int angle_0_discontinuite = 0;


	if (cpt_discontuinuiter >= 2)
	{
		for (int i = 0; i < 180; i++)
		{
			if (max_discontinuite_2_distance_1 < tab_discontuinuite[i][0])
			{
				max_discontinuite_2_distance_2 = max_discontinuite_2_distance_1;
				angle_discontinuite_2_2 = angle_discontinuite_2_1;
				max_discontinuite_2_distance_1 = tab_discontuinuite[i][0];
			}
			else if (max_discontinuite_2_distance_2 < tab_discontuinuite[i][0] && max_discontinuite_2_distance_1 > tab_discontuinuite[i][0])
			{
				max_discontinuite_2_distance_2 = tab_discontuinuite[i][0];
				angle_discontinuite_2_2 = i;
			}
			else if (max_discontinuite_2_distance_1 == tab_discontuinuite[i][0] && max_discontinuite_2_distance_2 < tab_discontuinuite[i][0])
			{
				max_discontinuite_2_distance_2 = tab_discontuinuite[i][0];
				angle_discontinuite_2_2 = i;
			}
		}
		angle = (angle_discontinuite_2_1 + angle_discontinuite_2_2) / 2;
	}

	else if (cpt_discontuinuiter == 1)
	{
		for (int i = 0; i < 180; i++)
		{
			if (tab_discontuinuite[i][0] > max_distance_1_discontinuite)
			{
				max_distance_1_discontinuite = tab_discontuinuite[i][0];
			}
		}

		for (int i = angle_1_discontinuite; i > (angle_1_discontinuite - 30); i--)
		{
			if (min_locaux[i][0] > max_distance_min_locaux_1)
			{
				max_distance_min_locaux_1 = min_locaux[i][0];
				angle_min_locaux_1 = i;
			}
		}
		for (int i = angle_1_discontinuite; i < (angle_1_discontinuite + 30); i++)
		{
			if (min_locaux[i][0] > max_distance_min_locaux_2)
			{
				max_distance_min_locaux_2 = min_locaux[i][0];
				angle_min_locaux_2 = i;
			}
		}

		if (max_distance_min_locaux_1 > max_distance_min_locaux_2)
		{
			angle = (angle_min_locaux_1 + angle_1_discontinuite) / 2;
		}
		else if (max_distance_min_locaux_2 > max_distance_min_locaux_1)
		{
			angle = (angle_min_locaux_2 + angle_1_discontinuite) / 2;
		}
	}

	else if (cpt_discontuinuiter == 0)
	{
		for (int i = 0; i < 180; i++)
		{
			if (data_lidar_mm_main[i] > max_distance_0_discontinuite)
			{
				max_distance_0_discontinuite = data_lidar_mm_main[i];
				angle_0_discontinuite = i;
			}
		}
		if (0 <= angle_0_discontinuite && angle_0_discontinuite <= 89)
		{
			angle = 0;
		}
		else if (90 <= angle_0_discontinuite && angle_0_discontinuite <= 179)
		{
			angle = 180;
		}
	}
}

void clear ()
{
	for (int i = 0; i < 180; i++)
	{
		tab_discontuinuite[i][0] = 0;
		tab_discontuinuite[i][1] = 0;
	}

	for (int i = 0; i < 180; i++)
	{
		min_locaux[i][0] = 0;
		min_locaux[i][1] = 0;
	}
}













