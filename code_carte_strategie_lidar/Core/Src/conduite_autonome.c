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

typedef enum
{
	DEBUT ,
	AVANCER,
	BLOQUER,
	RECULER,
} etat_deplacement_e;
static etat_deplacement_e etat_deplacement  = AVANCER ;


#define TAILLE_TAB_DISCONTINUITEES 180
#define TAILLE_MIN_LOCAUX 180
#define TAILLE_TABLEAU_LIDAR 180
#define SEUIL_RECULER_CAPTEUR_OBSTACLES 200


int tab_discontinuitees[TAILLE_TAB_DISCONTINUITEES][2] = {0};
int min_locaux[TAILLE_MIN_LOCAUX][2] = {0};

int cpt_discontinuitees = 0;

float angle_roue = 90.0;
float vitesse_lineaire_ancienne = 0.0;

#undef DEBUG_VERBOSE

void conduite_autonome(void)
{

	switch (etat_deplacement)
	{

	case DEBUT :
		etat_deplacement = AVANCER ;
		break ;

	case AVANCER :
		discontinuite() ;
		recherches_locaux();
		autonomous () ;
		set_consigne_vitesse(0.9, 0);
		set_angle_roue(angle_roue);


		if (vitesse_lineaire == 0 && vitesse_lineaire_ancienne > 0)
		{
			etat_deplacement = BLOQUER ;
		}
		vitesse_lineaire_ancienne = vitesse_lineaire;
		break ;
	case BLOQUER :
		if(capteur_obstacles_droit >= SEUIL_RECULER_CAPTEUR_OBSTACLES && capteur_obstacles_gauche >= SEUIL_RECULER_CAPTEUR_OBSTACLES)
		{
			etat_deplacement = RECULER ;
		}

		break ;
	case RECULER :

		set_consigne_vitesse(0.7, 1);
		set_angle_roue(180.0-angle_roue);
		HAL_Delay(1000) ;
        set_angle(90);
        HAL_Delay(500);
		etat_deplacement = AVANCER ;
		break ;



	default:
		break;
	}






#ifdef DEBUG_VERBOSE
	for (unsigned int i = 0; i < TAILLE_TAB_DISCONTINUITEES; i++)
	{
		printf("angle : %d distance : %d \n", i, tab_discontinuitees[i][0]);
	}
	printf("-------------------------------------------\n");
#endif

	LCD_gotoxy(0,0);
	LCD_printf("Nb disc %d,%d", cpt_discontinuitees,etat_deplacement) ;
	LCD_gotoxy(0,1);
	//LCD_printf("angle %3.2f deg", angle_roue);
	LCD_printf("%d  %d", capteur_obstacles_gauche, capteur_obstacles_droit);

	uint16_t motif_bar = 0b1000000000;
	BAR_set(motif_bar >>= ((uint16_t)angle_roue / 18));



#ifdef DEBUG_VERBOSE
	printf(">cpt_discontinuitees:%d|xy\n", cpt_discontinuitees);
	printf(">angle:%4.3f|xy\n", angle_roue);
#endif

	clear () ;
}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 180°.
 * @todo Tester et valider la fonction void lidar_complete_scan_callback()
 */
void lidar_complete_scan_callback()
{
	HAL_GPIO_TogglePin(led_temoin_GPIO_Port, led_temoin_Pin);
	/**
	 * @warning Pas de requête CAN dans cette fonction de callback
	 */
}

void discontinuite()
{
	int distance_courante = 0  ;
	int distance_suivante = 0 ;
	int diff = 0 ;
	int seuil_discontinuite = 250;
	cpt_discontinuitees = 0;


	for (int i = 0; i < TAILLE_TAB_DISCONTINUITEES - 1; i++)
	{
		distance_courante = data_lidar_mm_main[i];
		distance_suivante = data_lidar_mm_main[i + 1];
		diff = (int)fabs(distance_suivante - distance_courante);

		if (diff >= seuil_discontinuite)
		{
			tab_discontinuitees[i][0] = distance_courante;
			tab_discontinuitees[i][1] = i;
			cpt_discontinuitees++;
		}
	}
}


void recherches_locaux()
{
	static etat_discontuinuite etat = INIT;
	int distance_actuelle = 0;
	int distance_apres = 0;

	for (int i = 0; i < TAILLE_MIN_LOCAUX - 1; i++)
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
	int max_distance_1_discontinuite = 0;
	int angle_1_discontinuite = 0;

	//Variables pour 0 discontinuiter
	int max_distance_0_discontinuite = 0;
	int angle_0_discontinuite = 0;


	if (cpt_discontinuitees > 1)
	{
		for (int i = 0; i < TAILLE_TAB_DISCONTINUITEES; i++)
		{
			if (max_discontinuite_2_distance_1 < tab_discontinuitees[i][0])
			{
				max_discontinuite_2_distance_2 = max_discontinuite_2_distance_1;
				angle_discontinuite_2_2 = angle_discontinuite_2_1;
				max_discontinuite_2_distance_1 = tab_discontinuitees[i][0];
				angle_discontinuite_2_1 = i ;
			}
			else if (max_discontinuite_2_distance_2 < tab_discontinuitees[i][0] && max_discontinuite_2_distance_1 > tab_discontinuitees[i][0])
			{
				max_discontinuite_2_distance_2 = tab_discontinuitees[i][0];
				angle_discontinuite_2_2 = i;
			}
			else if (max_discontinuite_2_distance_1 == tab_discontinuitees[i][0] && max_discontinuite_2_distance_2 < tab_discontinuitees[i][0])
			{
				max_discontinuite_2_distance_2 = tab_discontinuitees[i][0];
				angle_discontinuite_2_2 = i;
			}
		}
		angle_roue = (angle_discontinuite_2_1 + angle_discontinuite_2_2) / 2;
	}

	else if (cpt_discontinuitees == 1)
	{
		for (int i = 0; i < TAILLE_TAB_DISCONTINUITEES; i++)
		{

			if (tab_discontinuitees[i][0] > max_distance_1_discontinuite)
			{
				max_distance_1_discontinuite = tab_discontinuitees[i][0];
				angle_1_discontinuite = i ;
			}

			if (min_locaux[i][0] > max_distance_min_locaux_1)
			{
				max_distance_min_locaux_1 = min_locaux[i][0];
				angle_min_locaux_1 = i;
			}
		}

#ifdef DEBUG_DISC_1
		printf(">max_distance_1_discontinuite:%d|xy\n", max_distance_1_discontinuite);
		printf(">angle_1_discontinuite:%d|xy\n", angle_1_discontinuite);

		printf(">max_distance_min_locaux_1:%d|xy\n", max_distance_min_locaux_1);
		printf(">angle_min_locaux_1:%d|xy\n", angle_min_locaux_1);
#endif

		angle_roue  = (angle_min_locaux_1 + angle_1_discontinuite)/2;
	}

	else if (cpt_discontinuitees == 0)
	{
		for (int i = 0; i < TAILLE_TABLEAU_LIDAR; i++)
		{
			if (data_lidar_mm_main[i] > max_distance_0_discontinuite)
			{
				max_distance_0_discontinuite = data_lidar_mm_main[i];
				angle_0_discontinuite = i;
			}
		}
#ifdef DEBUG_DISC_0
		printf(">max_distance_0_discontinuite:%d|xy\n", max_distance_0_discontinuite);
		printf(">angle_0_discontinuite:%d|xy\n", angle_0_discontinuite);
#endif
		angle_roue  = angle_0_discontinuite ;
	}
}

void clear ()
{
	for (int i = 0; i < TAILLE_TAB_DISCONTINUITEES; i++)
	{
		tab_discontinuitees[i][0] = 0;
		tab_discontinuitees[i][1] = 0;
	}

	for (int i = 0; i < TAILLE_MIN_LOCAUX; i++)
	{
		min_locaux[i][0] = 0;
		min_locaux[i][1] = 0;
	}
}













