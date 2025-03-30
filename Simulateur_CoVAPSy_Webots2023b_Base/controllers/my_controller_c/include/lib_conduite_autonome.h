//
// Created by David PROSPÉRIN on 29/03/2025.
//

#ifndef LIB_CONDUITE_AUTONOME_H
#define LIB_CONDUITE_AUTONOME_H

#define TAILLE_TABLEAU_DISCONTINUITEES 181

void test(float angle_degre);

int mod_python(int a, int b);
int rotation_angle_90_antihoraire(int angle_degre);
int rotation_angle_90_horaire(int angle_degre);
void cherche_discontinuitee(unsigned int seuil);

extern unsigned int nombre_discontinuitees;
extern unsigned int tableau_discontinuitees[TAILLE_TABLEAU_DISCONTINUITEES];
#endif //LIB_CONDUITE_AUTONOME_H
