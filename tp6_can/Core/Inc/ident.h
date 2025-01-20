#ifndef _IDENT_H
#define _IDENT_H

/* FILE: ident.h */
/****************************************************************************************/
/* MODULE NAME: MC9S12XS128 - MSCAN => Motorola Scalable Controller Area Network        */
/*                                     (BUS CAN)                                        */
/*                                                                                      */
/* AUTHOR     : BRUNO LARNAUDIE                                                         */
/*                                                                                      */
/* EMAIL      : bruno.larnaudie@u-psud.fr                                               */
/*                                                                                      */
/* INSTITUTION: IUT de CACHAN - 9 av. de la div. Leclerc - 94230 CACHAN                 */
/*                                                                                      */
/* DATE       : 03/07/2012                                                              */
/* CPU        : FREESCALE MC9S12XS128MAA                                                */
/* BOARD      : CACHAN Xboard+MicroB12                                                  */
/* OS         : None                                                                    */
/*                                                                                      */
/* DESCRIPTION: Identifiants utilises sur le bus CAN                                    */
/****************************************************************************************/

//         NOM            ID     |Flag |Sens |Data |     Fonction       | Data                                            |
//                               |Mess |(I/O)|Req  |                    | nb   | Type | contenu (dans l'ordre)            |
//------------------------------------------------------------------------------------------------------------------------|
// Carte IHM 0x700-0x7FF         |     |     |     |                    |      |      |                                   |
//------------------------------------------------------------------------------------------------------------------------|
//                             //|     |     |     |                    |      |      |                                   |
//  Module LCD 16x2            //|     |     |     |                    |      |      |                                   |
#define   LCD_CHAR0      0x700 //|  M  |  O  |  D  |char 0 a 7 sur LCD  |  8   |  U8  | C00|C01|C02|C03|C04|C05|C06|C07   |
#define   LCD_CHAR1      0x701 //|  M  |  O  |  D  |char 8 a 15 sur LCD |  8   |  U8  | C08|C09|C10|C11|C12|C13|C14|C15   |
#define   LCD_CHAR2      0x702 //|  M  |  O  |  D  |char 16 a 23 sur LCD|  8   |  U8  | C16|C17|C18|C19|C20|C21|C22|C23   |
#define   LCD_CHAR3      0x703 //|  M  |  O  |  D  |char 24 a 31 sur LCD|  8   |  U8  | C24|C25|C26|C27|C28|C29|C30|C31   |
//                             //|     |     |     |                    |      |      |                                   |
//                       0x704 //|     |     |     |                    |      |      | Reserve pour un éventuel LCD      |
// Réservé               ...   //|     |     |     |                    |      |      | plus grand ou tactile             |
//                       0x77D //|     |     |     |                    |      |      |                                   |
//                             //|     |     |     |                    |      |      |                                   |
#define   LCD_OVERFLOW   0x77E //|  M  |  O  |  R  |Flag d'Overflow LCD |  0   |      |                                   |
#define   LCD_CLEAR      0x77F //|  M  |  O  |  D  |                    |  0   |      | Efface l'ecran LCD                |
                               //|     |     |     |                    |      |      |                                   |
// JOG                         //|     |     |     |                    |      |      |                                   |
#define   JOG_REQ        0x790 //|  M  |  I  |  R  | Demande valeur JOG |  0   |      |                                   |
#define   JOG_DATA       0x791 //|  M  |  O  |  D  |     Valeur JOG     |  1   |  U8  | STATUS_JOG=PTP                    |
// CODEUR                        |     |     |     |                    |      |      |                                   |
#define   COD_REQ        0x7A0 //|  M  |  I  |  R  | Demande valeur COD |  0   |      |                                   |
#define   COD_DATA       0x7A1 //|  M  |  O  |  D  |     Valeur COD     |  1   |  S8  | OFFSET du CODEUR entre -128 et 127|
// BARGRAPH                      |     |     |     |                    |      |      |                                   |
#define   BAR_SET        0x7B0 //|  M  |  I  |  D  |   valeur bargraph  |  2   |  U16 | Les bits à allumer                |
//------------------------------------------------------------------------------------------------------------------------|
/* Légende : I = Consigne pour la carte
             O = Etat donné par la carte
             
             D = Trame de données
             R = Trame de requete */
             
#endif

