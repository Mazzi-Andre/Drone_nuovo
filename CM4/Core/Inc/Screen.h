/*
 * Screen.h
 *
 *  Created on: Feb 16, 2022
 *      Author: meefa
 */

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_
#include <fonts.h>
#include <ssd1306.h>

char buffer1[17], buffer2[17], buffer3[17];

/***************************************************************************************
 * Function name: screen_setup
 * Description  :Funzione che viene utilizzata per inizializzare il display e configurarlo per la
 * 				 scrittura.
 * Arguments    : None.
 * Return value : None.
 *
 ***************************************************************************************/
void screen_setup();

/***************************************************************************************
 * Function name: screen_setup
 * Description  :Funzione che riceve come argomento 3 float e li stampa a schermo.
 * Arguments    : None.
 * Return value : None.
 *
 ***************************************************************************************/
void print_acc(float ax,float ay,float az);

#endif /* INC_SCREEN_H_ */
