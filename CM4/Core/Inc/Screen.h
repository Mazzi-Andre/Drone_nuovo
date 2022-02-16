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

char buffer1[10], buffer2[10], buffer3[10];

void screen_setup();
void print_acc(float ax,float ay,float az);
void print_gyr(float gx,float gy,float gz);
#endif /* INC_SCREEN_H_ */
