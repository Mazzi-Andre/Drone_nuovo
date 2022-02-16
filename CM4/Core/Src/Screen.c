/*
 * Screen.c
 *
 *  Created on: Feb 16, 2022
 *      Author: meefa
 */
#include <fonts.h>
#include <ssd1306.h>
#include <Screen.h>
#include <stdio.h>


void screen_setup(){
	SSD1306_Init();
	SSD1306_Fill(1);
	SSD1306_UpdateScreen();
	HAL_Delay(200);
	SSD1306_Fill(0);
	SSD1306_UpdateScreen();
	HAL_Delay(200);
	SSD1306_Fill(1);
	SSD1306_UpdateScreen();
	HAL_Delay(200);
	SSD1306_Fill(0);
	SSD1306_UpdateScreen();
}
void print_acc(float ax,float ay,float az){
	sprintf(buffer1, "ax: %.3f", ax);
	sprintf(buffer2, "ay: %.3f", ay);
	sprintf(buffer3, "az: %.3f", az);


	SSD1306_Fill(0);
	SSD1306_UpdateScreen();
}
