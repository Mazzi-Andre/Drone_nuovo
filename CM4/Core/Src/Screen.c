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


	SSD1306_GotoXY(10, 10);
	SSD1306_Puts((char*)buffer1, &Font_7x10, 1);
	SSD1306_UpdateScreen();



	SSD1306_GotoXY(10, 20);
	SSD1306_Puts((char*)buffer2, &Font_7x10, 1);
	SSD1306_UpdateScreen();



	SSD1306_GotoXY(10, 30);
	SSD1306_Puts((char*)buffer3, &Font_7x10, 1);
	SSD1306_UpdateScreen();

}
