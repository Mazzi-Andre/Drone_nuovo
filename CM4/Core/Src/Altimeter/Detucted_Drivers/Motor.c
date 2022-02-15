/*
 * Motor.c
 *
 *  Created on: 26 nov 2021
 *      Author: Fabio Mecozzi
 */

// std-includes
#include "Motor.h"

#include <sys/_stdint.h>


bool motors_armed;
TIM_HandleTypeDef* htim;


/*************************************************************
 *Function name: Motors_Init
 *Description:   Start motor-related PWMs
 *Arguments:     none
 *Return value:  none
 ************************************************************/
void Motors_Init(TIM_HandleTypeDef htim3){

	// Start all PWMs
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start PWM1
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start PWM2
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM3
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Start PWM4
}

//#TODO: modificare la gestione di motor_armed
/*************************************************************
 *Function name: Motor_Arm_All
 *Description:   Arm all motors
 *Arguments:     none
 *Return value:  none
 ************************************************************/
void Motor_Arm_All(){
	//enable motors
	motors_armed = true;

	// Call the Motor_Arm() function for every motor
	for(int i=1; i<=4; i++){
		Motor_Arm(i);
	}
}

/*************************************************************
 *Function name: Motors_Htim_Start
 *Description:   Arm a motor
 *Arguments:     int variable to select the channel of the motor that has to be armed
 *Return value:  none
 ************************************************************/
void Motor_Arm(int channel){
	//channel value must be 1 or 4
	if(channel < 1 || channel > 4)	return;

	//motors should be enabled
	if(!motors_armed) return;

	// Call Motor_Write_up() function passing the constant related to armament of motors
	Motor_Write_up(channel, MOTOR_ARM_UP);
}

/*************************************************************
 *Function name: Motors_Off
 *Description  : Sends a 0% duty to the motors
 *Arguments    : none
 *Return value : none
 */
void Motors_Off () {
	Motor_Write_up(MOTOR_1, 0);
	Motor_Write_up(MOTOR_2, 0);
	Motor_Write_up(MOTOR_3, 0);
	Motor_Write_up(MOTOR_4, 0);

	motors_armed = false;
}


/*************************************************************
 *Function name: Motor_Write_up
 *Description  : Recalls Motor_Write_PWM function calculating the duty cycle value
 *Arguments    : int variable to select the motor, float variable to indicate the high value time
 *Return value : none
 ************************************************************/
void Motor_Write_up(int channel, float up)
{
	//channel value must be 1 or 4
	if(channel < 1 || channel > 4)	return;

	Motor_Write_PWM(channel, (up*100)/MOTOR_PWM_SIGNAL_PERIOD_UP);
}

/*************************************************************
 *Function name: Motors_Htim_Start
 *Description:   Set the power that has to be supplied to a motor
 *Arguments:     int variable to select the channel of the motor, float variable value (percentage) of the new duty cycle related to the selected motor
 *Return value:  none
 ************************************************************/
void Motor_Write_PWM(int channel, float value){
	//channel value must be 1 or 4
	if(channel < 1 || channel > 4)	return;

	//check if value is a percentage, if not in range 0-100 return
	if(value<=0 || value>=100)	return;

	// calculate the value of CCRx register (compare register), ARR is the MaxCOUNT of the timer
	float new_CCRx_value = TIM3->ARR * (value/100);
	// assign the value to the correct CCRx
	switch(channel){
	case 1:
			TIM3->CCR1 = new_CCRx_value;
			break;
	case 2:
			TIM3->CCR2 = new_CCRx_value;
			break;
	case 3:
			TIM3->CCR3 = new_CCRx_value;
			break;
	case 4:
			TIM3->CCR4 = new_CCRx_value;
			break;

	}



}
