/*
 * Motor.h
 *
 *  Created on: 26 nov 2021
 *      Author: Fabio Mecozzi
 */

#ifndef SRC_ALTIMETER_DUCTED_DRIVERS_MOTOR_H_
#define SRC_ALTIMETER_DUCTED_DRIVERS_MOTOR_H_

/*  -----------------------------------------------------------------------------------------------------
 * Description:	4 channels motor driver.
 * 				Default used timer channel is MTU3 and MTU4 with a PWM period of 20ms
 * 				This driver uses an automatic generated (by Renesas PDG2-"Peripheral Driver Generator 2")
 * 					low-level driver to interact with board peripherals
 *
 * Usage: 	-Motors are assigned to the port PC0 (channel 1), PC1 (channel 2), PE1 (channel 3) and PE2 (channel 4)
 * 				respectively on board pin JN1-23, JN2-9, JN2 22 and JN2 23
 * 			-Relay port is PD0 on pin JN2-13 used for both motors
 *  ----------------------------------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stm32h7xx.h>
#include <stm32h745xx.h>

#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4
#define MOTOR_PWM_SIGNAL_PERIOD_UP 20000.00	//20ms
#define MOTOR_ARM_UP 950	//Duty cycle 4.75% to arm up the motor
#define MOTOR_MIN_UP 1200	//Min value of duty cycle - 6%  Duty (min speed)
#define MOTOR_MAX_UP 1300	//Max value of duty cycle - 10% Duty (Max speed)
#define B_4 0.000001163 //thrust coefficient 4-cell battery
#define B_3 0.000001162 //thrust coefficient 3-cell battery
#define L 0.3375 //distance between motor and drone center
#define D 0.08 //drag coefficient TODO: needs to be calculated properly (it is the drag coefficient of the propeller)
#define MOTOR_MAX_SPEED_3 1220.6926 //sqrt(1/(4*B_3)+1/(2*L*B_3)+1/(4*D))
#define MOTOR_MAX_SPEED_4 1220.2435 //sqrt(1/(4*B_4)+1/(2*L*B_3)+1/(4*D))

extern bool motors_armed[4];

void Motors_Init(TIM_HandleTypeDef htim3);
void Motor_Arm_All();
void Motor_Arm(int channel);
void Motor_Write_PWM(int channel, float value);
void Motor_Write_up(int channel, float us);
void Motors_Off();


#endif /* SRC_ALTIMETER_DUCTED_DRIVERS_MOTOR_H_ */
