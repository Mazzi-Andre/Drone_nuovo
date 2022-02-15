/******************************************************************************
 *Author: Francesco Gaudeni
 ******************************************************************************/
/*******************************************************************************
* File Name    : PID.h
* Version      : 2.00
* Description  : Header for PID.c
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 03.02.2018     1.00        First release
*         : 06.12.2018	   2.00		   Update
*******************************************************************************/
#ifndef SRC_DUCTED_DRIVERS_PID_H_
#define SRC_DUCTED_DRIVERS_PID_H_

typedef struct {
	float kp, kd, ki; //coefficients of proportional, derivative, integral controller
	float dt; //step for the discrete control
	float lastError; //error at time t-1
	float ITerm; //integral term
	float outMax;
	float outMin;
} PID_config;

void PID_Init(PID_config* conf, float kp, float kd, float ki, float dt, float outMin, float outMax);
float PID_Compute(float input, float setPoint, PID_config* conf);

#endif /* SRC_DUCTED_DRIVERS_PID_H_ */
