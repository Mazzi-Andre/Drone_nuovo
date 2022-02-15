/******************************************************************************
 *Author: Alessandro De Toni
 ******************************************************************************/
/*******************************************************************************
* File Name    : PID.c
* Version      : 2.01
* Description  : Class to calculate the PID's parameter
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 03.02.2018     1.00        First release
*         : 06.12.2018	   2.00		   Update
*         : 19.12.2019	   2.01		   Added important comments
*******************************************************************************/
#include "PID.h"

/*************************************************************
 *Function name: PID_Init
 *Description:   Initialize all Pid variables
 *Arguments:     pointer 'conf' to PID_config struct, six float variables
 *Return value:  none
 ************************************************************/
void PID_Init(PID_config* conf, float kp, float kd, float ki, float dt, float outMin, float outMax)
{
	conf->kp = kp;
	conf->kd = kd;
	conf->ki = ki;
	conf->dt = dt;
	conf->ITerm = 0;
	conf->lastError = 0;
	conf->outMax = outMax;
	conf->outMin = outMin;
}

/*************************************************************
 *Function name: PID_Compute
 *Description:   Calculate new output value
 *Arguments:     two float variables to indicate the input from the sensors and the desired value
 *Return value:  float variable
 ************************************************************/
float PID_Compute(float input, float setPoint, PID_config* conf)
{
      /*Compute all the working error variables*/
      float error = setPoint - input;

      /*calculates the integral of error (adding the new error)
       	known the previous value of the integral of the error, we add the error in the new step,
        thus we obtain the area under the error function step by step*/
      conf->ITerm += (error * conf->dt)* conf->ki;

      if((conf->ITerm) > conf->outMax) conf->ITerm = conf->outMax;
      else if((conf->ITerm) < conf->outMin) conf->ITerm = conf->outMin;
      float dInput = (error - conf->lastError) / conf->dt; //derivative of the exit value (input). ((e(t)-e(t-1))/dt

      /*Compute PID Output*/
      float output = (conf->kp * error) + (conf->ITerm) + (conf->kd * dInput); //sum of the outputs of the 3 PIDs

	  if(output > conf->outMax) output = conf->outMax;
      else if(output < conf->outMin) output = conf->outMin;

      /*Remember some variables for next time*/
	  conf->lastError = error;
	  return output;
}
