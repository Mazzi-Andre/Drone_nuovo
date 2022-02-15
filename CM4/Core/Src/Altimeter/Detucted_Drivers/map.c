/******************************************************************************
 *Author: Francesco Gaudeni
 ******************************************************************************/
/*******************************************************************************
* File Name    : map.c
* Version      : 2.00
* Description  : Calculate the distance for the altimeter
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 03.02.2018     1.00        First release
*         : 06.12.2018	   2.00		   Converts the speed in a measure that can be read by the motors
*
*
*******************************************************************************/

#include "map.h"
inline float map(float val, float from_src, float to_src, float from_dst, float to_dst)
{
	return (((to_dst-from_dst)/(to_src-from_src))*(val-from_src)) + from_dst;
}
