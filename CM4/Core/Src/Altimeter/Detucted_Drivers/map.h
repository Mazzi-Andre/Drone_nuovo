/******************************************************************************
 *Author: Francesco Gaudeni
 ******************************************************************************/
/*******************************************************************************
* File Name    : map.h
* Version      : 2.00
* Description  : header for map class
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 03.02.2018     1.00        First release
*         : 06.12.2018	   2.00		   Update
*******************************************************************************/
#ifndef DUCTED_DRIVERS_MAP_H_
#define DUCTED_DRIVERS_MAP_H_

/*************************************************************
 *Function name: map
 *Description:   Function used to map a value from a range to an other range
 *Arguments:     five float values: val (value to convert), from_src (first range's minimum value), to_src (first range's maximum value), from_dst (final range's minimum value), to_dst (final range's maximum value)
 *Return value:  initial value mapped into the final range
 ************************************************************/
float map(float val, float from_src, float to_src, float from_dst, float to_dst);

#endif /* DUCTED_DRIVERS_MAP_H_ */
