/******************************************************************************
 *Author: Omar Cocchairella
 ******************************************************************************/
/*******************************************************************************
* File Name: mag.h
* Version: 1.01.1
* Description: Header for the mag.c class
*******************************************************************************/
/*******************************************************************************
* History
* 			DD.MM.YYYY     Version     Description
*			22.01.2018     1.01        Revisione della versione di Sebastian Giles
* 										da parte di Omar Cocchairella.
*			10.01.2019     1.01.1      Martin Breccia commento della versione
* 										di Omar Cocchairella.
*******************************************************************************/


#ifndef SRC_MAG_H_  /* Multiple inclusion prevention. */
#define SRC_MAG_H_

#define HMC5983_ADDRESS 0x1E << 1
#define HMC5983_WRITE	0x3C
#define HMC5983_READ 	0x3D

#define HMC5983_CONF_A		0x00
#define HMC5983_CONF_B		0x01
#define HMC5983_MODE		0x02
#define HMC5983_OUT_X_MSB	0x03
#define HMC5983_OUT_X_LSB	0x04
#define HMC5983_OUT_Z_MSB	0x05
#define HMC5983_OUT_Z_LSB	0x06
#define HMC5983_OUT_Y_MSB	0x07
#define HMC5983_OUT_Y_LSB	0x08
#define HMC5983_STATUS		0x09
#define HMC5983_ID_A		0x0A
#define HMC5983_ID_B		0x0B
#define HMC5983_ID_C		0x0C
#define HMC5983_TEMP_OUT_MSB	0x31
#define HMC5983_TEMP_OUT_LSB	0x32

#define HMC5983_ID_A_VAL    0x48
#endif /* SRC_MAG_H_  Multiple inclusion prevention. */
