/******************************************************************************
 *Author: Omar Cocchairella
 ******************************************************************************/
/*******************************************************************************
* File Name    : lcd_buffer.h
* Version      : 1.02.02
* Description  : Header for AHRS.h class
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 29.09.2011     1.00        First release
*         : 02.10.2011	   1.01		   Optimised for reduced CPU load (di SOH Madgwick)
*         : 19.02.2012	   1.01.01	   Magnetometer measurement is normalised (di SOH Madgwick)
*         : 22.01.2018     1.02        Revisione della versione di Sebastian Giles da parte di Omar Cocchairella.
*         : 13.04.2019     1.02.02      Ragaini Davide commento della versione di Omar Cocchairella.
*******************************************************************************/



#ifndef AHRS_h
#define AHRS_h
#include <math.h>
#include <stdio.h>
#include <MPU6050.h>



/********************
 * Struttura dati contenente i dati filtrati degli angoli di Tait-Bryan e delle velocità angolari, sia in gradi che in radianti.
 ********************/
typedef struct{
	float RollRad;        // Corrisponde all'uscita in radianti dell'asse x di AHRS
	float PitchRad;       // Corrisponde all'uscita in radianti dell'asse y di AHRS
	float YawRad;         // Corrisponde all'uscita in radianti dell'asse z di AHRS
	float RollDeg;        // Corrisponde all'uscita in gradi dell'asse x di AHRS
	float PitchDeg;       // Corrisponde all'uscita in gradi dell'asse y di AHRS
	float YawDeg;         // Corrisponde all'uscita in gradi dell'asse z di AHRS
	float omegaRollRad;        // Corrisponde all'uscita in radianti dell'asse x di AHRS (velocità angolare)
	float omegaPitchRad;       // Corrisponde all'uscita in radianti dell'asse y di AHRS (velocità angolare)
	float omegaYawRad;         // Corrisponde all'uscita in radianti dell'asse z di AHRS (velocità angolare)
	float omegaRollDeg;        // Corrisponde all'uscita in gradi dell'asse x di AHRS (velocità angolare)
	float omegaPitchDeg;       // Corrisponde all'uscita in gradi dell'asse y di AHRS (velocità angolare)
	float omegaYawDeg;         // Corrisponde all'uscita in gradi dell'asse z di AHRS (velocità angolare)
}AHRS_data;

/********************
 * Struttura principale che viene utilizza per raggruppare tutte delle sottostrutture, contenenti dati
 * raw e filtrati letti dall'MPU6050.
 ********************/
typedef struct{
MAG_data mag;
IMU_sens sens;
IMU_raw raw;
IMU_temp temp;
AHRS_data ahrs_data;
}AHRS_out;

/***************************************************************************************
 * Function name: setAHRSFrequency
 * Description: Il filtro ha bisogno di sapere la frequenza con cui sarà chiamata la funzione
				madgwickFilterUpdate(). Prima di tutto va impostato questo valore.
 **************************************************************************************/
void setAHRSFrequency(float f);

/***************************************************************************************
 * Function name: madgwickFilterUpdate
 * Description: Questa funzione ricalcola l'assetto facendo uso di nuovi dati forniti dall'IMU.
				Per avere una buona stima dell'assetto è ideale chiamare questa funzione con la
				frequenza più alta possibile, dalla funzione setAHRSFrequency.
 * Arguments:
 * Return value:None.
 **************************************************************************************/
void madgwickFilterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/***************************************************************************************
 * Function name: getAHRS
 * Description: Questa funzione ricava gli angoli di Tait-Bryan.
				I valori vengono scritti nelle variabili passate per riferimento.
				Questa funzione può essere chiamata con una frequenza più bassa di quella con
				cui viene aggiornato il filtro, per esempio alla frequenza dell'algoritmo di
				controllo.
 * Arguments    : Puntatori a float pitch, yaw e roll.
 * Return value : None.
 **************************************************************************************/
void getAHRS(float* pitch, float* yaw, float* roll);

/***************************************************************************************
 * Function name: getQuat
 * Description  : Assegnamento dei quaternioni.
 * Arguments    :
 * Return value : None
 **************************************************************************************/
void getQuat(float* qa, float* qb, float* qc, float* qd);

/***************************************************************************************
 * Function name: getYPR
 * Description  : Trasforma le misurazioni del MARG in quaternioni e ne restituisce i dati
 * 				  in gradi e in radianti nella struttura dati AHRS_data.
 * Arguments    : Puntatori a struttura dati MAG_data, IMU_temp e AHRS_data.
 * Return value : None.
 **************************************************************************************/
void getYPR(MAG_data*, IMU_temp*, AHRS_data*);

/***************************************************************************************
 * Function name: calibrationYPR
 * Description  : Funzione di calibrazione del magnetometro e stampa su schermo dei parametri;
 * 				  al suo termine si prema SW1 per continuare.
 * Arguments    : Ammette puntatore a char e puntatore a struttura dati AHRS.
 * Return value : Restituisce su schermo i dati della calibrazione.
 **************************************************************************************/
void calibrationYPR(char*, MAG_data*);
/***************************************************************************************
 * Function name: calibrationYPR1
 * Description  : Funzione di calibrazione del magnetometro e stampa su schermo dei parametri;
 * 				  al suo termine si prema SW1 per continuare.
 * Arguments    : Ammette puntatore a char e puntatore a struttura dati AHRS.
 * Return value : Restituisce su schermo i dati della calibrazione di default.
 *
 *
 ***************************************************************************************/
void calibrationYPR1(char*, MAG_data*);


#endif
