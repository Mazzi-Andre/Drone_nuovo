/*
 * MPU6050.h
 *
 *  Created on: 17 dic 2021
 *      Author: 39345
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include <stdbool.h>
#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_i2c.h>
#include <string.h>
#include <sys/_stdint.h>
#include <mag.h>


#define MPU6050_ADDR 0xD0 // addres of the peripherical
#define SMPLRT_DIV_REG 0x19 // register for sample rate divider (read and write) [sample rate = 8kHz / (1 + value in the register)
#define GYRO_CONFIG_REG 0x1B //
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0X75 // address of register in which should be written I_AM_MPU6050 value
#define I_AM_MPU6050 104
#define DEFAULT_SMPLRT_DIV 0x07 // divider for sample rate for initialization
#define DEFAULT_TIMEOUT 50

//extern I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c2;


// Variabili globali di prova
int16_t Accel_X_RAW,Accel_Y_RAW,Accel_Z_RAW;
int16_t Ax,Ay,Az;


int16_t Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW;
int16_t Gx,Gy,Gz;


/*
 * Inizializzazione delle strutture utilizzate per i dati di lettura dell'accellerometro,
 * del giroscopio e del magnetometro
 */
// Struttura per i dati raw del magnetometro
typedef struct {
short raw[3];
float x,y,z;
float sens;
float bias[3];
float scale[3];
float ABS;
} MAG_data;



typedef struct {
float accRoll;
float accPitch;
float accYaw;
float gyrRoll;
float gyrPitch;
float gyrYaw;
} IMU_raw;



typedef struct {
uint16_t acc_sens;
float gyr_sens;
} IMU_sens;



typedef struct {
float accRoll;
float accPitch;
float accYaw;
float gyrRoll;
float gyrPitch;
float gyrYaw;
} IMU_temp;



typedef struct{
float Roll_Rad_Ref;
float Pitch_Rad_Ref;
float Yaw_Rad_Ref;
} IMU_rif;


/***************************************************************************************
* Function name: MPU6050_Init
* Description  :Funzione di inizializzazione dell'IMU, che tramite gli indirizzi verifica il funzionamento
* 				 dell'MPU6050 e configura tramite gli indirizzi collegati, accellerometro e giroscopio.
* Return value : Restituisce un'intero nel caso le funzioni restituiscano errore, sennò 0.
*
*
***************************************************************************************/
int MPU6050_Init(void);
void MPU6050_Init_Gir(void);

/***************************************************************************************
 * Function name: MPU6050_Read_Accel
 * Description  :Funzione di lettura tramite le HAL dei dati grezzi dell'accellerometro, nella quale
 * 				 vengono direttamente filtrati direttamente la sensibilità prefissata e inseriti allìinterno
 * 				 della struttura IMU_temp.
 * Arguments    : Ammette puntatore a struttura dati AHRS.
 * Return value : None.
 *
 ***************************************************************************************/

void MPU6050_Read_Accel(IMU_raw* Imu_raw, IMU_temp *Imu_temp);
/***************************************************************************************
 * Function name: MPU6050_Read_Gir
 * Description  :Funzione di lettura tramite le HAL dei dati grezzi del giroscopio, nella quale
 * 				 vengono direttamente filtrati direttamente la sensibilità prefissata e inseriti allìinterno
 * 				 della struttura IMU_temp.
 * Arguments    : Ammette puntatore a struttura dati AHRS.
 * Return value : Restituisce su struttura i dati dell'accellerometro.
 *
 ***************************************************************************************/
void MPU6050_Read_Gir(IMU_raw* Imu_raw, IMU_temp *Imu_temp);
/***************************************************************************************
 * Function name: MPU6050_Mag_Init
 * Description  :Funzione di inizializzazione del magnetometro, nella quale tramite una funzione di bypass
 * 				 si attiva un I2c interno all'MPU che ci permette la comunicazione attraverso il nostro
 * 				 magnetometro, e di conseguenza la configurazione iniziale tramite gli indirizzi associati.
 * Arguments    : Ammette puntatore a struttura dati AHRS.
 * Return value : Restituisce su struttura i dati del giroscopio.
 *
 ***************************************************************************************/
int MPU6050_Mag_Init(MAG_data* mag_data);
/***************************************************************************************
 * Function name: mag_read_raw
 * Description  :Funzione di lettura che tramite le HAL collegate direttamente agli indirizzi del magnetometro
 * 				 ci restituiscono i dati grezzi.
 * Arguments    : Ammette puntatore a struttura dati AHRS.
 * Return value : Restituisce su struttura i dati raw del magnetometro.
 *
 ***************************************************************************************/
int mag_read_raw(MAG_data* mag_data);
/***************************************************************************************
 * Function name: mag_read
 * Description  :Funzione che ci permette di prendere i dati grezzi del magnetometro e tramite le sensibilità
 * 				 associate (default in questo caso) andare a filtrare i nostri dati.
 * Arguments    : Ammette puntatore a struttura dati AHRS.
 * Return value : Restituisce su struttura i dati filtrati del magnetometro.
 *
 ***************************************************************************************/
int mag_read(MAG_data* mag_data);
/***************************************************************************************
 * Function name: mpu_set_bypass
 * Description  :Funzione che serve per attivare il bypass del'MPU6050 che ci permette di attivare l'I2c di
 * 				 comunicazione con la nostra periferica.
 * Arguments    : None.
 * Return value : None.
 *
 ***************************************************************************************/
int mpu_set_bypass();
/***************************************************************************************
 * Function name: magcal
 * Description  :Funzione utilizzata per la calibrazione del magnetometro, che deve essre utilizzata nel
 * 				 caso in cui si utilizzi il magnetometro in una posizione nuova rispetto alla precedente.
 * Arguments    : None.
 * Return value : None.
 *
 ***************************************************************************************/
int magcal(MAG_data* mag_data);







#endif /* INC_MPU6050_H_ */
