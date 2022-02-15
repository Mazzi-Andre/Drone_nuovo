/*
 * MPU6050.h
 *
 *  Created on: 17 dic 2021
 *      Author: 39345
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_



#endif /* INC_MPU6050_H_ */

#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_i2c.h>
#include <string.h>
#include <sys/_stdint.h>
#include <stdbool.h>


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


//Inizializzazione delle struct per gli AHRS

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


//funzione init generale
int MPU6050_Init(void);
void MPU6050_Init_Gir(void);

//funzione di lettura accellerometro
void MPU6050_Read_Accel(IMU_raw* Imu_raw);
//funzione di lettura giroscopio
void MPU6050_Read_Gir(IMU_raw* Imu_raw);
//inizializzazione magnetometro e settagio valori di default
int MPU6050_Mag_Init(MAG_data* mag_data);
//lettura registri dati magnetometro
int mag_read_raw(MAG_data* mag_data);
//Lettura dati magnetometro
int mag_read(MAG_data* mag_data);
//settaggio bypass dell'MPU per accedere al magnetometro
int mpu_set_bypass();
//calibrazione magnetometro
int magcal(MAG_data* mag_data);






