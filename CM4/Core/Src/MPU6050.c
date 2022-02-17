/*
 * MPU6050.c
 *
 *  Created on: 16 dic 2021
 *      Author: meefa
 */

#include "MPU6050.h"
int MPU6050_Init() {
	uint8_t check_id, data;
	if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check_id,
			sizeof(check_id), 2 * DEFAULT_TIMEOUT))
		return 1;
	if (check_id != I_AM_MPU6050)
		return 2;
	data = DEFAULT_SMPLRT_DIV;
	if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data,
			sizeof(data), DEFAULT_TIMEOUT))
		return 3;
	HAL_Delay(DEFAULT_TIMEOUT);
	data = 0b00000000; // 0b00000000 -->viene settato a 0, quindi 2g  0x00
	if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data,
			sizeof(data), DEFAULT_TIMEOUT))
		return 5;
	HAL_Delay(DEFAULT_TIMEOUT);
	data = 0b00000000;
	if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data,
			sizeof(data), DEFAULT_TIMEOUT))
		return 6;
	HAL_Delay(DEFAULT_TIMEOUT);
	data = 0;
	if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data,
			sizeof(data), DEFAULT_TIMEOUT))
		return 7;
	HAL_Delay(DEFAULT_TIMEOUT);
	return 0;
}

void MPU6050_Init_Gir(void) {
	uint8_t check, data;
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
	if (check == 104) {
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1,
				50);
		HAL_Delay(50);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1,
				50);
		HAL_Delay(50);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1,
				50);
		HAL_Delay(50);
		data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1,
				50);
		HAL_Delay(50);
	}
}

//read Roll, Pitch and yaw acceleration
//TOOD: si punta sui delay
void MPU6050_Read_Accel(IMU_raw *Imu_raw, IMU_temp *Imu_temp) {
	uint8_t recData[6];
	memset(recData, 0, sizeof(recData));
	//8 bytes are required, first and second bytes of recData will be populated with x-axis data, third and fourth y-axis data and fith and sith with z-axis data
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG,
			I2C_MEMADD_SIZE_8BIT, recData, sizeof(recData),
			2 * DEFAULT_TIMEOUT);
//	HAL_Delay(50);
	;

	Imu_raw->accRoll = (int16_t) (recData[0] << 8 | recData[1]);
	Imu_raw->accPitch = (int16_t) (recData[2] << 8 | recData[3]);
	Imu_raw->accYaw = (int16_t) (recData[4] << 8 | recData[5]);

	Imu_temp->accRoll = Imu_raw->accRoll / 16384;
	Imu_temp->accPitch = Imu_raw->accPitch / 16384;
	Imu_temp->accYaw = Imu_raw->accYaw / 16384;
}

void MPU6050_Read_Gir(IMU_raw *Imu_raw, IMU_temp *Imu_temp) {
	uint8_t recData[6];
	memset(recData, 0, sizeof(recData));
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG,
			I2C_MEMADD_SIZE_8BIT, recData, 6, 100);
//	HAL_Delay(50);

	Imu_raw->gyrRoll = (int16_t) (recData[0] << 8 | recData[1]);  // angolo x
	Imu_raw->gyrPitch = (int16_t) (recData[2] << 8 | recData[3]); // angolo y
	Imu_raw->gyrYaw = (int16_t) (recData[4] << 8 | recData[5]);

	Imu_temp->gyrRoll = Imu_raw->gyrRoll / 131;
	Imu_temp->gyrPitch = Imu_raw->gyrPitch / 131;
	Imu_temp->gyrYaw = Imu_raw->gyrYaw / 131;

}
//TODO: ripristinare le funzioni
//initialize magnetometer - checking id, configuring registers, setting MAG_data parameters
int MPU6050_Mag_Init(MAG_data *mag_data) {
	uint8_t check_id, data;
	check_id=0x00;
	data=0x00;
	/*
	 //setting bypass if is off
	 if (!bypass)
	 if (mpu_set_bypass())
	 return 1;

	 // checking HMC5893 address
	 if (HAL_I2C_Mem_Read(&hi2c2, HMC5983_ADDRESS, HMC5983_ID_A, 1, &check_id,
	 sizeof(check_id), 2 * DEFAULT_TIMEOUT))
	 return 2;
	 //HMC5893 configuration - checking id
	 if (check_id == HMC5983_WHO_I_AM_REG)
	 return 3;
	 //setting contiuos mode
	 data = 0;
	 if (HAL_I2C_Mem_Write(&hi2c2, HMC5983_ADDRESS, HMC5983_MODE, 1, &data,
	 sizeof(data), 2 * DEFAULT_TIMEOUT))
	 return 0x4;
	 //reading CONF A register
	 if (HAL_I2C_Mem_Read(&hi2c2, HMC5983_ADDRESS, HMC5983_CONF_A, 1, &data,
	 sizeof(data), 2 * DEFAULT_TIMEOUT))
	 return 0x4;
	 data |= 0x1C;
	 //setting 220hz
	 if (HAL_I2C_Mem_Write(&hi2c2, HMC5983_ADDRESS, HMC5983_CONF_A, 1, &data,
	 sizeof(data), 2 * DEFAULT_TIMEOUT))
	 return 0x5;
	 //reading CONF B register
	 if (HAL_I2C_Mem_Read(&hi2c2, HMC5983_ADDRESS, HMC5983_CONF_B, 1, &data,
	 sizeof(data), 2 * DEFAULT_TIMEOUT))
	 return 0x6;
	 data = data >> 5;
	 */
	//setting default sense
	switch (data) {
	case 0:
		mag_data->sens = 0.73;
		break;
	case 1:
		mag_data->sens = 0.92;
		break;
	case 2:
		mag_data->sens = 1.22;
		break;
	case 3:
		mag_data->sens = 1.52;
		break;
	case 4:
		mag_data->sens = 2.27;
		break;
	case 5:
		mag_data->sens = 2.56;
		break;
	case 6:
		mag_data->sens = 3.03;
		break;
	case 7:
		mag_data->sens = 4.35;
		break;
	default:
		break;
	}

	//setting default scale, bias and ABS
	mag_data->scale[0] = 0.939307;
	mag_data->scale[1] = 0.967911;
	mag_data->scale[2] = 1.108362;
	mag_data->bias[0] = -184.460;
	mag_data->bias[1] = 136.160;
	mag_data->bias[2] = 216.660;
	mag_data->ABS = 205438.578;
	return 0x0;
}
//TODO: sistemare bypass, togliere il commento e provare read e read_raw
/*
 //read register in which measurements are stored and write in mag_data->raw
 int mag_read_raw(MAG_data* mag_data) {
 uint8_t data[6];
 memset(data, 0, sizeof(data));

 if (HAL_I2C_Mem_Read(&hi2c2, HMC5983_ADDRESS, HMC5983_OUT_X_MSB, 6, data,
 sizeof(data), 2 * DEFAULT_TIMEOUT))
 return 0x1;
 mag_data->raw[0] = (data[0] << 8) | data[1];

 mag_data->raw[2] = (data[2] << 8) | data[3];
 mag_data->raw[1] = (data[4] << 8) | data[5];

 return 0x0;

 }

 //write data in mag_data->x, mag_data->y, mag_data->z
 int mag_read(MAG_data* mag_data) {
 if (mag_read_raw(mag_data))
 return 0x1;

 mag_data->x =
 ((float) mag_data->raw[0] * mag_data->sens - mag_data->bias[0])
 * mag_data->scale[0];
 mag_data->y =
 ((float) mag_data->raw[1] * mag_data->sens - mag_data->bias[1])
 * mag_data->scale[1];
 mag_data->z =
 ((float) mag_data->raw[2] * mag_data->sens - mag_data->bias[2])
 * mag_data->scale[2];

 return 0x0;
 }

 // set bypass to 1 in order to communicate with magnetometer
 int mpu_set_bypass() {
 uint8_t data;
 data = 0x00;
 if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6A, 1, &data, sizeof(data),
 2 * DEFAULT_TIMEOUT)) //set i2c Master enable bit
 return 1;
 data = 0x02;
 if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x37, 1, &data, sizeof(data),
 2 * DEFAULT_TIMEOUT)) //set I2C Bypass enable bit
 return 2;
 data = 0x00;
 if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &data, sizeof(data),
 2 * DEFAULT_TIMEOUT)) //Turn off sleep mode by reseting SLEEP bit
 return 3;
 bypass = 1;
 return 0;
 }
 */
int magcal(MAG_data *mag_data) {
	mag_data->scale[0] = 1.0;
	mag_data->scale[1] = 1.0;
	mag_data->scale[2] = 1.0;
	mag_data->bias[0] = 0.0;
	mag_data->bias[1] = 0.0;
	mag_data->bias[2] = 0.0;

	uint16_t ii = 0;

	float mag_rad[3] = { 0, 0, 0 }, mag_max[3] = { -32767, -32767, -32767 },
			mag_min[3] = { 32767, 32767, 32767 };

	HAL_Delay(4000);

	for (ii = 0; ii < 3000; ii++) {

		//TODO: decommentare quando funziona mag_read
//		mag_read(mag_data);  // Read the mag data

		if (mag_data->x > mag_max[0])
			mag_max[0] = mag_data->x;
		if (mag_data->y > mag_max[1])
			mag_max[1] = mag_data->y;
		if (mag_data->z > mag_max[2])
			mag_max[2] = mag_data->z;
		if (mag_data->x < mag_min[0])
			mag_min[0] = mag_data->x;
		if (mag_data->y < mag_min[1])
			mag_min[1] = mag_data->y;
		if (mag_data->z < mag_min[2])
			mag_min[2] = mag_data->z;

		HAL_Delay(5);
	}

	// Get hard iron correction
	mag_data->bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_data->bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_data->bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	// Get soft iron correction estimate
	mag_rad[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_rad[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_rad[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_rad[0] + mag_rad[1] + mag_rad[2];
	avg_rad /= 3.0;

	mag_data->scale[0] = avg_rad / ((float) mag_rad[0]);
	mag_data->scale[1] = avg_rad / ((float) mag_rad[1]);
	mag_data->scale[2] = avg_rad / ((float) mag_rad[2]);
	mag_data->ABS = avg_rad * avg_rad;
	return 0;
}  // FINE - magcal(..)

