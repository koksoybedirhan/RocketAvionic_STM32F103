/*
 * mpu6050.c
 *
 *  Created on: Oct 8, 2022
 *      Author: b1d0
 */

#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;

//Register Definitions
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define AXIS_START 0x03

//Variables
float Ax, Ay, Az, Gx, Gy, Gz, Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z;
float Temperature1;
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

//Kalman Definitions

static const double R0 = 40; // noise coavirance (normally 10)
static const double H0 = 1.00; //measurment map scalar
static double Q0 = 10; //initial estimated covariance
static double P0 = 0; //initial error covariance (it must be 0)
static double K0 = 0; //initial kalman gain

static const double R1 = 40; // noise coavirance (normally 10)
static const double H1 = 1.00; //measurment map scalar
static double Q1 = 10; //initial estimated covariance
static double P1 = 0; //initial error covariance (it must be 0)
static double K1 = 0; //initial kalman gain

static const double R2 = 40; // noise coavirance (normally 10)
static const double H2 = 1.00; //measurment map scalar
static double Q2 = 10; //initial estimated covariance
static double P2 = 0; //initial error covariance (it must be 0)
static double K2 = 0; //initial kalman gain

static const double R3 = 40; // noise coavirance (normally 10)
static const double H3 = 1.00; //measurment map scalar
static double Q3 = 10; //initial estimated covariance
static double P3 = 0; //initial error covariance (it must be 0)
static double K3 = 0; //initial kalman gain

static const double R4 = 40; // noise coavirance (normally 10)
static const double H4 = 1.00; //measurment map scalar
static double Q4 = 10; //initial estimated covariance
static double P4 = 0; //initial error covariance (it must be 0)
static double K4 = 0; //initial kalman gain

static const double R5 = 40; // noise coavirance (normally 10)
static const double H5 = 1.00; //measurment map scalar
static double Q5 = 10; //initial estimated covariance
static double P5 = 0; //initial error covariance (it must be 0)
static double K5 = 0; //initial kalman gain

static const double R6 = 40; // noise coavirance (normally 10)
static const double H6 = 1.00; //measurment map scalar
static double Q6 = 10; //initial estimated covariance
static double P6 = 0; //initial error covariance (it must be 0)
static double K6 = 0; //initial kalman gain

static const double R7 = 40; // noise coavirance (normally 10)
static const double H7 = 1.00; //measurment map scalar
static double Q7 = 10; //initial estimated covariance
static double P7 = 0; //initial error covariance (it must be 0)
static double K7 = 0; //initial kalman gain

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> Â± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> Â± 250 Â°/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

float MPU6050_Temperature(void)
{
	uint8_t Temp_Data[2];
	int16_t temp;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Temp_Data, 2, 1000);
	temp = (Temp_Data[0] << 8 | Temp_Data[1]);

	Temperature1 = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	return Temperature1;
}

void MPU6050_Accel_Config(void)
{
	uint8_t Accel_Data[6];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Accel_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Accel_Data[0] << 8 | Accel_Data [1]);
	Accel_Y_RAW = (int16_t)(Accel_Data[2] << 8 | Accel_Data [3]);
	Accel_Z_RAW = (int16_t)(Accel_Data[4] << 8 | Accel_Data [5]);
}

float MPU6050_Read_Accel_X(void)
{
	MPU6050_Accel_Config();
	Ax = Accel_X_RAW/16384.0;
	return Ax;
}

float MPU6050_Read_Accel_Y (void)
{
	MPU6050_Accel_Config();
	Ay = Accel_Y_RAW/16384.0;
	return Ay;
}

float MPU6050_Read_Accel_Z (void)
{
	MPU6050_Accel_Config();
	Az = Accel_Z_RAW/16384.0;
	return Az;
}

void MPU6050_Gyro_Config(void)
{
	uint8_t Gyro_Data[6];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Gyro_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Gyro_Data[0] << 8 | Gyro_Data [1]);
	Gyro_Y_RAW = (int16_t)(Gyro_Data[2] << 8 | Gyro_Data [3]);
	Gyro_Z_RAW = (int16_t)(Gyro_Data[4] << 8 | Gyro_Data [5]);
}

float MPU6050_Read_Gyro_X (void)
{
	MPU6050_Gyro_Config();
	Gx = Gyro_X_RAW/131.0;
	return Gx;
}

float MPU6050_Read_Gyro_Y (void)
{
	MPU6050_Gyro_Config();
	Gy = Gyro_Y_RAW/131.0;
	return Gy;
}

float MPU6050_Read_Gyro_Z (void)
{
	MPU6050_Gyro_Config();
	Gz = Gyro_Z_RAW/131.0;
	return Gz;
}

float MPU6050_Roll_Angle(void)
{
	double AccelY = MPU6050_Kalman_Accel_Y();
	double AccelZ = MPU6050_Kalman_Accel_Z();
	float Roll = 0;
	Roll = atan2(AccelY, AccelZ) * 180;
	return Roll;
}

float MPU6050_Kalman_Roll_Angle (void)
{
	float RollAng_U = MPU6050_Roll_Angle();

	static double RollAng_U_hat = 0; //initial estimated state

	K0 = P0 * H0 / (H0 * P0 * H0 + R0);
	RollAng_U_hat = RollAng_U_hat + K0 * (RollAng_U - H0 * RollAng_U_hat);
	P0 = (1 - K0 * H0) * P0 + Q0;

	return RollAng_U_hat;
}

double MPU6050_Kalman_Accel_X (void)
{
	double Accel_X_U = MPU6050_Read_Accel_X();

	static double Accel_X_U_hat = 0; //initial estimated state

	K1 = P1 * H1 / (H1 * P1 * H1 + R1);
	Accel_X_U_hat = Accel_X_U_hat + K1 * (Accel_X_U - H1 * Accel_X_U_hat);
	P1 = (1 - K1 * H1) * P1 + Q1;

	return Accel_X_U_hat;
}

double MPU6050_Kalman_Accel_Y (void)
{
	double Accel_Y_U = MPU6050_Read_Accel_Y();

	static double Accel_Y_U_hat = 0; //initial estimated state

	K2 = P2 * H2 / (H2 * P2 * H2 + R2);
	Accel_Y_U_hat = Accel_Y_U_hat + K2 * (Accel_Y_U - H2 * Accel_Y_U_hat);
	P2 = (1 - K2 * H2) * P2 + Q2;

	return Accel_Y_U_hat;
}

double MPU6050_Kalman_Accel_Z (void)
{
	double Accel_Z_U = MPU6050_Read_Accel_Z();

	static double Accel_Z_U_hat = 0; //initial estimated state

	K3 = P3 * H3 / (H3 * P3 * H3 + R3);
	Accel_Z_U_hat = Accel_Z_U_hat + K3 * (Accel_Z_U - H3 * Accel_Z_U_hat);
	P3 = (1 - K3 * H3) * P3 + Q3;

	return Accel_Z_U_hat;
}

double MPU6050_Kalman_Gyro_X (void)
{
	double Gyro_X_U = MPU6050_Read_Gyro_X();

	static double Gyro_X_U_hat = 0; //initial estimated state

	K4 = P4 * H4 / (H4 * P4 * H4 + R4);
	Gyro_X_U_hat = Gyro_X_U_hat + K4 * (Gyro_X_U - H4 * Gyro_X_U_hat);
	P4 = (1 - K4 * H4) * P4 + Q4;

	return Gyro_X_U_hat;
}

double MPU6050_Kalman_Gyro_Y (void)
{
	double Gyro_Y_U = MPU6050_Read_Gyro_Y();

	static double Gyro_Y_U_hat = 0; //initial estimated state

	K5 = P5 * H5 / (H5 * P5 * H5 + R5);
	Gyro_Y_U_hat = Gyro_Y_U_hat + K5 * (Gyro_Y_U - H5 * Gyro_Y_U_hat);
	P5 = (1 - K5 * H5) * P5 + Q5;

	return Gyro_Y_U_hat;
}

double MPU6050_Kalman_Gyro_Z (void)
{
	double Gyro_Z_U = MPU6050_Read_Gyro_Z();

	static double Gyro_Z_U_hat = 0; //initial estimated state

	K6 = P6 * H6 / (H6 * P6 * H6 + R6);
	Gyro_Z_U_hat = Gyro_Z_U_hat + K6 * (Gyro_Z_U - H6 * Gyro_Z_U_hat);
	P6 = (1 - K6 * H6) * P6 + Q6;

	return Gyro_Z_U_hat;
}

float MPU6050_Kalman_Temp(void)
{
	float Temp_U = MPU6050_Temperature();

	static double Temp_U_hat = 0; //initial estimated state

	K7 = P7 * H7 / (H7 * P7 * H7 + R7);
	Temp_U_hat = Temp_U_hat + K7 * (Temp_U - H6 * Temp_U_hat);
	P7 = (1 - K7 * H7) * P7 + Q7;

	return Temp_U_hat;
}
