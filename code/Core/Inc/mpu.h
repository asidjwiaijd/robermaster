
#ifndef MPU_H
#define MPU_H
#include "i2c.h"
#include "stm32f4xx_hal.h"
void MPU6050_Init(void);
void MPU6050_Read_Accel(float* Ax, float* Ay, float* Az);
void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz);
void MPU6050_Read_Temp(float* Temp);
void MPU6050_Printf_Euler(UART_HandleTypeDef* huart);
void MPU6050_Update_Euler(float* roll_deg, float* pitch_deg, float* yaw_deg);
void MPU6050_Gyro_Calibrate(uint16_t sample_num);
#endif /* MPU_H */