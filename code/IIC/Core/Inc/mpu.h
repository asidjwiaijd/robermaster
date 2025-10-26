
#ifndef MPU_H
#define MPU_H
#include "i2c.h"
void MPU6050_Init(void);
void MPU6050_Read_Accel(float* Ax, float* Ay, float* Az);
void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz);
void MPU6050_Read_Temp(float* Temp);


#endif /* MPU_H */