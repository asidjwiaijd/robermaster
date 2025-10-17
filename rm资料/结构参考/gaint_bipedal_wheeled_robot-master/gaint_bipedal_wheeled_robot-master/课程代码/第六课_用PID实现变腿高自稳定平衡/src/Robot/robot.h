#ifndef ROBOT_H
#define ROBOT_H

#include "Arduino.h"
#include "bipedal_data.h"
#include "ppm.h"
#include "CAN_comm.h"
#include "Kinematics/Kinematics.h"
#include "pidsetting.h"
#include "Pid/pid.h"



extern float roll_kp , roll_kd;
// 轮足中两个轮毂电机的轮速
extern float motor1_vel, motor2_vel;
// 陀螺仪读取参数
extern float roll, pitch, yaw;
extern float gyroX, gyroY, gyroZ;
extern float balance_offset;
extern int ZeparamremoteValue; //腿高变化
extern float wheelTorqueTarget1 , wheelTorqueTarget2 ; // 电机目标值
extern float vel_kp ;
extern float balance_kp ;
extern float PIDstable_kd ;
extern float remoteBalanceOffset;
extern int speed_limit; //轮毂电机速度限制
extern PIDController PIDvel;
extern PIDController PIDstable;
extern PIDController PIDx;

void legControl(Node * LeftTarget, Node * RightTarget);
void mapPPMToRobotControl();
int mapJoystickValuerollzeparam(int inputValue);
float mapJoystickValuevel(int inputValue);
float mapJoystickValueInt(int inputValue);
float mapJoystickValuesteering(int inputValue);
void wheel_control();


#endif