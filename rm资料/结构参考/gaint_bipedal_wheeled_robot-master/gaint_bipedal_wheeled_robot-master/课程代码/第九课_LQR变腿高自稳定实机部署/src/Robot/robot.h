#ifndef ROBOT_H
#define ROBOT_H

#include "Kinematics/Kinematics.h"
#include "Arduino.h"
#include "bipedal_data.h"
#include "ppm.h"
#include "CAN_comm.h"
#include "Pid/pid.h"


extern float remoteBalanceOffset;
extern float robot_kp;
extern float roll_kp , roll_kd;
// 遥控器控制参数
extern float forwardBackward ;
// 轮足中两个轮毂电机的轮速
extern float motor1_vel, motor2_vel;
// 陀螺仪读取参数
extern float roll, pitch, yaw;
extern float gyroX, gyroY, gyroZ;
extern float balance_offset;
extern int ZeparamremoteValue; //腿高变化
extern float roll_EH;
extern float wheel_motor1_target , wheel_motor2_target ; // 电机目标值
extern float GyroBias;



void jump_control();
void robot_control(Node *LeftTarget, Node *RightTarget);
void mapPPMToRobotControl();
int mapJoystickValuerollzeparam(int inputValue);
float mapJoystickValuevel(int inputValue);
float mapJoystickValueInt(int inputValue);
float mapJoystickValuesteering(int inputValue);
void wheel_control(Node *LeftTarget, Node *RightTarget);
void calVMC_Right(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles);
void get_origin_pos();
void calVMC_Left(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles);
#endif