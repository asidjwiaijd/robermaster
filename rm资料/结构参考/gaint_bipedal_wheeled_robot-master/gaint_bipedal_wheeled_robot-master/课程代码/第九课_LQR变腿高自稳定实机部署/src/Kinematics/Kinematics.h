#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "bipedal_data.h"
#include "config.h"
// 逆解控制参数
#define L1 150
#define L2 250
#define L3 250
#define L4 150
#define L5 108

extern float gyroX, gyroY, gyroZ;
extern float roll, pitch, yaw;

typedef struct {
    float x;
    float y;
    
    float polarAngle;//极角
    float radius;//腿长
    

    float frontTorque;
    float backTorque;
} Node;




typedef struct 
{
    float alphaLeft;   // 左腿α角度
    float betaLeft;     // 左腿β角度
    float alphaRight;  // 右腿α角度
    float betaRight;   // 右腿β角度
    float thetaRight1;
    float thetaLeft1;
}JointAngles;



float constrainValue(float value, float minValue, float maxValue);
void inverseKinematics(Node * LeftTarget, Node * RightTarget ,JointAngles * jointAngles);
void forwardKinematics(Node *LeftTarget, Node *RightTarget, JointAngles *jointAngles);
void forwardKinematics_right(Node *RightTarget, JointAngles *jointAngles);
void forwardKinematics_left(Node *LeftTarget, JointAngles *jointAngles);

#endif