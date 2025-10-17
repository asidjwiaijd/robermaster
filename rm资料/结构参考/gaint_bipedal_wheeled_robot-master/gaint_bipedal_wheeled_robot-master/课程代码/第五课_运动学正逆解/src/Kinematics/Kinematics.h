#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// 逆解控制参数
#define L1 150
#define L2 250
#define L3 250
#define L4 150
#define L5 108



typedef struct {
    float x;
    float y;
} Node;


typedef struct 
{
    float alphaLeft;   // 左腿α角度
    float betaLeft;     // 左腿β角度
    float alphaRight;  // 右腿α角度
    float betaRight;   // 右腿β角度
}JointAngles;



float constrainValue(float value, float minValue, float maxValue);
void limit_Y(float Y1, float Y2);
void inverseKinematics(Node * LeftTarget, Node * RightTarget ,JointAngles * jointAngles);
void forwardKinematics(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles);

#endif                                                  