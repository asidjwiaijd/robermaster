#ifndef PID_H
#define PID_H

#include "Arduino.h"
#include "Robot/robot.h"

/*线性拟合PID结构体*/
struct PIDValues {
  float linear_vel_kp;
  float linear_balance_kp;
  float linear_angular_vel_feedback;
  float linear_robot_kp;
};

extern float vel_kp ;
extern float balance_kp ;
extern float angular_vel_feedback ;
extern int speed_limit; //轮毂电机速度限制


PIDValues  interpolatePID(int y_height);
float clampToRange(float value, float minVal, float maxVal);

#endif