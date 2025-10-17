#ifndef PIDSETTING_H
#define PIDSETTING_H

#include "Arduino.h"
#include "Robot/robot.h"

/*线性拟合PID结构体*/
struct PIDValues {
  float linear_vel_kp;
  float linear_balance_kp;
  float linear_PIDstable_kd;
  float linear_robot_kp;
};



String serialReceiveUserCommand();
PIDValues heightFit(int y_height);
float clampToRange(float value, float minVal, float maxVal);

#endif