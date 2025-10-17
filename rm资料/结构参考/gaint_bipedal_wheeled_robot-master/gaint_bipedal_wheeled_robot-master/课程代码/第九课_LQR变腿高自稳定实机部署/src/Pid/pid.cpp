#include "pid.h"

// PID参数
float vel_kp = -0;//速度环
float balance_kp = -0;//直立环Kp
float angular_vel_feedback = 0;
int speed_limit = 5; //轮毂电机速度限制
float robot_kp = 0;

//PID线性拟合函数
PIDValues  interpolatePID(int y_height) {
  if(Shake_shoulder == 0)
  {
      // 已知数据点
      float y0 = 0, y1 = 75, y2 = 150;
      PIDValues pid0 = {-0.525,-0.17, 0.015, 3.0};
      PIDValues pid1 = {-0.525,-0.155,0.015, 7};
      PIDValues pid2 = {-0.55,-0.16, 0.025, 9};

      PIDValues result;
      if (y_height <= y1) 
      {
          speed_limit = 5;
          float t = (y_height - y0) / (y1 - y0);
          vel_kp = pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp);
          balance_kp = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
          angular_vel_feedback = pid0.linear_angular_vel_feedback + t * (pid1.linear_angular_vel_feedback - pid0.linear_angular_vel_feedback);
          robot_kp = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp) ;
      } 
      else 
      {
          
          speed_limit = 3;
          float t = (y_height - y1) / (y2 - y1);
          vel_kp = pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp);
          balance_kp = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp) + 0.03;
          angular_vel_feedback = pid1.linear_angular_vel_feedback + t * (pid2.linear_angular_vel_feedback - pid1.linear_angular_vel_feedback);
          robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
      }
      return result;
  }
}



// 限幅函数
float clampToRange(float value, float minVal, float maxVal)
{
  if (value < minVal)
    return minVal;
  if (value > maxVal)
    return maxVal;
  return value;
}