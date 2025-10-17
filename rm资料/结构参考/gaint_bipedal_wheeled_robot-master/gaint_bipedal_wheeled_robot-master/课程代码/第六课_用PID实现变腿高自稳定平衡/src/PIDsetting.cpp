#include "pidsetting.h"

// PID参数
float vel_kp = -0;//速度环
float balance_kp = -0;//直立环Kp
int speed_limit = 5; //轮毂电机速度限制
float robot_kp = 0;

//PID线性拟合函数
PIDValues  heightFit(int y_height) {

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
          PIDvel.P= -(pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp));
          PIDstable.P = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
          PIDstable.D = pid0.linear_PIDstable_kd + t * (pid1.linear_PIDstable_kd - pid0.linear_PIDstable_kd);
          PIDx.P = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp) ;
      } 
      else 
      {
          
          speed_limit = 3;
          float t = (y_height - y1) / (y2 - y1);
          PIDvel.P = -(pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp));
          PIDstable.P = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp) ;
          PIDstable.D = pid1.linear_PIDstable_kd + t * (pid2.linear_PIDstable_kd - pid1.linear_PIDstable_kd);
          robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
      }
      

     return result;
  
}

//PID串口调参函数
String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      command = received_chars;
      int commaPosition = command.indexOf(',');
      int newlinePosition = command.indexOf('\n');
      if (commaPosition != -1 && newlinePosition != -1) //给的第一个值
      {
        String firstParam = command.substring(0, commaPosition);
        vel_kp = firstParam.toDouble(); //速度环Kp
        Serial.print("vel_kp:");
        Serial.println(vel_kp);

        String secondParamStr = command.substring(commaPosition + 1, newlinePosition);
        int secondCommaPosition = secondParamStr.indexOf(',');
        if (secondCommaPosition != -1) //如果给三个值
        {
          balance_kp = secondParamStr.substring(0, secondCommaPosition).toDouble();//直立环Kp
          PIDstable.D = secondParamStr.substring(secondCommaPosition + 1).toDouble();//直立环Kd
        }
        else //如果只给两个值
        {
          robot_kp = secondParamStr.toDouble();
        }
        Serial.print("balance_kp:");
        Serial.println(balance_kp,3);
        Serial.print("PIDstable.D:");
        Serial.println(PIDstable.D,3);
        Serial.print("robot_kp:");
        Serial.println(robot_kp,3);
      }
      received_chars = "";
    }
  }
  return command;
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