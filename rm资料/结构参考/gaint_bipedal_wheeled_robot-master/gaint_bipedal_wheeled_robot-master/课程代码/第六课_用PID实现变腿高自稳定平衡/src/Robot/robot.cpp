#include "robot.h"


// 左腿相关运动学基本参数
motionControlParams LeftMotionControlParams;
// 右腿相关运动学基本参数
motionControlParams RightMotionControlParams;
motorsparam motorsParam;
PIDController PIDvel = PIDController{0,0,0,0,5};
PIDController PIDstable = PIDController{0,0,0,0,5};
PIDController PIDx = PIDController{0,0,0,0,5};
int ZeparamremoteValue = 0;
float velTarget = 0;
float motor1_vel, motor2_vel;
int lastZeparamremoteValue = 0; // 上次输出的 ZeparamremoteValue
float  leftY = 150, rightY = 150;  //机器人初始高度
float  leftX = 60, rightX = 60; 
float roll, pitch, yaw; //陀螺仪xyz轴值
float gyroX, gyroY, gyroZ;//陀螺仪xyz轴加速度值
float steering = 0;
float remoteBalanceOffset = 0;//机器人重心调节
MIT devicesState[4];
extern float Am_kp;
float wheelTorqueTarget1 = 0, wheelTorqueTarget2 = 0; 


// 底部轮子霍尔电机控制函数
void wheel_control()
{
  
  float pitchTarget =  PIDvel(velTarget  - (-(motor1_vel  + motor2_vel) / 2));

  wheelTorqueTarget1 = clampToRange(( PIDstable.P*(pitchTarget - pitch ) + PIDstable.D * gyroY), -5, 5);
  wheelTorqueTarget2 = clampToRange(( PIDstable.P*(pitchTarget - pitch ) + PIDstable.D * gyroY), -5, 5);

  wheelTorqueTarget1 = wheelTorqueTarget1 - 0.65 * steering;
  wheelTorqueTarget2 = wheelTorqueTarget2 + 0.65 * steering;

  wheelTorqueTarget1 = clampToRange(wheelTorqueTarget1, -speed_limit, speed_limit);
  wheelTorqueTarget2 = clampToRange(wheelTorqueTarget2, -speed_limit, speed_limit);

}


// 将读取的PPM数据进行映射处理 转换为机器人控制参数
void mapPPMToRobotControl()
{

  // 这里用于控制腿部高度变化控制 采用增量式PID是为了让腿部快速变化时，保持稳定
  int error = 1000 - filteredPPMValues[0]; // 计算误差
  ZeparamremoteValue = -0.30 * error;      // 计算当前的值
  ZeparamremoteValue = constrainValue(ZeparamremoteValue, 0, 150);
  ZeparamremoteValue = ZeparamremoteValue + (ZeparamremoteValue - lastZeparamremoteValue) * 0.12; // 基于误差更新输出（增量控制）
  lastZeparamremoteValue = ZeparamremoteValue;                                                    // 存储当前值作为下一次计算的参考

  // 用于控制前进后退
  velTarget = -mapJoystickValuevel(filteredPPMValues[1]);
  // 用于调节重心偏置
  remoteBalanceOffset = mapJoystickValueInt(filteredPPMValues[2]);
  // 用于控制转向
  steering = -0.03 * (-mapJoystickValuesteering(filteredPPMValues[5]) - gyroZ);
  steering = constrainValue(steering, -10, 10);

}



// 轮足机器人控制部分程序
void legControl(Node * LeftTarget, Node * RightTarget)
{
  float XOffset =(-velTarget - (motor1_vel + (motor2_vel)) / 2);

  LeftTarget->y = leftY + ZeparamremoteValue ;
  RightTarget->y = rightY + ZeparamremoteValue ;
  
  LeftTarget->x = leftX + -PIDx(XOffset) ; //-0.02 * gyroY
  RightTarget->x = rightX + PIDx(XOffset) ; //-0.02 * gyroY
  // 限幅Y轴幅度 避免超限导致逆解出问题
  LeftTarget->y = constrainValue(LeftTarget->y, 130, 300);
  RightTarget->y = constrainValue(RightTarget->y, 130, 300);

}

int mapJoystickValuerollzeparam(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;

  int output = ((inputValue - 1500) / 20);

  return output;
}

float mapJoystickValuevel(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 100.0;
  return mappedValue;
}

// 摇杆轴数据的映射处理
float mapJoystickValueInt(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  float mappedValue = (inputValue - 1500) / 100.0;
  if (mappedValue > -0.7 && mappedValue < 0.7)
  {
    mappedValue = 0;
  }
  return mappedValue;
}
float mapJoystickValuesteering(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 2.0;
  return mappedValue;
}

