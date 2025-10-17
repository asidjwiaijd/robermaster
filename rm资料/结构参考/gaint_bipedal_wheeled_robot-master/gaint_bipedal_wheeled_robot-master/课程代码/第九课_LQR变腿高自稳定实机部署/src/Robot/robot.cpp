#include "robot.h"


// 左腿相关运动学基本参数
motionControlParams LeftMotionControlParams;
// 右腿相关运动学基本参数
motionControlParams RightMotionControlParams;

uint32_t last_time = 0;
uint32_t last_time1 = 0;
float nowLeft_y = 0;
float nowRight_y = 0;
float nowLeft_x = 0;
float nowRight_x = 0;
float radius= 0.07;
float position = 0; // 累计位移，初始为0
float roll_kp = 0.0025, roll_kd = -0.007;  //机器人自稳Kp、Kd值
float leftY = 140, rightY = 140;  //机器人Y轴方向腿部高度
float leftX = 60, rightX = 60;    //机器人X轴方向腿部高度
int ZeparamremoteValue = 0;
float roll_EH;
float forwardBackward = 0;
float motor1_vel, motor2_vel;
int lastZeparamremoteValue = 0; // 上次输出的 ZeparamremoteValue
int EH = 0;
float roll, pitch, yaw; //陀螺仪xyz轴值
float gyroX, gyroY, gyroZ;//陀螺仪xyz轴加速度值
float target_roll = 0.0;
float steering = 0;
float remoteBalanceOffset = 0;
MIT devicesState[4];
extern float Am_kp;
extern motorsparam motorsParam;
uint8_t origin_pos_flag = 1;
float jump_vlaue; //起跳高度
float motor1_target_vel = 0, motor2_target_vel = 0;
float wheel_motor1_target = 0, wheel_motor2_target = 0; 
//轮毂
float K1 = 0;         
float K2 = 0;         
float K3 = 0;         
float K4 = 0;        



//位移计算
float updatePosition()
{
  uint32_t now = micros();
  if(last_time == 0)
  {
    last_time = now;
  }
  float delta_t = (now - last_time) * 1e-6f;
  last_time = now;
  float v = -(motor1_vel *radius  + motor2_vel*radius) / 2;
  position += v*delta_t;
  float POS_MAX = 0.1;  // 10 cm
  if (position >  POS_MAX) position =  POS_MAX;
  if (position < -POS_MAX) position = -POS_MAX;
  return position;//单位m
}

//LQR拟合函数
float calc_K1(float h) { return 0.000687*h*h-0.247225*h+6.542786; }
float calc_K2(float h) { return 0.000005*h*h-0.010043*h-0.658127; }
float calc_K3(float h) { return -0.000000*h*h+0.000000*h+-0.000000; }
float calc_K4(float h) { return 0.000058*h*h-0.020423*h-2.622680; }


// 底部轮子霍尔电机控制函数
void wheel_control(Node *LeftTarget, Node *RightTarget)
{
 
  float vel,velOffset;
  float f = 0.2;
  updatePosition();
  K1 = -14.8198*0.5;//最高高度
  K2 = -3.1558;
  K3 =-0.7071*1;
  K4 =-3.7071*1;
  
    
  vel = -((motor1_vel *radius  + motor2_vel*radius) / 2);
  if(vel >0.01) velOffset = f;
  if(vel <0.01) velOffset = -f;
  if(fabs(vel )==0)velOffset=0;
  if(LeftTarget->y==290)
  { 
    wheel_motor1_target = clampToRange(-(K1 * (pitch) *PI/180 + K2 * (gyroY) *PI/180 +K3 * position + K4 *vel)+velOffset, -speed_limit, speed_limit);
    wheel_motor2_target = clampToRange(-(K1 * (pitch) *PI/180 + K2 * (gyroY) *PI/180 +K3* position + K4 *vel)+velOffset, -speed_limit, speed_limit);
  }
  else{
    wheel_motor1_target = clampToRange(-(K1 * (pitch) *PI/180 + K2 * (gyroY ) *PI/180 +K3 * position + K4 *vel), -speed_limit, speed_limit);
    wheel_motor2_target = clampToRange(-(K1 * (pitch) *PI/180 + K2 * (gyroY ) *PI/180 +K3 * position + K4*vel), -speed_limit, speed_limit);
  }

}


// 将读取的PPM数据进行映射处理 转换为机器人控制参数
void mapPPMToRobotControl()
{
  if(Shake_shoulder == 0)
  {
    // 这里用于控制腿部高度变化控制 采用增量式PID是为了让腿部快速变化时，保持稳定

    float error = (1000 - (filteredPPMValues[0] - 70))/1.5; // 计算误差
    ZeparamremoteValue = -0.30 * error;      // 计算当前的值
    ZeparamremoteValue = constrainValue(ZeparamremoteValue, 0, 150);
    ZeparamremoteValue = ZeparamremoteValue + (ZeparamremoteValue - lastZeparamremoteValue) * 0.12; // 基于误差更新输出（增量控制）
    lastZeparamremoteValue = ZeparamremoteValue;
  }                                                    // 存储当前值作为下一次计算的参考
  // 用于roll轴自稳
  if(EH_rollflag == 1) //自稳开关
  {
    target_roll = target_roll +  roll_kp * (roll - 0);
    target_roll = constrainValue(target_roll, -60, 60);
  }
  else
  {
    target_roll = 0;
  }
  // 用于控制前进后退
  forwardBackward = mapJoystickValuevel(filteredPPMValues[1]);
  // 用于调节重心偏置
  remoteBalanceOffset = mapJoystickValueInt(filteredPPMValues[2]);
  // 用于控制转向
  steering = -0.03 * (-mapJoystickValuesteering(filteredPPMValues[5]) - gyroZ);
  steering = constrainValue(steering, -10, 10);
  roll_EH = sin((target_roll * 3.14) / 180) * (LeftMotionControlParams.robotl) / 2;
  roll_EH = constrainValue(roll_EH, -80, 80);
}

void jump_control()
{
  static unsigned long startMillis = 0; // 用于记录起始时间
  static bool timingStarted = false;    // 标志是否开始计时
  if (jump_flag == 1)   //起跳flag
  {
    if (!timingStarted)  
    {
      // 开始计时
      startMillis = millis();
      timingStarted = true;
    }
    if (millis() - startMillis >= 70)
    { // 判断是否达到100ms
       Am_kp = 1.5;
       jump_vlaue = 0;
    }
    else
    {
       Am_kp = 2;
       jump_vlaue = 200;
    }
  }
  else
  {
    // 重置计时器和标志位
    Am_kp = 0.8;
    timingStarted = false;
    startMillis = 0;
  }
}



// 轮足机器人控制部分程序
// int aaa = 0;
float Shake_shoulder_vakue = 0;
void robot_control(Node *LeftTarget, Node *RightTarget)
{
   

  if(Shake_shoulder == 1)//抖肩
  {
    ZeparamremoteValue = 50;
    Shake_shoulder_vakue = filteredPPMValues[4];
    if(Shake_shoulder_vakue< 1600 && Shake_shoulder_vakue > 1400)Shake_shoulder_vakue = 1500; 
    Shake_shoulder_vakue = (Shake_shoulder_vakue - 1500) / 20;
    LeftTarget->y = leftY + ZeparamremoteValue + EH_rollflag * roll_EH + jump_vlaue - Shake_shoulder_vakue;
    RightTarget->y = rightY + ZeparamremoteValue - EH_rollflag * roll_EH + jump_vlaue + Shake_shoulder_vakue;  
  }
  else
  {
    LeftTarget->y = leftY + ZeparamremoteValue - EH_rollflag * roll_EH + jump_vlaue;
    RightTarget->y = rightY + ZeparamremoteValue + EH_rollflag * roll_EH + jump_vlaue;
    // LeftTarget->y = leftY ;
    // RightTarget->y = rightY ;
  }
    
    LeftTarget->x = leftX;
    RightTarget->x = rightX;
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

// 用于获取电机初始位置函数
void get_origin_pos()
{
  if (origin_pos_flag == 1)
  {
    // 左腿关节电机初始位置
    motorsParam.motorleftsita0_origin = -devicesState[0].pos;//后腿
    motorsParam.motorleftsita1_origin = -devicesState[1].pos;

    // 右腿关节电机初始位置
    motorsParam.motorrightsita0_origin = -devicesState[3].pos;
    motorsParam.motorrightsita1_origin = -devicesState[2].pos;//后腿

    if (motorsParam.motorleftsita0_origin != 0 && motorsParam.motorleftsita1_origin != 0 && motorsParam.motorrightsita0_origin != 0 && motorsParam.motorrightsita1_origin != 0)
      origin_pos_flag = 0;
  }
}

void calVMC_Right(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles)
{
  float j11,j12,j21,j22, theta2, F, torque, yError,  yTarget, xError, xTarget;
  float Kx =1;
  float Dx = 0;
  float Ky = 1;
  float Dy = 0;

  // forwardKinematics(LeftTarget,RightTarget,jointAngles);
  forwardKinematics_right(RightTarget,jointAngles);
   
  float ydot = 0.0;
  float xdot = 0.0;
  nowRight_y =  RightTarget->y;
  yTarget = (140 + ZeparamremoteValue/2);//230
  yError = yTarget - nowRight_y;
  // F =   (48.755 /cos(jointAngles->thetaRight1)) + Ky* yError +Dy*(0-ydot) ; 
  F =   (48.755 /cos(jointAngles->thetaRight1))+yError  ;   

  nowRight_x = RightTarget->x;
  xTarget = 50;
  xError = xTarget - nowRight_x;
  // torque = Kx* xError+Dx*(0-xdot);
  torque =  xError;

  theta2 =   acos((RightTarget->x - (L5 + L4 * cos(jointAngles->betaRight)))/L3);
  j11 = (L1 * sin(RightTarget->polarAngle - theta2) * sin(jointAngles->alphaRight - jointAngles->thetaRight1))/sin(theta2 - jointAngles->thetaRight1);
  j12 = (L1 * cos(RightTarget->polarAngle - theta2) * sin(jointAngles->alphaRight - jointAngles->thetaRight1))/(RightTarget->radius * sin(theta2 - jointAngles->thetaRight1));
  j21 = (L4 * sin(RightTarget->polarAngle - jointAngles->thetaRight1) * sin(theta2 - jointAngles->betaRight))/sin(theta2 - jointAngles->thetaRight1);
  j22 = (L4 * cos(RightTarget->polarAngle - jointAngles->thetaRight1) * sin(theta2 - jointAngles->betaRight))/(RightTarget->radius * sin(theta2 - jointAngles->thetaRight1));

  RightTarget->frontTorque = start  * (- 0.1 * (j21 * F + j22 * torque) / 1000.0f);
  RightTarget->backTorque  = start  * (- 0.1* (j11 * F + j12 * torque) / 1000.0f);


  // Serial.print((- 0.18 * (j21 * F + j22 * torque) / 1000.0f));
  // Serial.print(",");
  // Serial.print(nowRight_y);
  // Serial.print(",");
  // // Serial.print(RightTarget->radius);
  // // Serial.print(",");
  // Serial.print(nowRight_x);
  // Serial.print(",");
  // Serial.print(torque);
  // Serial.print(",");
  // Serial.print(RightTarget->y);
  // Serial.print(",");  
  // Serial.print(RightTarget->x);
  // Serial.print(",");

}



void calVMC_Left(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles)
{
  float j11,j12,j21,j22, theta2, F, torque, yError,  yTarget, xError, xTarget;
  float Kx =1;
  float Dx = 0;
  float Ky = 1;
  float Dy = 0;
  
  // forwardKinematics(LeftTarget,RightTarget,jointAngles);
  forwardKinematics_left(LeftTarget,jointAngles);

  float ydot = 0.0;
  float xdot = 0.0;
  nowLeft_y =  LeftTarget->y;
  yTarget = (140 + ZeparamremoteValue/2);//140~290
  yError = yTarget - nowLeft_y;
  // F =   (48.755 /cos(jointAngles->thetaLeft1)) + Ky* yError +Dy*(0-ydot) ; 
  F =   (48.755 /cos(jointAngles->thetaLeft1)) +  yError ;   

  nowLeft_x = LeftTarget->x;
  xTarget = 50;
  xError = xTarget - nowLeft_x;
  // torque = Kx* xError+Dx*(0-xdot);
  torque =  xError;

  theta2 =   acos((LeftTarget->x - (L5 + L4 * cos(jointAngles->betaLeft)))/L3);
  j11 = (L1 * sin(LeftTarget->polarAngle - theta2) * sin(jointAngles->alphaLeft  - jointAngles->thetaLeft1))/sin(theta2 - jointAngles->thetaLeft1);
  j12 = (L1 * cos(LeftTarget->polarAngle - theta2) * sin(jointAngles->alphaLeft  - jointAngles->thetaLeft1))/(LeftTarget->radius * sin(theta2 - jointAngles->thetaLeft1));
  j21 = (L4 * sin(LeftTarget->polarAngle - jointAngles->thetaLeft1) * sin(theta2 - jointAngles->betaLeft))/sin(theta2 - jointAngles->thetaLeft1);
  j22 = (L4 * cos(LeftTarget->polarAngle - jointAngles->thetaLeft1) * sin(theta2 - jointAngles->betaLeft))/(LeftTarget->radius * sin(theta2 - jointAngles->thetaLeft1));

  LeftTarget->frontTorque = start * ( -0.1* (j21 * F + j22 * torque) / 1000.0f);
  LeftTarget->backTorque  = start * ( -0.1* (j11 * F + j12 * torque) / 1000.0f);

  // // Serial.print(",");
  // Serial.print(devicesState[2].pos/8);//LeftTarget->frontTorque
  // Serial.print(",");
  // // // Serial.print(( -0.2* (j21 * F + j22 * torque) / 1000.0f));
  // // // Serial.print(",");
  // Serial.print(LeftTarget->frontTorque);
  // Serial.print(",");
  // Serial.print(devicesState[3].pos/8);
  // Serial.print(",");
  Serial.print(devicesState[0].pos);//2.67
  Serial.print(",");
  // // Serial.print(devicesState[1].pos);//1.83
  // // Serial.print(",");
  Serial.print(devicesState[2].pos);//2.87
  Serial.print(",");
  // // Serial.print(devicesState[3].pos);//4.03
  // // Serial.print(",");
  Serial.print(RightTarget->y);
  Serial.print(",");  
  Serial.println(RightTarget->x);

}
