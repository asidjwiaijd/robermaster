#include <Arduino.h>
#include "CAN_comm.h"
#include "bipedal_data.h"
#include "config.h"
#include "Kinematics/Kinematics.h"

#define JOINT_GEAR_RATIO     8       // 关节电机减速比为8:1
#define MOTOR_RF_VERTICAL_90DEG   1.00f  // Right Front (右前)
#define MOTOR_RR_VERTICAL_90DEG   3.70f  // Right Rear (右后)
#define MOTOR_LR_VERTICAL_90DEG   2.64f  // Left Rear (左后)
#define MOTOR_LF_VERTICAL_90DEG   3.01f  // Left Front (左前)
#define POS_OF_RR0DRGEE  MOTOR_RR_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //右后0°位置
#define POS_OF_RF0DRGEE  MOTOR_RF_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //右前0°位置
#define POS_OF_LR0DRGEE  MOTOR_LR_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //左后0°位置
#define POS_OF_LF0DRGEE  MOTOR_LF_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //左前0°位置

//正弦波参数
uint8_t offset = 240;
uint8_t amp = 40;
uint8_t period = 128;

motorsparam motorsParam;
MIT MITCtrlParam;
uint32_t prev_ts;
// 左腿关节电机MIT控制
MIT LeftFronMITCtrlParam;
MIT LeftRearMITCtrlParam;

// 右腿关节电机MIT控制
MIT RightFronMITCtrlParam;
MIT RightRearMITCtrlParam;

JointAngles jointAngles;//关节角度
Node LeftTarget; // 左腿目标坐标
Node RightTarget; // 右腿目标坐标
Node LeftTargetfk;
Node RightTargetfk;


float Am_kp = 0.8;  //关节电机Kp
MIT devicesState[4];
uint32_t sendNum; // for test send speed
uint32_t recNum;




#define SEND_INTERVAL 1 // 限制CAN发送频率，单位为毫秒

unsigned long lastSendTime = 0; // 记录上次发送时间

// 电机控制计算函数
void calculateMotorTargets(JointAngles * jointAngles, MIT * LeftFronMITCtrlParam, MIT * LeftRearMITCtrlParam, MIT * RightFronMITCtrlParam, MIT * RightRearMITCtrlParam)
{
  float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;

  motorRightRear = POS_OF_RR0DRGEE - (jointAngles->alphaRight * JOINT_GEAR_RATIO);     
  motorRightFront = POS_OF_RF0DRGEE - (jointAngles->betaRight * JOINT_GEAR_RATIO); 
  motorLeftRear = POS_OF_LR0DRGEE - (jointAngles->alphaLeft * JOINT_GEAR_RATIO); 
  motorLeftFront = POS_OF_LF0DRGEE - (jointAngles->betaLeft * JOINT_GEAR_RATIO); 
  
  MITCtrlParam.pos = 0;
  MITCtrlParam.vel = 0;
  MITCtrlParam.kp = 0;
  MITCtrlParam.kd = 0;
  MITCtrlParam.tor = 0;

  // 左腿关节电机1 控制参数  pos正  MIT协议为左后腿
  LeftFronMITCtrlParam->pos = 1 * motorLeftFront; // 28 - motorLeftRear motorLeftFront
  LeftFronMITCtrlParam->vel = 0;
  LeftFronMITCtrlParam->kp = 1 * Am_kp;
  LeftFronMITCtrlParam->kd = 0;
  LeftFronMITCtrlParam->tor = 0;

  // 左腿关节电机2 控制参数 pos负 控制升高降低  MIT协议为左前腿
  LeftRearMITCtrlParam->pos = 1 * motorLeftRear; //-23 + motorLeftFront
  LeftRearMITCtrlParam->vel = 0;
  LeftRearMITCtrlParam->kp = 1 * Am_kp;
  LeftRearMITCtrlParam->kd = 0;
  LeftRearMITCtrlParam->tor = 0;

  // 右腿关节电机1 控制参数
  RightFronMITCtrlParam->pos = 1 * motorRightRear;
  RightFronMITCtrlParam->vel = 0;
  RightFronMITCtrlParam->kp = 1 * Am_kp;
  RightFronMITCtrlParam->kd = 0;
  RightFronMITCtrlParam->tor = 0;

  // 右腿关节电机2 控制参数
  RightRearMITCtrlParam->pos = 1 * motorRightFront;
  RightRearMITCtrlParam->vel = 0;
  RightRearMITCtrlParam->kp = 1 * Am_kp;
  RightRearMITCtrlParam->kd = 0;
  RightRearMITCtrlParam->tor = 0;
}


void jointControl()
{
  unsigned long currentTime = millis(); // 获取当前时间

  recCANMessage();  // CAN接收函数
  uint32_t current_ts = micros();
  if (current_ts - prev_ts >= 1000) // 1s 1000000
  {                                  // 1ms
   
    prev_ts = current_ts;
    sendNum = 0;
    recNum = 0;
  }

      // 检查时间差是否大于或等于发送间隔  设置发送时间 控制CAN的发送频率
    if (currentTime - lastSendTime >= SEND_INTERVAL)
    {
      // 发送命令
      sendMITCommand(0x01, LeftFronMITCtrlParam);
      sendMITCommand(0x02, LeftRearMITCtrlParam);
      sendMITCommand(0x03, RightFronMITCtrlParam);
      sendMITCommand(0x04, RightRearMITCtrlParam);
      lastSendTime = currentTime; // 更新最后发送时间
    }
}




String serialReceiveUserCommand()  //串口调参函数   格式：   数值,数值
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
        LeftTarget.y = firstParam.toDouble();
        Serial.print("LeftTarget.y:");
        Serial.println(LeftTarget.y);

        String secondParamStr = command.substring(commaPosition + 1, newlinePosition);
        int secondCommaPosition = secondParamStr.indexOf(',');
 
        RightTarget.y = secondParamStr.toDouble();
        Serial.print("RightTarget.y:");
        Serial.println(RightTarget.y);
      }
      received_chars = "";
    }
  }
  return command;
}


void setup() {
  Serial2.begin(20000000, SERIAL_8N1, 18, 17);
  Serial.begin(115200);
  CANInit(); // 初始化CAN
  LeftTarget.x = 60;//初始值
  LeftTarget.y = 300;
  RightTarget.x = 60;
  RightTarget.y = 380; 
  delay(2000);
}

void loop() {
  serialReceiveUserCommand();//设置y轴坐标

  for (int i = -10; i < 118; i++)//正弦波
  {
    RightTarget.x = i;
    RightTarget.y = offset + amp * sin((2*PI/period)*RightTarget.x);
    delay(30);
    inverseKinematics(&LeftTarget, &RightTarget, &jointAngles);
    calculateMotorTargets(&jointAngles, &LeftFronMITCtrlParam, &LeftRearMITCtrlParam, &RightFronMITCtrlParam, &RightRearMITCtrlParam);
    jointControl();

  }

  for (int i = 117; i >= -10; i--)
  {
    RightTarget.x = i;
    RightTarget.y = offset + amp * sin((2*PI/period)*RightTarget.x);
    delay(30);
    inverseKinematics(&LeftTarget, &RightTarget, &jointAngles);
    calculateMotorTargets(&jointAngles, &LeftFronMITCtrlParam, &LeftRearMITCtrlParam, &RightFronMITCtrlParam, &RightRearMITCtrlParam);
    jointControl();
  }
}

