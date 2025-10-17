#include <Arduino.h>
#include "CAN_comm.h"
#include "bipedal_data.h"
#include "config.h"
#include "Kinematics/Kinematics.h"
#include <Wire.h>
#include "MPU6050.h"
#include "Robot/robot.h"
#include "Motor.h"


#define JOINT_GEAR_RATIO     8       // 关节电机减速比为8:1
#define MOTOR_RF_VERTICAL_90DEG   2.72f  // Right Front (右前)
#define MOTOR_RR_VERTICAL_90DEG  2.79f  // Right Rear (右后)
#define MOTOR_LR_VERTICAL_90DEG   4.03f  // Left Rear (左后)
#define MOTOR_LF_VERTICAL_90DEG   1.83f  // Left Front (左前)
#define POS_OF_RR0DRGEE  MOTOR_RR_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //右后0°位置
#define POS_OF_RF0DRGEE  MOTOR_RF_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //右前0°位置
#define POS_OF_LR0DRGEE  MOTOR_LR_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //左后0°位置
#define POS_OF_LF0DRGEE  MOTOR_LF_VERTICAL_90DEG + (PI/2) * JOINT_GEAR_RATIO  //左前0°位置



uint32_t prev_ts;
// 左腿关节电机MIT控制
MIT LeftFronMITCtrlParam;
MIT LeftRearMITCtrlParam;
// 右腿关节电机MIT控制
MIT RightFronMITCtrlParam;
MIT RightRearMITCtrlParam;
JointAngles ikjointAngles;//关节角度
JointAngles fkjointAngles;//关节角度
motorsparam motorsParam;
Node LeftTargetik;
Node RightTargetik;
Node LeftTargetfk;
Node RightTargetfk;


float Am_kp = 1;  //关节电机Kp
uint32_t sendNum; // for test send speed
uint32_t recNum;

#define SEND_INTERVAL 1 // 限制CAN发送频率，单位为毫秒
unsigned long lastSendTime = 0; // 记录上次发送时间

MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050



// 电机控制计算函数

// void calculateMotorTargets(JointAngles * jointAngles, MIT * LeftFronMITCtrlParam, MIT * LeftRearMITCtrlParam, MIT * RightFronMITCtrlParam, MIT * RightRearMITCtrlParam)
// {
//   float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;

//   motorRightRear = POS_OF_RR0DRGEE - (jointAngles->alphaRight * JOINT_GEAR_RATIO); // 打印的第3个数据      
//   motorRightFront = POS_OF_RF0DRGEE - (jointAngles->betaRight * JOINT_GEAR_RATIO); // 打印的第4个数据 
//   motorLeftRear = POS_OF_LR0DRGEE - (jointAngles->alphaLeft * JOINT_GEAR_RATIO ); // 打印的第1个数据  -1.8
//   motorLeftFront = POS_OF_LF0DRGEE - (jointAngles->betaLeft * JOINT_GEAR_RATIO ); // 打印的第2个数据  -1.5

//   // 左腿关节电机1 控制参数  pos正  MIT协议为左后腿
//   LeftFronMITCtrlParam->pos = 1 * motorLeftFront; 
//   LeftFronMITCtrlParam->vel = 0;
//   LeftFronMITCtrlParam->kp =  Am_kp ;
//   LeftFronMITCtrlParam->kd = 0;
//   LeftFronMITCtrlParam->tor = 0;

//   // 左腿关节电机2 控制参数 pos负 控制升高降低  MIT协议为左前腿
//   LeftRearMITCtrlParam->pos = 1 * motorLeftRear;
//   LeftRearMITCtrlParam->vel = 0;
//   LeftRearMITCtrlParam->kp =  Am_kp  ;  
//   LeftRearMITCtrlParam->kd = 0;
//   LeftRearMITCtrlParam->tor = 0;

//   // 右腿关节电机1 控制参数
//   RightFronMITCtrlParam->pos = 1 * motorRightFront;
//   RightFronMITCtrlParam->vel = 0;
//   RightFronMITCtrlParam->kp =  Am_kp;
//   RightFronMITCtrlParam->kd = 0;
//   RightFronMITCtrlParam->tor = 0;

//   // 右腿关节电机2 控制参数
//   RightRearMITCtrlParam->pos = 1 * motorRightRear;
//   RightRearMITCtrlParam->vel = 0;
//   RightRearMITCtrlParam->kp =  Am_kp;
//   RightRearMITCtrlParam->kd = 0;
//   RightRearMITCtrlParam->tor = 0;
// }


// 电机控制计算函数 VMC
void calculateMotorTargets(Node * LeftTarget, Node * RightTarget, MIT * LeftFronMITCtrlParam, MIT * LeftRearMITCtrlParam, MIT * RightFronMITCtrlParam, MIT * RightRearMITCtrlParam)
{
  // 左腿关节电机1 控制参数  pos正  MIT协议为左后腿
  LeftFronMITCtrlParam->pos = 0;
  LeftFronMITCtrlParam->vel = 0;
  LeftFronMITCtrlParam->kp =  0 ;
  LeftFronMITCtrlParam->kd = 0;
  LeftFronMITCtrlParam->tor = LeftTarget->frontTorque;

  // 左腿关节电机2 控制参数 pos负 控制升高降低  MIT协议为左前腿
  LeftRearMITCtrlParam->pos = 0;
  LeftRearMITCtrlParam->vel = 0;
  LeftRearMITCtrlParam->kp = 0  ;  
  LeftRearMITCtrlParam->kd = 0;
  LeftRearMITCtrlParam->tor= LeftTarget->backTorque;

  // 右腿关节电机1 控制参数
  RightFronMITCtrlParam->pos = 0;
  RightFronMITCtrlParam->vel = 0;
  RightFronMITCtrlParam->kp = 0;
  RightFronMITCtrlParam->kd = 0;
  RightFronMITCtrlParam->tor= RightTarget->frontTorque;

  // 右腿关节电机2 控制参数
  RightRearMITCtrlParam->pos = 0;
  RightRearMITCtrlParam->vel= 0;
  RightRearMITCtrlParam->kp  = 0;
  RightRearMITCtrlParam->kd = 0;
  RightRearMITCtrlParam->tor= RightTarget->backTorque;
}

void jointControl( MIT  LeftFronMITCtrlParam, MIT  LeftRearMITCtrlParam, MIT  RightFronMITCtrlParam, MIT  RightRearMITCtrlParam)
{
  unsigned long currentTime = millis(); // 获取当前时间

  recCANMessage();  // CAN接收函数
  uint32_t current_ts = micros();
  if (current_ts - prev_ts >= 1000) // 1s 1000000
  {                                  // 1ms
   
    prev_ts = current_ts;
    sendNum = 0;
    recNum = 0;

    //打印关节电机电角度 
    // Serial.printf("%.2f,%.2f,%.2f,%.2f\n", devicesState[0].pos, devicesState[1].pos, devicesState[2].pos,devicesState[3].pos);
  }

      // 检查时间差是否大于或等于发送间隔  设置发送时间 控制CAN的发送频率
    if (currentTime - lastSendTime >= SEND_INTERVAL)
    {
      // 发送命令
      sendMITCommand(0x02, LeftFronMITCtrlParam);
      sendMITCommand(0x04, LeftRearMITCtrlParam);
      sendMITCommand(0x03, RightFronMITCtrlParam);
      sendMITCommand(0x01, RightRearMITCtrlParam);
      lastSendTime = currentTime; // 更新最后发送时间
    }
}
void enable(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFC;
    // Serial.printf("enable ID %hhu\n",nodeID);
    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}


void IMUTask(void *pvParameters);
void Open_thread_function();//启动线程



void setup() {

  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  Open_thread_function();//启动线程
  mpu6050.begin(); //初始化MPU陀螺仪
  CANInit(); // 初始化CAN
  ppm_init(); //遥控器读取中断初始化
  motorInit();
  // enable(0x01);
  // delay(100);
  // enable(0x02);
  // delay(100);
  // enable(0x03);
  // delay(100);
  // enable(0x04);
  delay(2000);
}

void loop() {
  // PIDValues pid = interpolatePID(ZeparamremoteValue);                                     //PID调参
  wheel_control(&LeftTargetik, &RightTargetik);                                              // 轮子 霍尔电机控制函数 
  jointControl(LeftFronMITCtrlParam, LeftRearMITCtrlParam, RightFronMITCtrlParam, RightRearMITCtrlParam);                                                
  remote_switch(); 
  // jump_control();                           
  // inverseKinematics(&LeftTargetik, &RightTargetik, &ikjointAngles);
  // calculateMotorTargets(&ikjointAngles, &LeftFronMITCtrlParam, &LeftRearMITCtrlParam, &RightFronMITCtrlParam, &RightRearMITCtrlParam);
  calVMC_Right(&LeftTargetfk, &RightTargetfk, &fkjointAngles);
  calVMC_Left (&LeftTargetfk, &RightTargetfk, &fkjointAngles);
  calculateMotorTargets(&LeftTargetfk, &RightTargetfk, &LeftFronMITCtrlParam, &LeftRearMITCtrlParam, &RightFronMITCtrlParam, &RightRearMITCtrlParam); //vmc
  robot_control(&LeftTargetik, &RightTargetik); 
  sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target); // 发送控制轮毂电机的目标值
  storeFilteredPPMData();                                       // 对ppm数据进行滤波处理 避免数据大幅度跳动 
  mapPPMToRobotControl();                                       // 处理遥控器数据 将其映射为机器人行为控制  
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
  while (true)
  {
    mpu6050.update();
    roll = mpu6050.getAngleX();
    pitch = -(mpu6050.getAngleY()- remoteBalanceOffset+2);
    yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroZ = mpu6050.getGyroZ();
    // 这里是设置一定死区 避免数据的波动
    if (gyroZ > -1 && gyroZ < 1) gyroZ = 0;
    gyroY = lowPassFilter(-mpu6050.getGyroY(),gyroY,0.005);

    }
}

//启动线程
void Open_thread_function() 
{
    // 陀螺仪读取任务进程
    xTaskCreatePinnedToCore(
        IMUTask,   // 任务函数
        "IMUTask", // 任务名称
        2048,      // 堆栈大小
        NULL,      // 传递的参数
        1,         // 任务优先级
        NULL,      // 任务句柄
        1          // 运行在核心 0
    );
}

