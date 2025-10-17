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
#define MOTOR_RF_VERTICAL_90DEG   1.89f  // Right Front (右前)
#define MOTOR_RR_VERTICAL_90DEG   3.69f  // Right Rear (右后)
#define MOTOR_LR_VERTICAL_90DEG   1.22f  // Left Rear (左后)
#define MOTOR_LF_VERTICAL_90DEG   1.67f  // Left Front (左前)
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
JointAngles jointAngles;//关节角度
Node LeftTarget; // 左腿目标坐标
Node RightTarget; // 右腿目标坐标
Node LeftTargetfk;
Node RightTargetfk;
float Am_kp = 1;  //关节电机Kp
uint32_t sendNum; // for test send speed
uint32_t recNum;



#define SEND_INTERVAL 1 // 限制CAN发送频率，单位为毫秒
unsigned long lastSendTime = 0; // 记录上次发送时间

MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050

// 电机控制计算函数
void calculateMotorTargets(JointAngles * jointAngles, MIT * LeftFronMITCtrlParam, MIT * LeftRearMITCtrlParam, MIT * RightFronMITCtrlParam, MIT * RightRearMITCtrlParam)
{
  float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;

  motorRightRear = POS_OF_RR0DRGEE - (jointAngles->alphaRight * JOINT_GEAR_RATIO); // 打印的第3个数据      
  motorRightFront = POS_OF_RF0DRGEE - (jointAngles->betaRight * JOINT_GEAR_RATIO); // 打印的第4个数据 
  motorLeftRear = POS_OF_LR0DRGEE - (jointAngles->alphaLeft * JOINT_GEAR_RATIO ); // 打印的第1个数据  -1.8
  motorLeftFront = POS_OF_LF0DRGEE - (jointAngles->betaLeft * JOINT_GEAR_RATIO ); // 打印的第2个数据  -1.5

  // 左腿关节电机1 控制参数  pos正  MIT协议为左后腿
  LeftFronMITCtrlParam->pos = 1 * motorLeftFront; 
  LeftFronMITCtrlParam->vel = 0;
  LeftFronMITCtrlParam->kp =  Am_kp ;
  LeftFronMITCtrlParam->kd = 0;
  LeftFronMITCtrlParam->tor = 0;

  // 左腿关节电机2 控制参数 pos负 控制升高降低  MIT协议为左前腿
  LeftRearMITCtrlParam->pos = 1 * motorLeftRear;
  LeftRearMITCtrlParam->vel = 0;
  LeftRearMITCtrlParam->kp =  Am_kp  ;  
  LeftRearMITCtrlParam->kd = 0;
  LeftRearMITCtrlParam->tor = 0;

  // 右腿关节电机1 控制参数
  RightFronMITCtrlParam->pos = 1 * motorRightRear;
  RightFronMITCtrlParam->vel = 0;
  RightFronMITCtrlParam->kp =  Am_kp-0.18;
  RightFronMITCtrlParam->kd = 0;
  RightFronMITCtrlParam->tor = 0;

  // 右腿关节电机2 控制参数
  RightRearMITCtrlParam->pos = 1 * motorRightFront;
  RightRearMITCtrlParam->vel = 0;
  RightRearMITCtrlParam->kp =  Am_kp-0.18;
  RightRearMITCtrlParam->kd = 0;
  RightRearMITCtrlParam->tor = 0;
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
      sendMITCommand(0x01, LeftFronMITCtrlParam);
      sendMITCommand(0x02, LeftRearMITCtrlParam);
      sendMITCommand(0x03, RightFronMITCtrlParam);
      sendMITCommand(0x04, RightRearMITCtrlParam);
      lastSendTime = currentTime; // 更新最后发送时间
    }
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
  delay(2000);
}

void loop() {
  serialReceiveUserCommand();                                                                                                       //串口调参
  heightFit(ZeparamremoteValue);                                                                                                    //PID线性拟合           
  wheel_control();                                                                                                                  // 轮子 霍尔电机控制函数
  jointControl(LeftFronMITCtrlParam, LeftRearMITCtrlParam, RightFronMITCtrlParam, RightRearMITCtrlParam);                           //关节电机控制
  remote_switch();                                                                                                                  //遥控器开关控制                                          
  inverseKinematics(&LeftTarget, &RightTarget, &jointAngles);                                                                       //运动学逆解
  calculateMotorTargets(&jointAngles, &LeftFronMITCtrlParam, &LeftRearMITCtrlParam, &RightFronMITCtrlParam, &RightRearMITCtrlParam);//电机控制计算
  legControl(&LeftTarget, &RightTarget);                                                                                            //机器人xy坐标控制
  sendMotorTargets(up_start * wheelTorqueTarget1, up_start * wheelTorqueTarget2);                                                   // 发送控制轮毂电机的目标值
  storeFilteredPPMData();                                                                                                           // 对ppm数据进行滤波处理 避免数据大幅度跳动 
  mapPPMToRobotControl();                                                                                                           // 处理遥控器数据 将其映射为机器人行为控制

}

// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
  while (true)
  {
    mpu6050.update();
    roll = mpu6050.getAngleX();
    pitch = mpu6050.getAngleY() - remoteBalanceOffset;
    yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroY = mpu6050.getGyroY();
    gyroZ = mpu6050.getGyroZ();
    // 这里是设置一定死区 避免数据的波动
    if (gyroZ > -1 && gyroZ < 1) gyroZ = 0;
    gyroY = lowPassFilter(mpu6050.getGyroY(),gyroY,0.005);
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

