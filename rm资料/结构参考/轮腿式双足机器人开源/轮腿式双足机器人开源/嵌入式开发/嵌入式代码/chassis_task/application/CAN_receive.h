/**
  ****************************(C) COPYRIGHT 2021 SuperPower****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function，CAN1 receives chassis motor and joint motor data,
  *             and CAN send function sends motor parameters to control motor.
  *             这里是CAN中断接收函数，CAN1接收关节电机和底盘电机数据，CAN发送函数发送电机参数控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  v1.2.0     Nov-24-2021     HYX             1. add joint motor
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 SuperPower****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

/*定义关节电机的控制模式*/
#define CMD_MOTOR_MODE      0x05
#define CMD_RESET_MODE      0x06
#define CMD_ZERO_POSITION   0x07

/*定义关节电机的控制参数幅值*/
#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

/* CAN send and receive ID */
typedef enum
{
	/*HT03电机ID*/
	CAN_HT03_M1_ID = 0x01,
	CAN_HT03_M2_ID = 0x02,
	CAN_HT03_M3_ID = 0x03,
	CAN_HT03_M4_ID = 0x04,
	
	/*M3508电机ID*/
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	
	/*CAN发送函数中控制所有M3508电机的ID*/
	CAN_CHASSIS_ALL_ID = 0x200,
} can_msg_id_e;

//3508 motor data
/*3508电机的数据结构体*/
typedef struct
{
    uint16_t ecd;            //编码器码盘值，范围0~8191
    int16_t speed_rpm;       //电机转速，单位RPM
    int16_t given_current;   //转矩电流
    uint8_t temperate;       //电机温度
    int16_t last_ecd;        //上一次的码盘值
} chassis_motor_measure_t;

//HT-03 motor data
/*HT03电机的数据结构体*/
typedef struct
{
	  uint32_t motorid;        //电机ID值
    float position;          //转子位置，单位rad
    float velocity;          //转子速度，单位rad/s
    float current;           //电流值
} joint_motor_measure_t;

/**
  * @brief          send control current of chassis motor (0x201,0x202)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送底盘电机控制电流(0x201,0x202)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2);

/**
  * @brief          send control parameters of joint motor (0x01, 0x02, 0x03, 0x04)
  * @param[in]      f_p: position command, range [-95.5,95.5] rad
  * @param[in]      f_v: velocity command, range [-45,45] rad/s
  * @param[in]      f_kp: kp parameter, range [0,500] N.m/rad
  * @param[in]      f_kd: kd parameter, range [0,5] N.m/rad/s
  * @param[in]      f_t:  torque command,range [-18,18] N.m
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          发送关节电机控制参数(0x01,0x02,0x03,0x04)
  * @param[in]      f_p: 目标位置，范围 [-95.5,95.5] rad
  * @param[in]      f_v: 目标转速，范围 [-45,45] rad/s
  * @param[in]      f_kp: kp参数， 范围 [0，500] N.m/rad
  * @param[in]      f_kd: kd参数,  范围 [0,5] N.m/rad/s
  * @param[in]      f_t: 目标力矩, 范围 [-18,18] N.m
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
extern void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id);

/**
  * @brief          send mode data to set the control mode of joint motor 
  * @param[in]      cmd: mode command, CMD_MOTOR_MODE(0x05), CMD_RESET_MODE(0x06), CMD_ZERO_POSITION(0x07)
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          发送模式数据去设置关节电机的控制模式
  * @param[in]      cmd：模式指令，电机模式(0x05)、失效模式(0x06)、零位设置模式(0x07)
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
extern void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         chassis motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         底盘电机数据指针
  */
extern const chassis_motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          return the joint motor HT-03 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         joint motor data point
  */
/**
  * @brief          返回关节电机HT-03电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         关节电机数据指针
  */
extern const joint_motor_measure_t *get_joint_motor_measure_point(uint8_t i);

#endif