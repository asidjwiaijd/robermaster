/**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       jointmotor_task.c/h
  * @brief      joint motor control task,
  *             关节电机控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-04-2022     HYX              1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#ifndef JOINTMOTOR_TASK_H
#define JOINTMOTOR_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//Y方向运动的KP和KD值
#define KP_Y 100.0f
#define KD_Y 40.0f

//ROLL方向运动的KP和KD值
#define KP_ROLL 20.0f
#define KD_ROLL 5.0f

//任务初始化的延迟时间
#define JOINTMOTOR_TASK_INIT_TIME 360

//控制任务的周期时间，ms
#define JOINTMOTOR_CONTROL_TIME_MS 5

//控制任务的周期时间，s
#define JOINTMOTOR_CONTROL_TIME_S 0.005

//关节电机的零位角度，150°换算为弧度制
#define INIT_ANGLE 2.618

//腿部连杆的长度参数
#define l1 0.1
#define l2 0.16
#define l3 0.32

//控制y方向和Roll方向的摇杆通道号
#define LEG_Y_CHANNEL 1
#define LEG_ROLL_CHANNEL 0

//摇杆位置数据映射到y方向高度和Roll方向角度的比例系数
#define LEG_Y_RC_SEN 0.0002
#define LEG_ROLL_RC_SEN 0.0002

//y方向的中间高度
#define LEY_Y_MID_HEIGHT 0.27

//半边机体重力（6kg×9.8=58.8）
#define G 58.8

//两轮间距
#define D 0.523

//关节电机数据结构体
typedef struct
{
  const joint_motor_measure_t *joint_motor_measure;
	uint32_t joint_motor_id;    //关节电机ID
	fp32 angle;                 //关节电机角度，单位rad
	fp32 torque_set;            //关节电机力矩设定值
} joint_motor_t;

//腿部运动数据结构体
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //获取遥控器指针
  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  const fp32 *chassis_INS_angle_speed;       //获取陀螺仪解算出的旋转角速度指针
	const fp32 *accel_fliter;                  //获取滤波后的加速度计数据指针
	joint_motor_t joint_motor[4];              //关节电机数据
	
	fp32 y;                                    //车体高度
	fp32 yL;                                   //左边腿部高度
	fp32 yR;                                   //右边腿部高度
	fp32 delta_y;                              //车体高度设定值与反馈值的差值
	fp32 delta_roll;                           //车体roll角度设定值与反馈值的差值
	fp32 y_set;                                //车体高度设定值
	fp32 roll_set;                             //ROLL轴角度设定值
	fp32 vy;                                   //车体y方向的速度
	fp32 vyL;                                  //左边腿部的位移微分值，用来计算末端执行力转换成关节电机力矩
	fp32 vyR;                                  //右边腿部的位移微分值，用来计算末端执行力转换成关节电机力矩
	fp32 FL;                                   //左边腿部的末端执行力
	fp32 FR;                                   //右边腿部的末端执行力
	fp32 y_accel;                              //滤波后的y方向加速度
	
	fp32 chassis_yaw;                          //底盘陀螺仪反馈的当前yaw角度
  fp32 chassis_pitch;                        //底盘陀螺仪反馈的当前pitch角度
  fp32 chassis_roll;                         //底盘陀螺仪反馈的当前roll角度
  fp32 chassis_yaw_speed;                    //底盘陀螺仪反馈的当前yaw角速度
	fp32 chassis_pitch_speed;                  //底盘陀螺仪反馈的当前pitch角速度
	fp32 chassis_roll_speed;                   //底盘陀螺仪反馈的当前roll角速度
}leg_move_t;

extern void jointmotor_task(void const *argument);

#endif