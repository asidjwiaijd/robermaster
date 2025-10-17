/**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.1.0     Apr-01-2022     HYX             1. done,apply to two-wheeled Robot
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//任务开始后空闲一段时间
#define CHASSIS_TASK_INIT_TIME 1000

//the channel num of controlling vertial movement 
//控制前后运动的遥控器通道号码
#define CHASSIS_X_CHANNEL 3

//the channel num of controlling rotation movement
//控制YAW轴旋转运动的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode
//选择底盘状态的开关通道号
#define CHASSIS_MODE_CHANNEL 0

//the channel of choosing leg-movement mode
//选择腿部运动状态的开关通道号
#define CHASSIS_LEG_CHANNEL 1

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0015f

//rocker value change to rotation target angle
//遥控器的yaw遥杆（max 660）转化成车体旋转目标角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000003f

//遥杆值一阶低通滤波时的滤波参数值
#define CHASSIS_ACCEL_X_NUM 0.1666666667f

//rocker value deadline
//摇杆死区幅值
#define CHASSIS_RC_DEADLINE 10

//radius of wheel
//车轮半径
#define WHEEL_RADIUS 0.07425

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//chassis 3508  motor max control current
//底盘3508电机最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//chassi forward, back key
//底盘前后运动键盘控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S

//reducation of 3508 motor
//m3508电机的减速比
#define M3508_MOTOR_REDUCATION 19.2032f

//m3508 rpm change to chassis speed
//m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00040490766f

//m3508 rpm change to motor angular velocity
//m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.0054533f

//m3508 current change to motor torque
//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
//c=20/16384*0.3，0.3为转矩常数(N.m/A)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//single chassis motor max torque
//单个底盘电机最大力矩
#define MAX_WHEEL_TORQUE 5.5f

//chassis forward or back max speed
//底盘运动过程最大前后运动速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1.0f

//LQR feedback parameter
//LQR反馈增益系数
#define LQR_K1 -0.0224f
#define LQR_K2 -2.2554f
#define LQR_K3 -10.1436f
#define LQR_K4 -1.5177f
#define LQR_K15 2.2361f
#define LQR_K16 0.6863f
#define LQR_K25 -LQR_K15
#define LQR_K26 -LQR_K16

//底盘控制模式
typedef enum
{
  CHASSIS_REMOTE_MODE,  //遥控模式
  CHASSIS_BALANCE_MODE, //平衡模式
  CHASSIS_DOWN_MODE,    //DOWN模式
} chassis_mode_e;

//底盘电机数据结构体
typedef struct
{
  const chassis_motor_measure_t *chassis_motor_measure;
  fp32 speed;           //电机轮轴位移速度
	fp32 omg;             //电机输出轴旋转速度
	fp32 torque;          //电机输出力矩
	fp32 torque_set;      //电机输出力矩设定值
  int16_t give_current; //电机控制电流设定值
} chassis_motor_t;

//底盘运动数据结构体
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //获取遥控器指针
  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  const fp32 *chassis_INS_angle_speed;       //获取陀螺仪解算出的旋转角速度指针
	chassis_mode_e chassis_mode;               //底盘控制模式状态机
  chassis_mode_e last_chassis_mode;          //底盘上次控制模式状态机
  chassis_motor_t motor_chassis[2];          //底盘电机数据

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值

  fp32 vx;                          //底盘速度，前进为正后退为负，单位m/s
  fp32 omg;                         //底盘合成轮的角速度，单位rad/s
  fp32 vx_set;                      //底盘速度设定值，前进为正后退为负，单位m/s
  fp32 chassis_yaw_set;             //底盘yaw轴角度设定值
  fp32 delta_angle;                 //底盘yaw轴角度设定值与yaw轴角度当前值之差

  fp32 vx_max_speed;                //前进方向最大速度，单位m/s
  fp32 vx_min_speed;                //后退方向最大速度，单位m/s
  fp32 chassis_yaw;                 //底盘陀螺仪反馈的当前yaw角度
  fp32 chassis_pitch;               //底盘陀螺仪反馈的当前pitch角度
  fp32 chassis_roll;                //底盘陀螺仪反馈的当前roll角度
  fp32 chassis_yaw_speed;           //底盘陀螺仪反馈的当前yaw角速度
	fp32 chassis_pitch_speed;         //底盘陀螺仪反馈的当前pitch角速度
	fp32 chassis_roll_speed;          //底盘陀螺仪反馈的当前roll角速度
} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向速度设定值
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector);

#endif