  /**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *  v2.1.0     Feb-14-2022     HYX             1. delete some behaviours
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"

/*底盘行为模式*/
typedef enum
{
  CHASSIS_ZERO_FORCE,  //底盘无力模式
  CHASSIS_BALANCE,     //底盘平衡模式
  CHASSIS_REMOTE,      //底盘遥控模式
} chassis_behaviour_e;

/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          set control target-point. according to different control mode, usually call different control function.
  * @param[out]     vx_set, usually controls vertical movement.
  * @param[out]     angle_set, usually controls rotation movement.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          设置控制目标量.根据不同底盘控制模式，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     angle_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif