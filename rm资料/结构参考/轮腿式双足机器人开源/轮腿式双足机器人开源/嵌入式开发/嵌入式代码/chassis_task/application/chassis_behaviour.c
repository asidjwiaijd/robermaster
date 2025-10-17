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

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_down_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/*底盘行为模式变量*/
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

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
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    /*遥控器设置底盘行为模式*/
		
		/*如果遥控器的S2开关处于中位，则是平衡模式*/
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_BALANCE;
    }
		/*如果遥控器的S2开关处于低位，则是无力模式*/
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
		/*如果遥控器的S2开关处于高位，则是遥控模式*/
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_REMOTE;
    }

    /*根据行为模式选择一个底盘控制模式*/
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_DOWN_MODE; 
    }
    else if (chassis_behaviour_mode == CHASSIS_BALANCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_BALANCE_MODE; 
    }
    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_REMOTE_MODE;
    }
}

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
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_BALANCE)
    {
			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
    {
			chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          底盘DOWN掉的行为状态机下，目标速度和目标转角均为0
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_down_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}

/**
  * @brief          底盘平衡的行为状态机下，目标速度和目标转角均为0
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}

/**
  * @brief          底盘遥控的行为状态机下，目标速度是摇杆3通道的映射，目标转角是摇杆2通道的映射
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角，范围-PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, chassis_move_rc_to_vector);
		
    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}