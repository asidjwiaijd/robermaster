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

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"

#define rc_deadband_limit(input, output, dealine)    \
{                                                    \
	if ((input) > (dealine) || (input) < -(dealine))   \
	{                                                  \
		(output) = (input);                              \
	}                                                  \
	else                                               \
	{                                                  \
		(output) = 0;                                    \
	}                                                  \
}

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//底盘运动数据
chassis_move_t chassis_move;

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
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
	
    //make sure all chassis motor is online,
    //判断底盘电机是否都在线
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE)||toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);

        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
			
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
			
        //chassis control LQR calculate
        //底盘控制LQR计算
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0,0);
            }
            else
            {
							//send control current
							//发送控制电流
							CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current);
            }
        }
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include first order low-pass filter initialization, remote control data point initialization, 
  *                 3508 chassis motors data point initialization, gyro sensor angle point and angular velocity point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括一阶低通滤波器初始化，遥控器指针初始化，3508底盘电机指针初始化，陀螺仪角度和角速度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

		//parameter of first order low-pass
		//一阶低通滤波器的滤波参数
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
		
    //in beginning， chassis mode is DOWN
    //底盘开机状态为DOWN模式
    chassis_move_init->chassis_mode = CHASSIS_DOWN_MODE;
		
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
		
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
		
		//get gyro sensor euler angular velocity
		//获取陀螺仪角速度指针
		chassis_move_init->chassis_INS_angle_speed = get_gyro_data_point();

    //get chassis motor data point
    //获取底盘电机数据指针
		uint8_t i;
    for (i = 0; i < 2; i++)
    {
			chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    }
		
    //first order low-pass filter  replace ramp function,and init parameters
    //用一阶滤波代替斜波函数生成，初始化相关参数
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);

    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          chassis some measure data updata, such as motor angular velocity, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机旋转速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 2; i++)
    {
			//update motor speed
			//更新电机轮轴位移速度
			chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
			
			//update motor angular velocity
			//更新电机输出轴角速度
			chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
			
			//update motor torque
			//更新电机转矩
			chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->given_current;
    }

    //calculate chassis euler angle
    //计算底盘姿态角度
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
		
		//calculate chassis euler angular velocity
		//计算底盘姿态角速度
		chassis_move_update->chassis_yaw_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_YAW_ADDRESS_OFFSET);
		chassis_move_update->chassis_pitch_speed =  *(chassis_move_update->chassis_INS_angle_speed + INS_PITCH_ADDRESS_OFFSET);
		chassis_move_update->chassis_roll_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_ROLL_ADDRESS_OFFSET);
	  
		//calculate chassis velocity and synthesis angular velocity
		//计算底盘速度和合成轮角速度
		chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed))/2 ;
	  chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg))/2 ;
}

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
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL )
    {
        return;
    }
    
    int16_t vx_channel ;
    fp32 vx_set_channel ;
		
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		
    //将遥杆参数转换为运动参数
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		
    //keyboard set speed set-point
    //键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    
		//stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
}

/**
  * @brief          set chassis control target-point, movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制目标值, 运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
      return;
    }

    fp32 vx_set = 0.0f, angle_set = 0.0f;
		
    //get movement control target-points, 获取运动控制目标值
    chassis_behaviour_control_set(&vx_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_REMOTE_MODE)
    {
			chassis_move_control->delta_angle = 0.0f;
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_BALANCE_MODE)
    {
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
		{
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
      chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
}

/**
  * @brief          control loop, according to control set-point, calculate current to control motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_torque = 0.0f, torque_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;

		/*驱动轮输出力矩=SUM[LQR增益系数*(状态变量目标值-状态变量反馈值)]*/
		/*注意左右轮输出力矩的正负号，以及各状态变量反馈值的正负号*/
	  //左轮输出力矩
	  chassis_move_control_loop->motor_chassis[0].torque_set = - (LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - LQR_K3*chassis_move_control_loop->chassis_pitch  + LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + LQR_K15*chassis_move_control_loop->delta_angle + LQR_K16*chassis_move_control_loop->chassis_yaw_speed);
	  //右轮输出力矩
	  chassis_move_control_loop->motor_chassis[1].torque_set =   (LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - LQR_K3*chassis_move_control_loop->chassis_pitch  + LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + LQR_K25*chassis_move_control_loop->delta_angle + LQR_K26*chassis_move_control_loop->chassis_yaw_speed);

		/*如果是DOWN，则控制电流值设置为0*/
	  if (chassis_move_control_loop->chassis_mode == CHASSIS_DOWN_MODE) 
    {
			for (i = 0; i < 2; i++)
			{
				chassis_move_control_loop->motor_chassis[i].give_current = 0;
		  }
			return;
    }
		
		/*如果电机打滑，打滑会使电机持续加速旋转，直至超过警戒值，此时将电流值设置为0*/
		if(fabs(chassis_move_control_loop->motor_chassis[0].omg)>=30 || fabs(chassis_move_control_loop->motor_chassis[1].omg)>=30)
		{      
			for(i=0;i<2;i++)
			{
				chassis_move_control_loop->motor_chassis[i].give_current = 0;
			}
			return ;
		}
		
		//calculate the max torque in two wheels, limit the max torque
    //计算轮子最大转矩，并限制其最大转矩
    for (i = 0; i < 2; i++)
    {
			temp = fabs(chassis_move_control_loop->motor_chassis[i].torque_set);
			if (max_torque < temp)
			{
				max_torque = temp;
			}
    }

    if (max_torque > MAX_WHEEL_TORQUE)
    {
			torque_rate = MAX_WHEEL_TORQUE / max_torque;
			for (i = 0; i < 2; i++)
			{
				chassis_move_control_loop->motor_chassis[i].torque_set *= torque_rate;
			}
    }

    //赋值电流值
    for (i = 0; i < 2; i++)
    {
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_chassis[i].torque_set / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);
    }
}
