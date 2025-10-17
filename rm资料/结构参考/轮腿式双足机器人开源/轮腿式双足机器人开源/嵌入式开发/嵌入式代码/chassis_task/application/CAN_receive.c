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

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* get joint motor data */
#define get_joint_motor_measure(ptr, data)                                                  \
    {                                                                                       \
				(ptr)->motorid  = data[0];                                                          \
        (ptr)->position = uint_to_float(((data[1] << 8 ) | (data[2])),P_MIN,P_MAX,16);      \
        (ptr)->velocity = uint_to_float(((data[3] << 4 ) | (data[4]>>4)),V_MIN,V_MAX,12);   \
        (ptr)->current  = uint_to_float(((data[4] << 4 ) | (data[5])),T_MIN,T_MAX,12);      \
    }

/* get chassis motor data */
#define get_chassis_motor_measure(ptr, data)                            \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/*
		joint_motor data,    0:joint motor1 HT-03;  1:joint motor2 HT-03;  2:joint motor3 HT-03;  3:joint motor4 HT-03.
		chassis_motor data,  0:chassis motor1 3508; 1:chassis motor2 3508.
*/
static joint_motor_measure_t    joint_motor[4];
static chassis_motor_measure_t  chassis_motor[2];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data，Unpack the data and store it in the corresponding array
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据,并将数据解包后存入相应数组
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHead; 
    uint8_t Rxdata[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);
	
	  /*根据StdID筛选出M3508电机*/
		if(RxHead.StdId == CAN_3508_M1_ID || RxHead.StdId == CAN_3508_M2_ID)
		{
				static uint8_t j=0;
				j=RxHead.StdId - CAN_3508_M1_ID;
				get_chassis_motor_measure(&chassis_motor[j],Rxdata);
				detect_hook(CHASSIS_MOTOR1_TOE+j);
		}
		
		/*根据StdID筛选出HT03电机，HT03电机的StdID均为0x00*/
		else if(RxHead.StdId == 0)
		{
			/*根据数据首字节（电机ID）进一步筛选HT03电机*/
			if (Rxdata[0] == CAN_HT03_M1_ID || Rxdata[0] == CAN_HT03_M2_ID || Rxdata[0] == CAN_HT03_M3_ID || Rxdata[0] == CAN_HT03_M4_ID )
			{
				static uint8_t i = 0;
				i = Rxdata[0] - CAN_HT03_M1_ID;
				get_joint_motor_measure(&joint_motor[i],Rxdata);
				detect_hook(JOINT_MOTOR1_TOE+i);
			}
	  }
}

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
void CAN_cmd_chassis(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          send parameter data from CAN1 to control joint motor
  * @param[in]      buf: 8 bytes data, including motor control parameter information
  * @param[in]      len: size of buf
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          通过CAN1发送参数数据去控制关节电机
  * @param[in]      buf: 8个字节的数据，包含电机控制参数信息
  * @param[in]      len: buf的长度
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
static void CanTransmit(uint8_t *buf, uint8_t len,uint32_t motor_id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox;
    
    if((buf != NULL) && (len != 0))
    {
			TxHead.StdId    = motor_id;         /* 指定标准标识符，该值在0x01-0x04 */
			TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
			TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
			TxHead.DLC      = len;              /* 指定将要传输的帧长度 */
			
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
			{
			}
    }
}

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
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,   P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,   T_MIN,  T_MAX,  12);
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* 通过CAN接口把buf中的内容发送出去 */
    CanTransmit(buf, sizeof(buf),motor_id);
}

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
void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
			case CMD_MOTOR_MODE:
					buf[7] = 0xFC;
					break;
			
			case CMD_RESET_MODE:
					buf[7] = 0xFD;
			break;
			
			case CMD_ZERO_POSITION:
					buf[7] = 0xFE;
			break;
			
			default:
			return; /* 直接退出函数 */
    }
    CanTransmit(buf,sizeof(buf),motor_id);
}

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
const chassis_motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &chassis_motor[i];
}

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
const joint_motor_measure_t *get_joint_motor_measure_point(uint8_t i)
{
    return &joint_motor[i];
}