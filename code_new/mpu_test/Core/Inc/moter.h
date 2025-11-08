#ifndef MOTER_H
#define MOTER_H
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// 电机反馈数据结构（原有结构不变，新增电流反馈字段）
typedef struct moter_feedback {
    int16_t speed_left_front;    // 左前电机速度原始值（16位有符号）
    int16_t speed_right_front;   // 右前电机速度原始值
    int16_t speed_left_rear;     // 左后电机速度原始值
    int16_t speed_right_rear;    // 右后电机速度原始值
    int8_t acc_time_left_front;  // 左前加速度时间原始值（1ms单位）
    int8_t acc_time_right_front; // 右前加速度时间原始值
    int8_t acc_time_left_rear;   // 左后加速度时间原始值
    int8_t acc_time_right_rear;  // 右后加速度时间原始值
    int8_t temp_left_front;      // 左前温度原始值（℃）
    int8_t temp_right_front;     // 右前温度原始值
    int8_t temp_left_rear;       // 左后温度原始值
    int8_t temp_right_rear;      // 右后温度原始值
    uint8_t fault_left_front;    // 左前电机故障码
    uint8_t fault_right_front;   // 右前电机故障码
    uint8_t fault_left_rear;     // 左后电机故障码
    uint8_t fault_right_rear;    // 右后电机故障码
    float speed_lf;              // 左前电机速度（rpm）
    float speed_rf;              // 右前电机速度（rpm）
    float speed_lr;              // 左后电机速度（rpm）
    float speed_rr;              // 右后电机速度（rpm）
    float acc_t_lf;              // 左前加速度时间（ms/rpm）
    float acc_t_rf;              // 右前加速度时间
    float acc_t_lr;              // 左后加速度时间
    float acc_t_rr;              // 右后加速度时间
    float temp_lf;               // 左前实际温度（℃）
    float temp_rf;               // 右前实际温度
    float temp_lr;               // 左后实际温度
    float temp_rr;               // 右后实际温度
    int16_t current_left_front;  // 新增：左前电机电流原始值（16位有符号）
    int16_t current_right_front; // 新增：右前电机电流原始值
    int16_t current_left_rear;   // 新增：左后电机电流原始值
    int16_t current_right_rear;  // 新增：右后电机电流原始值
    float current_lf;            // 新增：左前实际电流（A）
    float current_rf;            // 新增：右前实际电流（A）
    float current_lr;            // 新增：左后实际电流（A）
    float current_rr;            // 新增：右后实际电流（A）
} moter_feedback;

// 电机控制指令结构（新增电流给定、开环电压给定字段）
typedef struct moter_command {
    uint8_t id_lf;               // 左前电机ID（1=ID1，2=ID2）
    uint8_t id_rf;               // 右前电机ID
    uint8_t id_lr;               // 左后电机ID
    uint8_t id_rr;               // 右后电机ID
    uint8_t mode_lf;             // 左前电机模式（0=开环，1=电流环，2=速度环）
    uint8_t mode_rf;             // 右前电机模式
    uint8_t mode_lr;             // 左后电机模式
    uint8_t mode_rr;             // 右后电机模式
    float speed_left_front_set;  // 左前速度设定值（rpm，范围-380~380）
    float speed_right_front_set; // 右前速度设定值
    float speed_left_rear_set;   // 左后速度设定值
    float speed_right_rear_set;  // 右后速度设定值
    float acc_time_left_front_set;// 左前加速度时间设定（ms/rpm，0默认1ms）
    float acc_time_right_front_set;// 右前加速度时间设定
    float acc_time_left_rear_set; // 左后加速度时间设定
    float acc_time_right_rear_set;// 右后加速度时间设定
    uint8_t brake_lf;            // 左前刹车控制（0xFF=刹车，其他=不刹车）
    uint8_t brake_rf;            // 右前刹车控制
    uint8_t brake_lr;            // 左后刹车控制
    uint8_t brake_rr;            // 右后刹车控制
    float current_left_front_set;// 新增：左前电流设定值（A，范围-4~4）
    float current_right_front_set;// 新增：右前电流设定值
    float current_left_rear_set; // 新增：左后电流设定值
    float current_right_rear_set;// 新增：右后电流设定值
    float voltage_left_front_set;// 新增：左前开环电压设定值（V，范围0~25.2）
    float voltage_right_front_set;// 新增：右前开环电压设定值
    float voltage_left_rear_set; // 新增：左后开环电压设定值
    float voltage_right_rear_set;// 新增：右后开环电压设定值
} moter_command;

// 原有函数声明
void CRC_8(char *data, uint8_t len, uint8_t *crc);
HAL_StatusTypeDef Motor_SendCommand(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb);
HAL_StatusTypeDef Motor_ReceiveFeedback(UART_HandleTypeDef *huart, moter_feedback *fb, uint8_t motor_id);
HAL_StatusTypeDef Motor_SetMode(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode, moter_feedback *fb);
HAL_StatusTypeDef Motor_GetVersion(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t *version_info, moter_feedback *fb);
void Motor_ParseFeedback(uint8_t *rx_buf, moter_feedback *fb, uint8_t motor_id);
HAL_StatusTypeDef Motor_Init_All(moter_feedback *fb);

// 新增函数声明：电流环控制
HAL_StatusTypeDef Motor_SendCurrentCmd(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb);
// 新增函数声明：开环控制
HAL_StatusTypeDef Motor_SendOpenLoopCmd(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb);
// 新增函数声明：故障码解析
void Motor_ParseFaultCode(uint8_t fault_code, char *fault_desc, uint8_t desc_len);

#endif // MOTER_H