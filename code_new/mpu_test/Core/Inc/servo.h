#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
uint8_t calculate_check_sum(uint8_t count, ...);
typedef struct {
    /* 目标与控制相关设置 */
    float setpoint;       /* 目标位置 */
    bool enabled;         /* 控制器使能标志 */
    float position_aim;         /* 物理位置（单位: 度 */
    float position_real;   /* 实际位置（单位: 度） */
    /* 状态标志与时间戳 */
} Servo;
void Servo_SendCommand(UART_HandleTypeDef *huart, Servo *Servo);
void Servo_GetfeedBack(UART_HandleTypeDef *huart, Servo *Servo);
#endif /* SERVO_H */