#include "servo.h"
#include "stm32f4xx_hal.h"       // 核心HAL类型（如HAL_StatusTypeDef）
#include "stm32f4xx_hal_dma.h"    // 若用到DMA_HandleTypeDef
#include "stm32f4xx_hal_uart.h"   // 若用到UART_HandleTypeDef
// 其他自定义头文件（如servo.h）
#include <stdint.h>
#include <string.h>
#include "stdio.h"
#include <stdarg.h>

uint8_t calculate_check_sum(uint8_t count, ...) {
    va_list args;
    va_start(args, count);  // 初始化可变参数列表
    
    uint16_t sum_total = 0;  // 用16位存储累加和，避免溢出
    for (uint8_t i = 0; i < count; i++) {
        // 逐个取出字段，累加（自动忽略高位，仅保留有效字节）
        sum_total += va_arg(args, int);
    }
    va_end(args);  // 释放可变参数列表
    
    uint8_t sum_low8 = sum_total & 0xFF;  // 取累加和低8位
    uint8_t check_sum = (~sum_low8) & 0xFF;  // 按位取反，确保结果为1字节
    
    return check_sum;
}

void Servo_SendCommand(UART_HandleTypeDef *huart, Servo *Servo) { 
    uint8_t power_off[8] = {0xFF, 0xFF ,0xFE ,0x04 ,0x03 ,0x18 ,0x00 ,0xE2};
    if (Servo->enabled == false) {
        HAL_UART_Transmit_DMA(huart, power_off, 8);
        return; // 控制器未启用，直接返回    
    }
    uint8_t tx_buf[10] = {0};
    uint8_t crc;
    uint16_t anger_raw = 3*Servo->position_aim; // 物理位置转换为原始值（单位: 度转为原始值）
    tx_buf[0] = 0XFF;        
    tx_buf[1] = 0xFF;            
    tx_buf[2] = 0XFE;            
    tx_buf[3] = 0x05;            
    tx_buf[4] = 0x03;            
    tx_buf[5] = 0x1E;            
    tx_buf[6] = anger_raw & 0xFF;            
    tx_buf[7] = (anger_raw >> 8);       
    crc = calculate_check_sum(6,tx_buf[2],tx_buf[3],tx_buf[4],tx_buf[5],tx_buf[6],tx_buf[7]);
    tx_buf[8] = crc;
    HAL_UART_Transmit_DMA(huart, tx_buf, 9);

}

void Servo_GetfeedBack(UART_HandleTypeDef *huart, Servo *Servo){
    uint8_t rx_buf[8] = {0};
    uint8_t get_feedback[8] = {0xFF ,0xFF ,0x01 ,0x04 ,0x02 ,0x24 ,0x02 ,0xD2};
    HAL_UART_Transmit(huart, get_feedback, 8, 100);
    HAL_UART_Receive(huart, rx_buf, 10, 500);
    uint8_t high_byte = rx_buf[5];  // 高8位
    uint8_t low_byte = rx_buf[6];   // 低8位
    uint16_t position_raw = (uint16_t)(high_byte << 8) | low_byte; // 合并为16位原始值
    Servo->position_real = position_raw / 3.0f;
}


