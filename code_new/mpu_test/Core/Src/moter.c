#include "moter.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "stdio.h"
// 硬件附加的发送数据长度（根据实际硬件调整，示例为10字节）
#define RX_OFFSET 1
// 通信缓冲区（需容纳附加数据+真实反馈数据）
uint8_t moter_send_buffer[32];
uint8_t moter_receive_buffer[64 + RX_OFFSET]; 

// 定义UART句柄（左前/右前=USART2，左后/右后=USART6）
#define UART_LF_RF  &huart2
#define UART_LR_RR  &huart6

/**
 * @brief  CRC-8/MAXIM算法实现（多项式：x^8 + x^5 + x^4 + 1 = 0x8C）
 * @param  data：待校验数据指针
 * @param  len：数据长度（字节）
 * @param  crc：输出校验结果
 */
void CRC_8(char *data, uint8_t len, uint8_t *crc) {
    uint8_t i, j;
    *crc = 0x00; 
    for (i = 0; i < len; i++) {
        *crc ^= *(data + i); 
        for (j = 0; j < 8; j++) {
            if ((*crc & 0x01) != 0) {
                *crc = (*crc >> 1) ^ 0x8C;
            } else {
                *crc >>= 1;
            }
        }
    }
}

/**
 * @brief  发送电机控制指令（速度/加速度/刹车）
 * @param  huart：UART句柄
 * @param  cmd：控制指令结构体指针
 * @param  motor_id：电机ID（1=左前，2=右前，3=左后，4=右后）
 * @param  fb：反馈结构体指针（用于存储解析结果）
 * @retval HAL状态
 */
HAL_StatusTypeDef Motor_SendCommand(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb) {
    uint8_t tx_buf[10] = {0};
    uint8_t crc;
    int16_t speed_raw = 0;       
    uint8_t acc_time_raw = 0;    
    uint8_t brake = 0;           
    uint8_t id = 0;              

    switch (motor_id) {
        case 1: 
            id = cmd->id_lf;
            speed_raw = (int16_t)cmd->speed_left_front_set;
            acc_time_raw = (uint8_t)cmd->acc_time_left_front_set;
            brake = cmd->brake_lf;
            break;
        case 2: 
            id = cmd->id_rf;
            speed_raw = (int16_t)cmd->speed_right_front_set;
            acc_time_raw = (uint8_t)cmd->acc_time_right_front_set;
            brake = cmd->brake_rf;
            break;
        case 3: 
            id = cmd->id_lr;
            speed_raw = (int16_t)cmd->speed_left_rear_set;
            acc_time_raw = (uint8_t)cmd->acc_time_left_rear_set;
            brake = cmd->brake_lr;
            break;
        case 4: 
            id = cmd->id_rr;
            speed_raw = (int16_t)cmd->speed_right_rear_set;
            acc_time_raw = (uint8_t)cmd->acc_time_right_rear_set;
            brake = cmd->brake_rr;
            break;
        default:
            return HAL_ERROR; 
    }

    tx_buf[0] = id;                      
    tx_buf[1] = 0x64;                    
    tx_buf[2] = (uint8_t)(speed_raw >> 8);
    tx_buf[3] = (uint8_t)(speed_raw & 0xFF);
    tx_buf[4] = 0x00;                    
    tx_buf[5] = 0x00;                    
    tx_buf[6] = acc_time_raw;            
    tx_buf[7] = brake;                   
    tx_buf[8] = 0x00;                    

    CRC_8((char *)tx_buf, 9, &crc);
    tx_buf[9] = crc;

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, tx_buf, 10, 100);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    status = HAL_UART_Receive(huart, moter_receive_buffer, 10 + RX_OFFSET, 500);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t *real_rx_buf = moter_receive_buffer + RX_OFFSET;
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc) {
        return HAL_ERROR;
    }

    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}

/**
 * @brief  接收电机反馈数据
 * @param  huart：UART句柄
 * @param  fb：反馈结构体指针
 * @param  motor_id：电机ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Motor_ReceiveFeedback(UART_HandleTypeDef *huart, moter_feedback *fb, uint8_t motor_id) {
    HAL_StatusTypeDef status;
    uint8_t crc;

    status = HAL_UART_Receive(huart, moter_receive_buffer, 10 + RX_OFFSET, 500);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t *real_rx_buf = moter_receive_buffer + RX_OFFSET;
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc) {
        return HAL_ERROR;
    }

    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}

/**
 * @brief  切换电机模式（使能/失能/开环/电流环/速度环）
 * @param  huart：UART句柄
 * @param  motor_id：电机ID
 * @param  mode：模式值
 * @param  fb：反馈结构体指针
 * @retval HAL状态
 */
HAL_StatusTypeDef Motor_SetMode(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode, moter_feedback *fb) {
    uint8_t tx_buf[10] = {0};
    uint8_t crc;

    tx_buf[0] = motor_id;        
    tx_buf[1] = 0xA0;            
    tx_buf[2] = mode;            
    tx_buf[3] = 0x00;            
    tx_buf[4] = 0x00;            
    tx_buf[5] = 0x00;            
    tx_buf[6] = 0x00;            
    tx_buf[7] = 0x00;            
    tx_buf[8] = 0x00;            

    CRC_8((char *)tx_buf, 9, &crc);
    tx_buf[9] = crc;             

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, tx_buf, 10, 100);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    status = HAL_UART_Receive(huart, moter_receive_buffer, 10 + RX_OFFSET, 500);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t *real_rx_buf = moter_receive_buffer + RX_OFFSET;
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc || real_rx_buf[1] != 0xA1) {
        return HAL_ERROR;
    }

    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}

/**
 * @brief  获取电机版本信息
 * @param  huart：UART句柄
 * @param  motor_id：电机ID
 * @param  version_info：输出版本信息
 * @param  fb：反馈结构体指针
 * @retval HAL状态
 */
HAL_StatusTypeDef Motor_GetVersion(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t *version_info, moter_feedback *fb) {
    uint8_t tx_buf[10] = {0};
    uint8_t crc;

    tx_buf[0] = motor_id;        
    tx_buf[1] = 0xFD;            
    tx_buf[2] = 0x00;            
    tx_buf[3] = 0x00;            
    tx_buf[4] = 0x00;            
    tx_buf[5] = 0x00;            
    tx_buf[6] = 0x00;            
    tx_buf[7] = 0x00;            
    tx_buf[8] = 0x00;            

    CRC_8((char *)tx_buf, 9, &crc);
    tx_buf[9] = crc;             

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, tx_buf, 10, 100);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    status = HAL_UART_Receive(huart, moter_receive_buffer, 10 + RX_OFFSET, 500);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t *real_rx_buf = moter_receive_buffer + RX_OFFSET;
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc || real_rx_buf[1] != 0xFE) {
        return HAL_ERROR;
    }

    version_info[0] = real_rx_buf[2]; 
    version_info[1] = real_rx_buf[3]; 
    version_info[2] = real_rx_buf[4]; 
    version_info[3] = real_rx_buf[5]; 
    version_info[4] = real_rx_buf[6]; 
    version_info[5] = real_rx_buf[7]; 
    version_info[6] = real_rx_buf[8]; 
    version_info[7] = 0x00;            

    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}
/**
 * @brief  解析电机反馈数据（优化：新增电流解析）
 * @param  rx_buf：接收缓冲区（真实反馈数据起始地址，10字节）
 * @param  fb：反馈结构体指针
 * @param  motor_id：电机ID（1=左前，2=右前，3=左后，4=右后）
 */
void Motor_ParseFeedback(uint8_t *rx_buf, moter_feedback *fb, uint8_t motor_id) {
    // 解析速度原始值（16位有符号，rx_buf[2]=高8位，rx_buf[3]=低8位）
    int16_t speed_raw = (rx_buf[2] << 8) | rx_buf[3];
    // 解析电流原始值（16位有符号，rx_buf[4]=高8位，rx_buf[5]=低8位，规格书定义）
    int16_t current_raw = (rx_buf[4] << 8) | rx_buf[5];
    
    switch (motor_id) {
        case 1: // 左前电机
            fb->speed_left_front = speed_raw;
            fb->speed_lf = ((float)speed_raw)/10; // 速度环：raw=rpm；电流环/开环：无效
            fb->current_left_front = current_raw;
            fb->current_lf = (float)current_raw / 8191.75f; // 电流转换：raw→A（系数8191.75）
            fb->acc_time_left_front = rx_buf[6];
            fb->acc_t_lf = (rx_buf[6] == 0) ? 1.0f : (float)rx_buf[6]; // 加速时间默认1ms
            fb->temp_left_front = rx_buf[7];
            fb->temp_lf = (float)rx_buf[7]; // 温度直接为℃
            fb->fault_left_front = rx_buf[8];
            break;
        case 2: // 右前电机
            fb->speed_right_front = speed_raw;
            fb->speed_rf = ((float)speed_raw)/10;
            fb->current_right_front = current_raw;
            fb->current_rf = (float)current_raw / 8191.75f;
            fb->acc_time_right_front = rx_buf[6];
            fb->acc_t_rf = (rx_buf[6] == 0) ? 1.0f : (float)rx_buf[6];
            fb->temp_right_front = rx_buf[7];
            fb->temp_rf = (float)rx_buf[7];
            fb->fault_right_front = rx_buf[8];
            break;
        case 3: // 左后电机
            fb->speed_left_rear = speed_raw;
            fb->speed_lr = ((float)speed_raw)/10;
            fb->current_left_rear = current_raw;
            fb->current_lr = (float)current_raw / 8191.75f;
            fb->acc_time_left_rear = rx_buf[6];
            fb->acc_t_lr = (rx_buf[6] == 0) ? 1.0f : (float)rx_buf[6];
            fb->temp_left_rear = rx_buf[7];
            fb->temp_lr = (float)rx_buf[7];
            fb->fault_left_rear = rx_buf[8];
            break;
        case 4: // 右后电机
            fb->speed_right_rear = speed_raw;
            fb->speed_rr = ((float)speed_raw)/10;
            fb->current_right_rear = current_raw;
            fb->current_rr = (float)current_raw / 8191.75f;
            fb->acc_time_right_rear = rx_buf[6];
            fb->acc_t_rr = (rx_buf[6] == 0) ? 1.0f : (float)rx_buf[6];
            fb->temp_right_rear = rx_buf[7];
            fb->temp_rr = (float)rx_buf[7];
            fb->fault_right_rear = rx_buf[8];
            break;
    }
}

/**
 * @brief  初始化4个电机，使能并设置为电流环模式
 * @param  fb：反馈结构体指针（用于存储初始化过程中的电机反馈）
 * @retval HAL状态（HAL_OK表示初始化成功）
 */
HAL_StatusTypeDef Motor_Init_All(moter_feedback *fb) {
    HAL_StatusTypeDef status;

    // 1. 左前电机初始化
    status = Motor_SetMode(UART_LF_RF, 1, 0x09, fb); // 失能左前电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LF_RF, 1, 0x08, fb); // 使能左前电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LF_RF, 1, 0x01, fb); // 设置左前电机为电流环模式
//    if (status != HAL_OK) return HAL_ERROR;


    // 2. 右前电机初始化
    status = Motor_SetMode(UART_LF_RF, 2, 0x09, fb); // 失能右前电机
  //  if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LF_RF, 2, 0x08, fb); // 使能右前电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LF_RF, 2, 0x01, fb); // 设置右前电机为电流环模式
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);

    // 3. 左后电机初始化
    status = Motor_SetMode(UART_LR_RR, 3, 0x09, fb); // 失能左后电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LR_RR, 3, 0x08, fb); // 使能左后电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LR_RR, 3, 0x01, fb); // 设置左后电机为电流环模式
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);


    // 4. 右后电机初始化
    status = Motor_SetMode(UART_LR_RR, 4, 0x09, fb); // 失能右后电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LR_RR, 4, 0x08, fb); // 使能右后电机
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = Motor_SetMode(UART_LR_RR, 4, 0x01, fb); // 设置右后电机为电流环模式
//    if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);

    return HAL_OK;
}

/**
 * @brief  发送电流环控制指令（电流给定）
 * @param  huart：UART句柄（左前/右前=USART2，左后/右后=USART6）
 * @param  cmd：控制指令结构体指针（含电流设定值）
 * @param  motor_id：电机ID（1=左前，2=右前，3=左后，4=右后）
 * @param  fb：反馈结构体指针（存储电流反馈）
 * @retval HAL状态（HAL_OK=成功，HAL_ERROR=失败）
 */
HAL_StatusTypeDef Motor_SendCurrentCmd(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb) {
    uint8_t tx_buf[10] = {0};
    uint8_t crc;
    int16_t current_raw = 0;  // 电流原始值（-32767~32767 对应 -4A~4A）
    uint8_t id = 0;           // 电机ID（1/2）
    uint8_t receive_buffer[64] = {0};
    // 1. 根据电机ID获取电流设定值与电机ID
    switch (motor_id) {
        case 1: // 左前电机
            id = cmd->id_lf;
            // 电流转换：A → 原始值（系数=32767/4 ≈ 8191.75）
            current_raw = (int16_t)(cmd->current_left_front_set * 8191.75f);
            // 限制原始值范围（防止溢出）
            current_raw = (current_raw < -32767) ? -32767 : (current_raw > 32767) ? 32767 : current_raw;
            break;
        case 2: // 右前电机
            id = cmd->id_rf;
            current_raw = (int16_t)(cmd->current_right_front_set * 8191.75f);
            current_raw = (current_raw < -32767) ? -32767 : (current_raw > 32767) ? 32767 : current_raw;
            break;
        case 3: // 左后电机
            id = cmd->id_lr;
            current_raw = (int16_t)(cmd->current_left_rear_set * 8191.75f);
            current_raw = (current_raw < -32767) ? -32767 : (current_raw > 32767) ? 32767 : current_raw;
            break;
        case 4: // 右后电机
            id = cmd->id_rr;
            current_raw = (int16_t)(cmd->current_right_rear_set * 8191.75f);
            current_raw = (current_raw < -32767) ? -32767 : (current_raw > 32767) ? 32767 : current_raw;
            break;
        default:
            return HAL_ERROR; // 无效电机ID
    }
    
    // 2. 构造10字节指令帧（符合规格书0x64指令格式）
    tx_buf[0] = id;                       // DATA[0]：电机ID（1/2）
    tx_buf[1] = 0x64;                     // DATA[1]：指令码（电流/速度/电压给定）
    tx_buf[2] = (uint8_t)(current_raw >> 8); // DATA[2]：电流原始值高8位
    tx_buf[3] = (uint8_t)(current_raw & 0xFF); // DATA[3]：电流原始值低8位
    tx_buf[4] = 0x00;                     // DATA[4]：保留（0）
    tx_buf[5] = 0x00;                     // DATA[5]：保留（0）
    tx_buf[6] = 0x00;                     // DATA[6]：加速时间（电流环无效，设0）
    tx_buf[7] = 0x00;                     // DATA[7]：刹车（不刹车，设0）
    tx_buf[8] = 0x00;                     // DATA[8]：保留（0）
    CRC_8((char *)tx_buf, 9, &crc);       // 计算CRC8校验（DATA[0]~DATA[8]）
    tx_buf[9] = crc;                      // DATA[9]：CRC8校验值
    
    // 3. 发送指令
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, tx_buf, 10, 100); // 超时100ms
    if (status != HAL_OK) {
        return HAL_ERROR; // 发送失败
    }
    
    // 4. 接收电机反馈（10字节反馈+硬件偏移RX_OFFSET）
    status = HAL_UART_Receive(huart, receive_buffer, 10 + RX_OFFSET, 500); // 超时500ms
    if (status != HAL_OK) {
        return HAL_ERROR; // 接收失败
    }
    
    // 5. 校验反馈数据CRC
    uint8_t *real_rx_buf = receive_buffer + RX_OFFSET; // 跳过硬件偏移
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc || real_rx_buf[1] != 0x65) { // 反馈指令码需为0x65
        return HAL_ERROR; // CRC校验失败或指令码错误
    }
    
    // 6. 解析反馈数据（含电流）
    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}

/**
 * @brief  发送开环控制指令（电压给定）
 * @param  huart：UART句柄（左前/右前=USART2，左后/右后=USART6）
 * @param  cmd：控制指令结构体指针（含电压设定值）
 * @param  motor_id：电机ID（1=左前，2=右前，3=左后，4=右后）
 * @param  fb：反馈结构体指针（存储温度、故障码）
 * @retval HAL状态（HAL_OK=成功，HAL_ERROR=失败）
 */
HAL_StatusTypeDef Motor_SendOpenLoopCmd(UART_HandleTypeDef *huart, moter_command *cmd, uint8_t motor_id, moter_feedback *fb) {
    uint8_t tx_buf[10] = {0};
    uint8_t crc;
    uint16_t voltage_raw = 0; // 电压原始值（0~32767 对应 0~25.2V）
    uint8_t id = 0;           // 电机ID（1/2）
    
    // 1. 根据电机ID获取电压设定值与电机ID
    switch (motor_id) {
        case 1: // 左前电机
            id = cmd->id_lf;
            // 电压转换：V → 原始值（系数=32767/25.2 ≈ 1299.88）
            voltage_raw = (uint16_t)(cmd->voltage_left_front_set * 1299.88f);
            // 限制原始值范围（防止溢出，且开环无负电压）
            voltage_raw = (voltage_raw > 32767) ? 32767 : voltage_raw;
            break;
        case 2: // 右前电机
            id = cmd->id_rf;
            voltage_raw = (uint16_t)(cmd->voltage_right_front_set * 1299.88f);
            voltage_raw = (voltage_raw > 32767) ? 32767 : voltage_raw;
            break;
        case 3: // 左后电机
            id = cmd->id_lr;
            voltage_raw = (uint16_t)(cmd->voltage_left_rear_set * 1299.88f);
            voltage_raw = (voltage_raw > 32767) ? 32767 : voltage_raw;
            break;
        case 4: // 右后电机
            id = cmd->id_rr;
            voltage_raw = (uint16_t)(cmd->voltage_right_rear_set * 1299.88f);
            voltage_raw = (voltage_raw > 32767) ? 32767 : voltage_raw;
            break;
        default:
            return HAL_ERROR; // 无效电机ID
    }
    
    // 2. 构造10字节指令帧（符合规格书0x64指令格式）
    tx_buf[0] = id;                       // DATA[0]：电机ID（1/2）
    tx_buf[1] = 0x64;                     // DATA[1]：指令码（电流/速度/电压给定）
    tx_buf[2] = (uint8_t)(voltage_raw >> 8); // DATA[2]：电压原始值高8位
    tx_buf[3] = (uint8_t)(voltage_raw & 0xFF); // DATA[3]：电压原始值低8位
    tx_buf[4] = 0x00;                     // DATA[4]：保留（0）
    tx_buf[5] = 0x00;                     // DATA[5]：保留（0）
    tx_buf[6] = 0x00;                     // DATA[6]：加速时间（开环无效，设0）
    tx_buf[7] = 0x00;                     // DATA[7]：刹车（不刹车，设0）
    tx_buf[8] = 0x00;                     // DATA[8]：保留（0）
    CRC_8((char *)tx_buf, 9, &crc);       // 计算CRC8校验（DATA[0]~DATA[8]）
    tx_buf[9] = crc;                      // DATA[9]：CRC8校验值
    
    // 3. 发送指令
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, tx_buf, 10, 100); // 超时100ms
    if (status != HAL_OK) {
        return HAL_ERROR; // 发送失败
    }
    
    // 4. 接收电机反馈（10字节反馈+硬件偏移RX_OFFSET）
    status = HAL_UART_Receive(huart, moter_receive_buffer, 10 + RX_OFFSET, 500); // 超时500ms
    if (status != HAL_OK) {
        return HAL_ERROR; // 接收失败
    }
    
    // 5. 校验反馈数据CRC
    uint8_t *real_rx_buf = moter_receive_buffer + RX_OFFSET; // 跳过硬件偏移
    CRC_8((char *)real_rx_buf, 9, &crc);
    if (real_rx_buf[9] != crc || real_rx_buf[1] != 0x65) { // 反馈指令码需为0x65
        return HAL_ERROR; // CRC校验失败或指令码错误
    }
    
    // 6. 解析反馈数据（开环无速度/电流反馈，仅解析温度、故障码）
    Motor_ParseFeedback(real_rx_buf, fb, motor_id);
    return HAL_OK;
}
/**
 * @brief  解析电机故障码（基于规格书故障定义）
 * @param  fault_code：故障码（uint8_t，来自反馈结构体）
 * @param  fault_desc：故障描述输出缓冲区
 * @param  desc_len：缓冲区最大长度
 */
void Motor_ParseFaultCode(uint8_t fault_code, char *fault_desc, uint8_t desc_len) {
    if (fault_desc == NULL || desc_len < 10) {
        return; // 缓冲区无效
    }
    
    // 清空描述缓冲区
    memset(fault_desc, 0, desc_len);
    
    // 无故障
    if (fault_code == 0x00) {
        snprintf(fault_desc, desc_len, "No Fault");
        return;
    }
    
    // 按BIT位解析故障（规格书定义：BIT0=霍尔，BIT1=过流，BIT3=堵转，BIT4=过温，BIT5=断联，BIT6=过欠压）
    char *fault_list[8] = {
        "Hall Fault",    // BIT0
        "Overcurrent",   // BIT1
        "Reserved",      // BIT2（保留）
        "Stall",         // BIT3
        "Over Temperature", // BIT4
        "Disconnect",    // BIT5
        "Over/Under Voltage", // BIT6
        "Reserved"       // BIT7（保留）
    };
    
    uint8_t fault_count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (fault_code & (1 << i)) {
            if (fault_count > 0) {
                strncat(fault_desc, " | ", desc_len - strlen(fault_desc) - 1);
            }
            strncat(fault_desc, fault_list[i], desc_len - strlen(fault_desc) - 1);
            fault_count++;
        }
    }
}
