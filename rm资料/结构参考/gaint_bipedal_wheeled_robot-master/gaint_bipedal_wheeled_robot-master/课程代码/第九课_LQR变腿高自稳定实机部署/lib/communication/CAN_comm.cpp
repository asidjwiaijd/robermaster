#include "CAN_comm.h"
#include <Arduino.h>

// CAN_device_t CAN_cfg;   
// uint32_t deviceID = 0x10;

#define RECEIVER_INTERVAL 0 // 限制接收频率，单位为毫秒
unsigned long lastReceiveTime = 0; // 记录上次接收时间
void CANInit(){
    // CAN_cfg.speed = CAN_SPEED_1000KBPS;
    // CAN_cfg.tx_pin_id = GPIO_NUM_22;
    // CAN_cfg.rx_pin_id = GPIO_NUM_19;
    // CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // // Init CAN Module
    // ESP32Can.CANInit();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_35, GPIO_NUM_41, TWAI_MODE_NORMAL); // 配置TX RX引脚
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // 配置波特率
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 配置滤波器 todo 增加滤波器

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("Driver installed");
    } else {
        Serial.println("Failed to install driver");
        return;
    }
    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    } else {
        Serial.println("Failed to start driver");
        return;
    }

    // Reconfigure alerts to detect TX alerts and Bus-Off errors
    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        Serial.println("CAN Alerts reconfigured");
    } else {
        Serial.println("Failed to reconfigure alerts");
        return;
    }
}

// 1.0版本接收函数
// void recCANMessage(){
//     twai_message_t rxFrame;
//     if(twai_receive(&rxFrame,  pdMS_TO_TICKS(10000)) == ESP_OK){
//         recNum++;
//         if(!rxFrame.extd){
//             if(!rxFrame.rtr){
//                 uint8_t DLC = rxFrame.data_length_code;
//                 uint8_t nodeID = rxFrame.identifier & 0x7F;
//                 uint32_t funcID = rxFrame.identifier & 0x780;
//                 // Serial.printf("%x,%x,%x,%x\n",DLC,nodeID,funcID,rxFrame.identifier);
//                 if(funcID == HEARTBEAT_FUNC_ID){
//                     MITState_callback(nodeID, rxFrame.data);

//                 }
//             }
//         }
//     }
// }

// 1.0.1接收版本函数 可以限制接收频率
void recCANMessage(){
    // 获取当前时间
    unsigned long currentTime = millis();
    
    // 判断是否到达接收频率限制
    if (currentTime - lastReceiveTime >= RECEIVER_INTERVAL) {
        // 如果是，那么清空缓存区并接收数据
        twai_message_t rxFrame;

        // 接收消息
        if (twai_receive(&rxFrame, pdMS_TO_TICKS(10000)) == ESP_OK) {
            lastReceiveTime = currentTime;  // 更新最后接收时间
            recNum++;

            if(!rxFrame.extd) {
                if(!rxFrame.rtr) {
                    uint8_t DLC = rxFrame.data_length_code;
                    uint8_t nodeID = rxFrame.identifier & 0x7F;
                    uint32_t funcID = rxFrame.identifier & 0x780;

                    // 处理心跳消息
                    if(funcID == HEARTBEAT_FUNC_ID) {
                        MITState_callback(nodeID, rxFrame.data);
                    }
                }
            }
        }
    }
}


// void doCANCommand(){
//     CAN_frame_t rxFrame;
//     if (xQueueReceive(CAN_cfg.rx_queue, &rxFrame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
//         if(rxFrame.FIR.B.FF == CAN_frame_std){ // 标准帧
//             if(rxFrame.FIR.B.RTR == CAN_no_RTR){ // 数据帧
//                 uint8_t DLC = rxFrame.FIR.B.DLC;
//                 uint8_t deviceID = rxFrame.MsgID>>DEVICEID_BITS;
//                 uint8_t commandID = rxFrame.MsgID&0x3F;
//                 uint8_t data[8];
//                  for(int i=0; i<8; i++){
//                     data[i] = rxFrame.data.u8[i]; 
//                 }
//                 switch (commandID){
//                     case HEARTBEAT_ID:
//                         MITState_callback(deviceID, data);
//                         break;
                    
//                     default:
//                         break;
//                     }
//             }else{ // 远程帧
//                 // todo
//             }
//         }
//     }
// }

void sendCANCommand(uint32_t nodeID, uint32_t msgID, uint8_t *data){
    twai_message_t txFrame;
    txFrame.extd = false; // 标准帧
    txFrame.identifier = msgID + nodeID; // 帧ID
    txFrame.data_length_code = 8; // DLC
    for(int i=0; i<8; i++){
        txFrame.data[i] = data[i];
    }
  // Queue message for transmission
  //超时时间为1ms，pdMS_TO_TICKS(1000)
  if (twai_transmit(&txFrame, pdMS_TO_TICKS(1000)) == ESP_OK) {
    // printf("Message queued for transmission\n");
    sendNum++;
  } else {
    printf("Failed to queue message for transmission\n");
  }


}

void MITState_callback(uint8_t nodeID, uint8_t *data){
    uint32_t posInt =  (data[1]<<8) | data[2];
    uint32_t velInt = (data[3]<<4) | (data[4]>>4 & 0xF);
    uint32_t torInt = ((data[4]&0xF)<<8) | data[5];

    nodeID = nodeID -1; // nodeid从0x01开始,数组是从0开始的
    // MITdata state;
    devicesState[nodeID].pos = uint_to_float(posInt,devicesState[nodeID].posMin, devicesState[nodeID].posMax, 16);
    devicesState[nodeID].vel = uint_to_float(velInt,devicesState[nodeID].velMin, devicesState[nodeID].velMax, 12);
    devicesState[nodeID].tor = uint_to_float(torInt,devicesState[nodeID].torMin, devicesState[nodeID].torMax, 12);
    

    // Serial.printf("%X,%.2f,%.2f,%.2f\n",deviceID,state.pos,state.vel,state.tor);
    
}




void sendMITCommand(uint8_t nodeID,MIT command){
    uint8_t MITcommand[8];

    LIMIT_MIN_MAX(command.pos, command.posMin, command.posMax);
    LIMIT_MIN_MAX(command.vel, command.velMin, command.velMax);
    LIMIT_MIN_MAX(command.kp, command.kpMin, command.kpMax);
    LIMIT_MIN_MAX(command.kd, command.kdMin, command.kdMax);
    LIMIT_MIN_MAX(command.tor, command.torMin, command.torMax);

    uint32_t posInt = float_to_uint(command.pos, command.posMin, command.posMax, 16);
    uint32_t velInt = float_to_uint(command.vel, command.velMin, command.velMax, 12);
    uint32_t kpInt = float_to_uint(command.kp, command.kpMin, command.kpMax, 12);
    uint32_t kdInt = float_to_uint(command.kd, command.kdMin, command.kdMax, 12);
    uint32_t torInt = float_to_uint(command.tor, command.torMin, command.torMax, 12);

    MITcommand[0] = posInt>>8;
    MITcommand[1] = posInt&0xFF;
    MITcommand[2] = velInt>>4;
    MITcommand[3] = ((velInt&0xF)<<4) | (kpInt>>8);
    MITcommand[4] = kpInt&0xFF;
    MITcommand[5] = kdInt>>4;
    MITcommand[6] = ((kdInt&0xF)<<4) | (torInt>>8);
    MITcommand[7] = torInt&0xFF;

    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}