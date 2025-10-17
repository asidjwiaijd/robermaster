#include "CAN_comm.h"
#include <Arduino.h>


void CANInit(){
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_35, GPIO_NUM_41, TWAI_MODE_NORMAL); 
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 


    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("Driver installed");
    } else {
        Serial.println("Failed to install driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    } else {
        Serial.println("Failed to start driver");
        return;
    }

    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        Serial.println("CAN Alerts reconfigured");
    } else {
        Serial.println("Failed to reconfigure alerts");
        return;
    }
}

void recCANMessage(){
    twai_message_t rxFrame;
    if(twai_receive(&rxFrame,  pdMS_TO_TICKS(1)) == ESP_OK){
        recNum++;
        if(!rxFrame.extd){
            if(!rxFrame.rtr){
                uint8_t DLC = rxFrame.data_length_code;
                uint8_t nodeID = rxFrame.identifier & 0x7F;
                uint32_t funcID = rxFrame.identifier & 0x780;
                // Serial.printf("%x,%x,%x,%x\n",DLC,nodeID,funcID,rxFrame.identifier);
                if(funcID == HEARTBEAT_FUNC_ID){
                    MITState_callback(nodeID, rxFrame.data);

                }
            }
        }
    }
}


void sendCANCommand(uint32_t nodeID, uint32_t msgID, uint8_t *data){
    twai_message_t txFrame;
    txFrame.extd = false; 
    txFrame.rtr = false; 
    txFrame.identifier = msgID + nodeID; 
    txFrame.data_length_code = 8; 
    for(int i=0; i<8; i++){
        txFrame.data[i] = data[i];
    }

    if (twai_transmit(&txFrame, pdMS_TO_TICKS(1000)) == ESP_OK) {
        sendNum++;
    } else {

    }


}

void MITState_callback(uint8_t nodeID, uint8_t *data){
    uint32_t posInt =  (data[1]<<8) | data[2];
    uint32_t velInt = (data[3]<<4) | (data[4]>>4 & 0xF);
    uint32_t torInt = ((data[4]&0xF)<<8) | data[5];

    uint8_t index = nodeID - 1;
    devicesState[index].pos = uint_to_float(posInt,devicesState[index].posMin, devicesState[index].posMax, 16);
    devicesState[index].vel = uint_to_float(velInt,devicesState[index].velMin, devicesState[index].velMax, 12);
    devicesState[index].tor = uint_to_float(torInt,devicesState[index].torMin, devicesState[index].torMax, 12);

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

void disable(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFD;
    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}

void enable(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFC;
    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}

void zeroPos(uint8_t nodeID){
    uint8_t MITcommand[8];
    MITcommand[0] = 0xFF;
    MITcommand[1] = 0xFF;
    MITcommand[2] = 0xFF;
    MITcommand[3] = 0xFF;
    MITcommand[4] = 0xFF;
    MITcommand[5] = 0xFF;
    MITcommand[6] = 0xFF;
    MITcommand[7] = 0xFE;

    sendCANCommand(nodeID, FUNC_ID_RPDO1, MITcommand);
}