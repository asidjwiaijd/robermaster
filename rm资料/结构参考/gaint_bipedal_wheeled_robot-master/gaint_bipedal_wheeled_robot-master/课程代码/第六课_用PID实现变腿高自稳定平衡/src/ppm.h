#ifndef PPM_H
#define PPM_H

#include "Arduino.h"

#define PPM_PIN 40 
#define NUM_CHANNELS 8                                                                         // 输出通道
#define SYNC_GAP 3000 

extern uint16_t ppmValues[];
extern int filteredPPMValues[];
extern int up_start;  //一键启动
extern uint8_t currentChannel;
extern uint32_t lastTime ;
extern uint8_t EH_rollflag;

void ppm_init();
void storeFilteredPPMData();
float lowPassFilter(float currentValue, float previousValue, float alpha);
void IRAM_ATTR onPPMInterrupt();
void remote_switch();

#endif