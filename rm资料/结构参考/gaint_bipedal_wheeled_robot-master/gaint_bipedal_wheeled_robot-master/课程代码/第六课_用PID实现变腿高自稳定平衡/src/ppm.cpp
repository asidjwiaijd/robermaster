#include "ppm.h"

uint16_t ppmValues[NUM_CHANNELS] = {0} ;
int filteredPPMValues[NUM_CHANNELS] = {0};
uint32_t lastTime = 0;
uint8_t currentChannel = 0;
float alpha = 0.02;       // 设置滤波系数（值越小，平滑度越高）ime = 0;
uint8_t EH_rollflag;
int up_start = 0;  //一键启动


// 遥控器开关拨动 
void remote_switch()
{
  //通道五右上角
  if (ppmValues[4] < 1600 && ppmValues[4] > 1400)//机器人轮毂电机启动和机器人关闭自稳定
  {
    up_start = 1;
  }
  else if (ppmValues[4] < 1400)//机器人关闭轮毂电机和机器人关闭自稳定 最上面
  {
    
    up_start = 0;
  }
}

//遥控器PPM读取初始化
void ppm_init()
{
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), onPPMInterrupt, RISING); // 开启ppm读取中断
}

//遥控器低通滤波
float lowPassFilter(float currentValue, float previousValue, float alpha) 
{
  return alpha * currentValue + (1 - alpha) * previousValue;
}

//遥控器低通滤波接口
void storeFilteredPPMData()
{
  filteredPPMValues[0] = lowPassFilter(ppmValues[2], filteredPPMValues[0], alpha);//左遥杆左右腿高
  filteredPPMValues[1] = lowPassFilter(ppmValues[1], filteredPPMValues[1], alpha);//右摇杆前进后退
  filteredPPMValues[2] = lowPassFilter(ppmValues[5], filteredPPMValues[2], alpha);//重心调节
  filteredPPMValues[3] = lowPassFilter(ppmValues[0], filteredPPMValues[3], alpha);//右摇杆横向
  filteredPPMValues[4] = lowPassFilter(ppmValues[3], filteredPPMValues[4], alpha);//左遥杆横向
  filteredPPMValues[5] = lowPassFilter(ppmValues[0], filteredPPMValues[5], alpha);//转向
}

void IRAM_ATTR onPPMInterrupt()
{
  uint32_t now = micros();
  uint32_t duration = now - lastTime;
  lastTime = now;

  if (duration >= SYNC_GAP)
  {
    // Sync pulse detected, reset channel index
    currentChannel = 0;
  }
  else if (currentChannel < NUM_CHANNELS)
  {
    ppmValues[currentChannel] = duration;
    currentChannel++;
  }
}