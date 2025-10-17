#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Robot/robot.h"

void serialCommandTask(void *pvParameters);
void sendMotorTargets(float motor1Target, float motor2Target);
bool receiveMotorTargets(float &motor1Target, float &motor2Target);
void motorInit();

#endif