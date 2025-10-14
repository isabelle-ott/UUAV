#pragma once

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "pid.h"

void Motor_StartPWM();

void motor1_SetVelocity(float velocity);
void motor2_SetVelocity(float velocity);
void motor3_SetVelocity(float velocity);
void motor4_SetVelocity(float velocity);

void motor1_control(float target);
void motor2_control(float target);
void motor3_control(float target);
void motor4_control(float target);
