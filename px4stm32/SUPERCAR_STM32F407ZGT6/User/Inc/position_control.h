#pragma once

#include "stdint.h"
#include "encoder.h"

// #define WHEEL_RADIUS 0.034f // 车轮半径(m)
// #define WHEEL_TRACK 0.230f  // 左右轮距(m)
// #define WHEEL_BASE 0.094f   // 前后轮距(m)

typedef struct
{

} PositionControl;

void PositionControl_Init(PositionControl *pc);

void PositionControl_SetPosition(PositionControl *pc, float target_x, float target_y, float max_speed);

void PositionControl_Calculate_wheel_vel(PositionControl *pc);

void PositionControl_SetAngle(PositionControl *pc, float target_angle, float max_angular_speed);

void PositionControl_Stop(PositionControl *pc);

void PositionControl_GetPosition(PositionControl *pc);
