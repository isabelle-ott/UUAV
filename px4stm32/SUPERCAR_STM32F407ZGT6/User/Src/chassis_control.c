#include "chassis_control.h"

void Set_Chassis_Vel(float vel_x, float vel_y, float vel_yaw)
{
    float cmd[4];

    cmd[1] = (vel_x - vel_y - vel_yaw * (Wheel_base + Wheel_track) / 2) / Wheel_Radius;
    cmd[3] = (vel_x + vel_y + vel_yaw * (Wheel_base + Wheel_track) / 2) / Wheel_Radius;
    cmd[2] = (vel_x + vel_y - vel_yaw * (Wheel_base + Wheel_track) / 2) / Wheel_Radius;
    cmd[4] = (vel_x - vel_y + vel_yaw * (Wheel_base + Wheel_track) / 2) / Wheel_Radius;

    motor1_control(cmd[1]);
    motor2_control(cmd[2]);
    motor3_control(cmd[3]);
    motor4_control(cmd[4]);
}