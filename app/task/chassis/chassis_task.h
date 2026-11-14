//
// Created by JJ on 2026/11/14.
//
#pragma once
#include "stdint.h"

#define WHEEL_RADIUS 0.07615
#define WHEEL_CENTER_DISTANCE 0.258

enum chassis_mode_e{
    CHASSIS_DISABLE,           //
    CHASSIS_FOLLOW,         // 底盘跟随云台
    CHASSIS_SINGLE,         // 底盘独立控制
    CHASSIS_SPIN            // 底盘小陀螺
};

typedef struct {
    struct {
        int8_t s_l;

        float vx;       // 遥控器前后平移指令，坐标变换前，m/s
        float vy;       // 遥控器左右平移指令，坐标变换前，m/s
        float rotate;   // rad/s
    }rc;
    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    float vx_cmd;      // 控制指令 vx
    float vy_cmd;      // 控制指令 vy
    float rotate_cmd;  // 控制指令 rotate

    float ins_yaw_target;    // 云台目标值
    float ins_yaw_current;   // 云台当前值
}chassis_e;

extern chassis_e chassis;