//
// Created by JJ on 2026/10/31.
//
#pragma once

struct chassis_cmd_t {
    // chassis_mode_e mode = CHASSIS_LOCK;

    float vx = 0.f;
    float vy = 0.f;
    float wz = 0.f;

    // 两者同属世界航向坐标，单位rad；vx/vy以当前云台水平朝向为参考。
    float yaw_angle = 0.f;
    float yaw_target = 0.f;
};

struct chassis_feedback_t {
    float yaw_motor_angle = 0.f;
};
