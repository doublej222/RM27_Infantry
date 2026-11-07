//
// Created by JJ on 2026/10/31.
//

#ifndef TROBOT_DEF_H
#define TROBOT_DEF_H

enum chassis_run_e {
    chassis_disable,  // 底盘失能
    chassis_active    // 底盘使能
};

enum chassis_mode_e{
    FOLLOW,         // 底盘跟随云台
    NO_FOLLOW,    // 底盘独立控制
    SPIN            // 底盘小陀螺
};

struct chassis_cmd_t {
    chassis_run_e run = chassis_disable;
    chassis_mode_e mode = NO_FOLLOW;

    float vx = 0.f;
    float vy = 0.f;
    float wz = 0.f;

    // 两者同属世界航向坐标，单位rad；vx/vy以当前云台水平朝向为参考。
    float yaw_angle = 0.f;
    float yaw_target = 0.f;
};

struct chassis_feedback_t {
    float vx = 0.f;
    float vy = 0.f;
    float wz = 0.f;
};
enum gimbal_run_e {
    gimbal_disable,
    gimbal_active,
};
#endif //TROBOT_DEF_H
