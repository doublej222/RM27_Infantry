//
// Created by JJ on 2026/11/14.
//

#pragma once
#include "rc/dr16.h"
#include "rc/ht10.h"

enum gimbal_mode_e {
    GIMBAL_DISABLE,     // 云台无力，所有控制量置0
    GIMBAL_FOLD,        // pitch 折叠
    GIMBAL_UNFOLD,      // pitch 展开 陀螺仪角度闭环
    GIMBAL_AUTO_AIM,    // 自瞄模式
};

typedef struct {
    rc::dr16::data_t rc;
    gimbal_mode_e gimbal_mode;
    gimbal_mode_e last_gimbal_mode;

    float gimable_picth_target;     // pitch 目标角度
    float gimbal_yaw_taget;         // yaw 目标角度
}gimbal_e;
extern gimbal_e gimbal;