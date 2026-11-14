//
// Created by double_J on 2026/10/31.
//

#include "def.h"
#include "gimbal.h"
#include "controller/pid.h"

#include "motor/dji.h"
#include "utils/os.h"
#include "motor/dm.h"
#include "utils/msg.h"
#include "rc/dr16.h"

using namespace motor;
using namespace controller;
// 位置速度模式
dm fold_picth("Fold", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});
// MIT 模式
dm pitch("pitch", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});
dji ammo_left("ammo_left", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 1);
dji ammo_right("ammo_right", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 1);

pid ammo_left_pid(750, 20, 5, 16384, 16384);
pid ammo_right_pid(750, 20, 5, 16384, 16384);

msg::can_sender<chassis_cmd_t, E_CAN_2, 0x301> gimbal_tx;
msg::can_receiver<chassis_feedback_t, E_CAN_2, 0x302> gimbal_rx;

gimbal_e gimbal;
// declaration
void gimbal_init();
void gimbal_setmode();
void gimbal_reference();
void gimbal2chassis_data();
// task
[[noreturn]] void gimbal_task(void *args) {
    gimbal_tx.init();

    while (true) {
        // message receive


        // yaw


        // control
        ammo_left.update(0);
        ammo_right.update(0);

        // message send
        gimbal2chassis_data();
        os::task::sleep(1);
    }
}

void gimbal_init() {

}
void gimbal_setmode() {
    if (gimbal.rc.s_l == -1) gimbal.gimbal_mode = GIMBAL_DISABLE;
    else if (gimbal.rc.s_l == 0) gimbal.gimbal_mode = GIMBAL_FOLD;
    else if (gimbal.rc.s_l == 0 and gimbal.rc.s_r == 0) gimbal.gimbal_mode = GIMBAL_UNFOLD;
}
void gimbal_reference() {
    switch (gimbal.gimbal_mode) {
        case GIMBAL_DISABLE : {
            fold_picth.control(0, 0, 0, 0,0);
            ammo_left.update(0), ammo_right.update(0);
            ammo_left_pid.clear(); ammo_right_pid.clear();

            break;
            }
        case GIMBAL_FOLD : {

        }
        case GIMBAL_UNFOLD:
        case GIMBAL_AUTO_AIM:
        default: break;
    }
}
auto cmd = gimbal_tx();
void gimbal2chassis_data() {

    auto rc = ::rc::dr16::data();

    cmd->vx = static_cast<float>(rc->rc_l[0]) / 20.f;
    cmd->vy = static_cast<float>(rc->rc_l[1]) / 20.f;
    cmd->wz = 0;

    gimbal_tx.send();
}


