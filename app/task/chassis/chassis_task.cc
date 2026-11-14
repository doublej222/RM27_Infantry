//
// Created by JJ on 2026/10/31.
//
#include "chassis_task.h"

#include "def.h"

#include "utils/os.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "controller/pid.h"
#include "utils/msg.h"
#include "utils/vofa.h"

constexpr float SQRT2 = M_SQRT2;
constexpr float fpi = M_PI;

using namespace motor;
using namespace controller;
msg::can_receiver<chassis_cmd_t, E_CAN_2, 0x301> chassis_rx;
msg::can_sender<chassis_feedback_t, E_CAN_2, 0x302> chassis_tx;

dm yaw_motor("Yaw", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});


dji motor_1("FL", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_2("BL", dji::M3508,dji::param_t{.id = 2,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_3("BR", dji::M3508,dji::param_t{.id = 3,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_4("FR", dji::M3508,dji::param_t{.id = 4,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
pid speed_pid1(750, 20, 5, 16384, 16384);
pid speed_pid2(750, 20, 5, 16384, 16384);
pid speed_pid3(750, 20, 5, 16384, 16384);
pid speed_pid4(750, 20, 5, 16384, 16384);

pid follow_pid(4.f, 0.f, 0.f, 0.f, 3.f);

chassis_e chassis;

// declaration
void chassis_init();
void chassis_setmode();
void chassis_reference();
void chassis2gimbal_data();
void chassis_update(const float &vx, const float &vy, const float &rotate);
// task
[[noreturn]] void chassis_task(void *args) {
    // chassis_rx.init();
    // chassis_tx.init();
    chassis_init();

    while (true) {
        chassis_update(0, 0, 0);


        os::task::sleep(1);
    }
}

void chassis_init() {
    motor_1.init(), motor_2.init(), motor_3.init(), motor_4.init();
    yaw_motor.init();
}
void chassis_setmode() {
    switch (chassis.rc.s_l) {
        case -1:chassis.chassis_mode = CHASSIS_DISABLE; break;
        case  0:chassis.chassis_mode = CHASSIS_FOLLOW; break;
        case  1:chassis.chassis_mode = CHASSIS_SPIN; break;
        default: ;
    }
}
void chassis_reference() {
    switch (chassis.chassis_mode) {
        case CHASSIS_DISABLE : {
            chassis.vy_cmd = 0, chassis.vy_cmd = 0; chassis.rotate_cmd = 0;
            motor_1.update(0), motor_2.update(0), motor_3.update(0), motor_4.update(0);
            speed_pid1.clear(), speed_pid2.clear(), speed_pid3.clear(), speed_pid4.clear();
            yaw_motor.control(0, 0, 0, 0, 0);

            break;
            }

        case CHASSIS_FOLLOW : {
            if (chassis.last_chassis_mode != CHASSIS_FOLLOW) {
                follow_pid.clear();
            }
            float relative_yaw = remainder(yaw_motor.feedback.pos, 2.f * fpi);
            // TODO:测试作为前馈值 or pid目标量
            float yaw_error = remainder(chassis.ins_yaw_target - chassis.ins_yaw_current, 2.f * fpi);
            float follow_error = remainder(relative_yaw + yaw_error, 2.f * fpi);

            // 对齐车头或车尾，误差折到(-π/2, π/2]
            while (follow_error > fpi / 2.f) follow_error -= fpi;
            while (follow_error <= -fpi / 2.f) follow_error += fpi;

            chassis.vx_cmd = chassis.rc.vx * cos(relative_yaw) - chassis.rc.vy * sin(relative_yaw);
            chassis.vy_cmd = chassis.rc.vx * sin(relative_yaw) + chassis.rc.vy * cos(relative_yaw);
            chassis.rotate_cmd = follow_pid.update(follow_error, 0.f);

            break;
            }
        case CHASSIS_SPIN : {

            }
        case CHASSIS_SINGLE :
        default: break;
    }

    chassis.last_chassis_mode = chassis.chassis_mode;


    // control
    // yaw_motor.control(0, 0, 0, 0, 0);
    // chassis_update(chassis.vx_cmd, chassis.vy_cmd, chassis.rotate_cmd);
}
void chassis_update(const float &vx, const float &vy, const float &rotate) {
    motor_1.update(speed_pid1.update(motor_1.feedback.speed, ((  vy - vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_2.update(speed_pid2.update(motor_2.feedback.speed, (( -vy - vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_3.update(speed_pid3.update(motor_3.feedback.speed, (( -vy + vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_4.update(speed_pid4.update(motor_4.feedback.speed, ((  vy + vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
}
void chassis2gimbal_data() {
    auto cmd = chassis_tx();

    cmd->yaw_motor_angle = remainder(yaw_motor.feedback.pos, 2.f * fpi);

    chassis_tx.send();
}
