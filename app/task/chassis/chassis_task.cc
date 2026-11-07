//
// Created by double_J on 2026/10/31.
//
#include "def.h"

#include "utils/os.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "controller/pid.h"
#include "utils/msg.h"

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

static chassis_run_e chassis_run = chassis_disable;
static chassis_mode_e chassis_mode;

// declaration
void motor_init();
void motor_disable();
void chassis_update(const float &vx, const float &vy, const float &rotate);
// task
[[noreturn]] void chassis_task(void *args) {
    chassis_rx.init();
    chassis_tx.init();

    float body_vx = 0, body_vy = 0, body_rotate = 0;
    while (true) {
        auto cmd = chassis_rx();

        if (chassis_run == chassis_active) {
            if (chassis_mode == FOLLOW) {
                pid follow_pid(4.f, 0.f, 0.f, 0.f, 6.f);

                float relative_yaw = remainder(yaw_motor.feedback.pos, 2.f * fpi);
                float yaw_error = remainder(cmd->yaw_target - cmd->yaw_angle, 2.f * fpi);
                float follow_error = remainder(relative_yaw + yaw_error, 2.f * fpi);

                // 允许车头或车尾对齐，误差折到(-π/2, π/2]。
                while (follow_error > fpi / 2.f) follow_error -= fpi;
                while (follow_error <= -fpi / 2.f) follow_error += fpi;

                body_vx = cmd->vx * cos(relative_yaw) - cmd->vy * sin(relative_yaw);
                body_vy = cmd->vx * sin(relative_yaw) + cmd->vy * cos(relative_yaw);
                body_rotate = follow_pid.update(follow_error, 0.f);
            } else if (chassis_mode == NO_FOLLOW) {

            } else if (chassis_mode == SPIN) {

            }

            // control
            yaw_motor.control(0, 0, 0, 0, 0);
            chassis_update(body_vx, body_vy, body_rotate);
        }
        else {
            motor_disable();
        }
        os::task::sleep(1);
    }
}

void motor_init() {
    motor_1.init(), motor_2.init(), motor_3.init(), motor_4.init();
    yaw_motor.init();
}

void motor_disable() {
    motor_1.update(0), motor_2.update(0), motor_3.update(0), motor_4.update(0);
    speed_pid1.clear(), speed_pid2.clear(), speed_pid3.clear(), speed_pid4.clear();
    yaw_motor.control(0, 0, 0, 0, 0);
}

void chassis_update(const float &vx, const float &vy, const float &rotate) {
    motor_1.update(speed_pid1.update(motor_1.feedback.speed, rotate / SQRT2 + vx + vy));
    motor_2.update(speed_pid2.update(motor_2.feedback.speed, rotate / SQRT2 + vx - vy));
    motor_3.update(speed_pid3.update(motor_3.feedback.speed, rotate / SQRT2 - vx - vy));
    motor_4.update(speed_pid4.update(motor_4.feedback.speed, rotate / SQRT2 - vx + vy));
}