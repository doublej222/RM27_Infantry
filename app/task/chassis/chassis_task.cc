//
// Created by double_J on 2026/10/31.
//
#include "def.h"

#include "utils/os.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "controller/pid.h"

constexpr float SQRT2 = M_SQRT2;
constexpr float fpi = M_PI;

using namespace motor;
using namespace controller;

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

    float body_vx = 0, body_vy = 0, body_rotate = 0;
    while (true) {
        if (chassis_run == chassis_active) {
            if (chassis_mode == follow) {

            } else if (chassis_mode == independent) {

            } else if (chassis_mode == spin) {

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
    // motor_1.disable(), motor_2.disable(), motor_3.disable(), motor_4.disable();
    // yaw_motor.disable();
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