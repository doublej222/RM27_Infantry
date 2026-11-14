//
// Created by JJ on 2026/11/15.
//
#include "shoot_task.h"

#include "bsp/uart.h"
#include "utils/os.h"
#include "motor/dji.h"
#include "controller/pid.h"
#include "filter/lowpass_filter.h"
#include "utils/vofa.h"

using namespace motor;
using namespace controller;
using namespace filter;

dji trigger_motor("trigger", dji::M2006,dji::param_t{.id = 5,.port = E_CAN_1,.mode = dji::CURRENT}, -1);
pid trigger_pid(300, 0, 0, 16384, 16384);
// low_pass_filter trigger_filter();

[[noreturn]] void shooter_task(void *args) {

    while (true) {

        trigger_motor.update(trigger_pid(trigger_motor.feedback.speed, 0));
        vofa::send(E_UART_1, trigger_motor.feedback.timestamp, trigger_motor.feedback.speed);
        os::task::sleep(1);
    }
}