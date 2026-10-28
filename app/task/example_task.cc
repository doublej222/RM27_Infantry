//
// Created by fish on 2026/1/24.
//
#include <cstdio>

#include "controller/pid.h"
#include "motor/dji.h"
#include "utils/os.h"
#include "utils/vofa.h"

using namespace controller;
using namespace motor;
dji test("test", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
pid spd_pid(750, 20, 5, 16384, 16384);

float spd = 5;
[[noreturn]] void example_task(void *args) {
    test.init();

    bsp_uart_set_callback(E_UART_1, [](bsp_uart_e device, const uint8_t *data, size_t len) {
        sscanf(reinterpret_cast<const char *>(data), "%f", &spd);
    });

    for (;;) {
        // test
        float motor_spd = spd_pid.update(test.feedback.speed, spd);

        test.update(motor_spd);

        vofa::send(E_UART_1, test.feedback.speed, spd);
        os::task::sleep(1);
    }
}
