//
// Created by double_J on 2026/10/31.
//

#include "def.h"

#include "motor/dji.h"
#include "utils/os.h"
#include "motor/dm.h"
#include "utils/msg.h"

using namespace motor;

dm fold_picth("Fold", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});
dm aim_motor("Aim", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});

msg::can_sender<chassis_cmd_t, E_CAN_2, 0x301> gimbal_tx;
msg::can_receiver<chassis_feedback_t, E_CAN_2, 0x302> gimbal_rx;

[[noreturn]] void gimbal_task(void *args) {

    while (true) {

        os::task::sleep(1);
    }
}

void gimbal_init() {

}

