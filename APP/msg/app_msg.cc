//
// Created by fish on 2024/11/17.
//

#include "app_msg.h"

#include "app_ins.h"
#include "bsp_can.h"
#include "bsp_led.h"

#include "app_conf.h"

#include "sys_task.h"

#include <cstring>
#include <initializer_list>

#include "bsp_def.h"

/*
 *  app_msg
 *  实现双控制器之间的通信，以及对部分通信协议的封装。
 */

app_msg_dual_t dual_msg;
uint32_t dual_msg_id_chassis = 0x123, dual_msg_id_gimbal = 0x345;
bsp_can_e dual_msg_port = E_CAN2;

app_msg_dual_t *app_msg_dual() {
    return &dual_msg;
}

void app_msg_dual_send() {
    dual_msg.ins_yaw = INS::data()->yaw;
#ifdef COMPILE_GIMBAL
    bsp_can_send(dual_msg_port, dual_msg_id_gimbal, reinterpret_cast <uint8_t*> (&dual_msg));
#else
    bsp_can_send(dual_msg_port, dual_msg_id_chassis, reinterpret_cast <uint8_t*> (&dual_msg));
#endif
}

void app_msg_dual_recv(bsp_can_msg_t *msg) {
    memcpy(&dual_msg, msg->data, sizeof(app_msg_dual_t));
}

void app_msg_dual_init() {
#ifdef USE_DUAL_CONTROLLERS
#ifdef COMPILE_GIMBAL
    bsp_can_set_callback(dual_msg_port, dual_msg_id_chassis, app_msg_dual_recv);
#else
    bsp_can_set_callback(dual_msg_port, dual_msg_id_gimbal, app_msg_dual_recv);
#endif
#endif
}

void app_msg_dual_task(void *argument) {
#ifndef USE_DUAL_CONTROLLERS
    OS::Task::Current().Delete();
#endif
    while(true) {
        app_msg_dual_send();
        OS::Task::SleepMilliseconds(1);
    }
}

/*
 *  Vofa+ Justfloat
 */

float ch[APP_MSG_VOFA_CHANNEL_LIMIT];
uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

void app_msg_vofa_send(bsp_uart_e e, std::initializer_list <float> f) {
    BSP_ASSERT(0 < f.size() and f.size() <= APP_MSG_VOFA_CHANNEL_LIMIT);
    uint8_t p = 0;
    for(auto &i : f) {
        ch[p ++] = i;
    }
    bsp_uart_send(e, reinterpret_cast <uint8_t *> (&ch), f.size() * sizeof(float));
    bsp_uart_send(e, vofa_tail, sizeof vofa_tail);
}