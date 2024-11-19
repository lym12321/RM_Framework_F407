//
// Created by fish on 2024/11/17.
//

#ifndef APP_MSG_H
#define APP_MSG_H

#include "bsp_uart.h"
#include <cstdint>
#include <initializer_list>
struct __attribute__((packed)) app_msg_dual_t {
    float ins_yaw;
    uint32_t reserved;
};

#define APP_MSG_VOFA_CHANNEL_LIMIT 10

app_msg_dual_t *app_msg_dual();
void app_msg_vofa_send(bsp_uart_e e, std::initializer_list <float> f);

#ifdef __cplusplus
extern "C" {
#endif

    void app_msg_dual_init();
    void app_msg_dual_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif //APP_MSG_H
