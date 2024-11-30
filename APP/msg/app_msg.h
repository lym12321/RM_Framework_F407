//
// Created by fish on 2024/11/17.
//

#ifndef APP_MSG_H
#define APP_MSG_H

#include "bsp_uart.h"
#include <cstdint>
#include <initializer_list>

#define APP_MSG_VOFA_CHANNEL_LIMIT 10

void app_msg_vofa_send(bsp_uart_e e, std::initializer_list <float> f);

#endif //APP_MSG_H
