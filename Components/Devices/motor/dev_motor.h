//
// Created by fish on 2024/11/15.
//

#pragma once


#include <cstdint>

typedef struct {
    float speed, angle, current, temperature;
    uint32_t last_online_time;
} MotorStatus;

#include "dev_motor_dji.h"
