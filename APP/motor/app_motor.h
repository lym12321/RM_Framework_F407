//
// Created by fish on 2024/11/16.
//

#pragma once
#include "alg_pid.h"
#include "dev_motor_dji.h"

#define MOTOR_CONTROLLER_LIMIT 16

typedef enum {
    PID_SPEED = 0b01,
    PID_ANGLE = 0b10
} app_motor_ctrl_e;

template <typename T>
class MotorController {
public:
    MotorController() = default;
};

template <>
class MotorController <DJIMotor> {
public:
    MotorController(const char *name, const DJIMotor::Model &model, const DJIMotor::Param &param,
        uint8_t control_mode,
        const Algorithm::PID::pid_param_t &pid_speed, const Algorithm::PID::pid_param_t &pid_angle);

    void init();
    void task();
    void relax() { is_relax_ = true, pid_speed_.clear(), pid_angle_.clear(), motor_.update(0); };
    void activate() { if(is_relax_) is_relax_ = false; };

    double target, lst_angle = 0, sum_angle = 0;
    DJIMotor *device() { return &motor_; };
    bool reverse = false;
    bool ext_angle = false;

private:
    uint16_t err_count = 0;
    bool is_relax_ = false, offline_ = false;
    uint8_t control_mode_;
    DJIMotor motor_;
    Algorithm::PID pid_speed_, pid_angle_;
};