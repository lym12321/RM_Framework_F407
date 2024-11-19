//
// Created by fish on 2024/11/16.
//

#include "app_motor.h"

#include "bsp_def.h"
#include "bsp_time.h"
#include "cmsis_os2.h"
#include "sys_task.h"

#define ABS(x) (((x) < 0) ? -(x) : (x))

static OS::Task motor_task;

static double angleDelta(double current, double target) {
    double dt = target - current;
    // 0 - 8191
    if(dt > 4096) dt -= 8192;
    if(dt < -4096) dt += 8192;
    return dt;
}

MotorController <DJIMotor> :: MotorController(const char *name, const DJIMotor::Model &model, const DJIMotor::Param &param,
        uint8_t control_mode,
        const Algorithm::PID::pid_param_t &pid_speed, const Algorithm::PID::pid_param_t &pid_angle) :
        motor_(name, model, param), control_mode_(control_mode), target(0.0) {

    pid_speed_.set_para(pid_speed);
    pid_angle_.set_para(pid_angle);
}

void MotorController <DJIMotor> ::init() {
    motor_.init();

    auto fn = [](MotorController *p) -> void {
        while(true) {
            p->task();
            osDelay(1);
        }
    };

    motor_task.Create(fn, this, "motor_task", 128, OS::Task::REALTIME);
}

void MotorController <DJIMotor> ::task() {
    // Relax
    // if(is_relax_) return;
    // Offline
    if(bsp_time_get_ms() - motor_.status.last_online_time > 500) {
        if(!offline_) {
            offline_ = true;
            target = 0;
            motor_.clear();
            pid_angle_.clear();
            pid_speed_.clear();
        }
        return;
    }

    offline_ = false;

    // 速度积分算总角度
    if(ext_angle) {
        sum_angle -= angleDelta(motor_.status.angle, lst_angle);
        lst_angle = motor_.status.angle;
    }

    // Relax
    if(is_relax_) return;

    if(ABS(motor_.status.current) > 10000) {
        if(++ err_count == 1000) {
            relax();
            return;
        }
    } else {
        err_count = 0;
    }

    double output = reverse ? -target : target;
    if(control_mode_ & PID_ANGLE) {
        // 建议 reverse 仅在速度环中使用，否则可能导致逻辑混乱
        BSP_ASSERT(!reverse);
        if(ext_angle)
            output = pid_angle_.update(sum_angle, output);
        else
            output = pid_angle_.update(0, angleDelta(motor_.status.angle, output));

    }
    if(control_mode_ & PID_SPEED) {
        output = pid_speed_.update(motor_.status.speed, output);
    }
    motor_.update(static_cast <float> (output));
}