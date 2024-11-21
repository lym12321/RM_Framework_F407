//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_ins.h"
#include "app_motor.h"
#include "dev_motor_dji.h"

#include "sys_task.h"

#include "app_sys.h"
#include "app_conf.h"
#include "app_msg.h"
#include "bsp_rc.h"
#include "bsp_uart.h"

#include "cmath"

#ifdef COMPILE_GIMBAL

MotorController <DJIMotor> m_yaw(
    "gimbal_yaw",
    DJIMotor::GM6020,
    { .id = 0x06, .port = E_CAN1, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 50, .iout_limit = 5 }
);

MotorController <DJIMotor> m_pitch(
    "gimbal_pitch",
    DJIMotor::GM6020,
    { .id = 0x05, .port = E_CAN1, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 15, .iout_limit = 1 }
);

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *argument) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    while(true) {

        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    m_yaw.init(); m_pitch.init();
    m_yaw.relax();
    m_pitch.relax();
    m_yaw.ext_angle = true;
}

#endif