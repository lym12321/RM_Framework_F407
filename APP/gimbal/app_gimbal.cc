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
#include <cstdio>

#ifdef COMPILE_GIMBAL

MotorController <DJIMotor> m_yaw(
    "gimbal_yaw",
    DJIMotor::GM6020,
    { .id = 0x02, .port = E_CAN1, .mode = DJIMotor::VOLTAGE },
    PID_SPEED,
    { .Kp = 70.0, .Ki = 2.0, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 50, .iout_limit = 5 }
);

MotorController <DJIMotor> m_pitch(
    "gimbal_pitch",
    DJIMotor::GM6020,
    { .id = 0x01, .port = E_CAN1, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 15, .iout_limit = 1 }
);

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    float target;
    sscanf((char *) s, "%f", &target);
    m_yaw.target = target;
    m_yaw.activate();
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *argument) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    m_yaw.encoder_zero = m_yaw.device()->status.angle;
    m_pitch.encoder_zero = 7000;

    m_yaw.activate();

    uint32_t count = 0;

    while(true) {
        app_msg_vofa_send(E_UART_DEBUG, {
            m_yaw.target,
            m_yaw.speed,
            m_pitch.sum_angle
        });
        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    m_yaw.init(); m_pitch.init();
    m_yaw.relax();
    m_pitch.relax();
    m_yaw.use_ext_angle = true;
    m_pitch.use_ext_angle = true;
    m_yaw.use_degree_angle = true;
    m_pitch.use_degree_angle = true;
}

#endif