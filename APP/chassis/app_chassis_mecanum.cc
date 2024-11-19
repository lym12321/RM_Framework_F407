//
// Created by fish on 2024/11/16.
//

#include "app_chassis.h"

#include "app_motor.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "bsp_rc.h"
#include "app_conf.h"

/*
 *  适用于麦克纳姆轮（民航英雄）
 *  实现了基础的旋转、平移
 */

/*
 *  麦克纳姆轮
 *  ^ vy
 *  |       LU              RU
 *  |           O ------ O
 *  |           |        |
 *  |           |        |
 *  |           O ------ O
 *  |       LD              RD
 *  O------------------------------> vx
 *
 *  定义每个轮子的正速度为 vy 方向的速度，故 RU、RD 需要 reverse 一下
 *
 *  v_LU =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_LD =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_RU = -rotate * sqrt(2) + vy - vx * sqrt(2)
 *  v_RD = -rotate * sqrt(2) + vy - vx * sqrt(2)
 */

#ifdef COMPILE_CHASSIS_MECANUM

OS::Task chassis_task;

MotorController <DJIMotor> LU("chassis_left_up", DJIMotor::M3508, { .id = 0x01, .port = E_CAN2, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RU("chassis_right_up", DJIMotor::M3508, { .id = 0x02, .port = E_CAN2, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RD("chassis_left_down", DJIMotor::M3508, { .id = 0x03, .port = E_CAN2, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> LD("chassis_right_down", DJIMotor::M3508, { .id = 0x04, .port = E_CAN2, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});

// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0;
// 旋转速度
double rotate;

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *argument) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    while(true) {
        vx = 1.0 * bsp_rc_data()->rc_l[0] * 2, vy = 1.0 * bsp_rc_data()->rc_l[1] * 2;
        if(bsp_rc_data()->s_l == 1) rotate = 1000;
        else if(bsp_rc_data()->s_l == -1) rotate = -1000;
        else rotate = 1.0 * bsp_rc_data()->rc_r[0];

        LU.target =  rotate * M_SQRT2 + vy + vx * M_SQRT2;
        RD.target = -rotate * M_SQRT2 + vy + vx * M_SQRT2;
        LD.target =  rotate * M_SQRT2 + vy - vx * M_SQRT2;
        RU.target = -rotate * M_SQRT2 + vy - vx * M_SQRT2;

        // 下面的代码用来在电机速度过高的异常状态下停止电机。
        if(LU.device()->status.speed > 4000) LU.relax();
        if(LD.device()->status.speed > 4000) LD.relax();
        if(RU.device()->status.speed > 4000) RU.relax();
        if(RD.device()->status.speed > 4000) RD.relax();

        OS::Task::SleepMilliseconds(1);
    }
}

void app_chassis_init() {
    LU.init(); LD.init(); RU.init(); RD.init();
    RU.reverse = RD.reverse = true;
}

#endif
