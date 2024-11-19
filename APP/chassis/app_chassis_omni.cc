//
// Created by fish on 2024/11/16.
//

#include "app_chassis.h"

#include "app_motor.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "bsp_rc.h"
#include "app_conf.h"
#include "app_ins.h"
#include "app_msg.h"
#include "app_sys.h"

#include <cmath>

/*
 *  适用于全向轮（全向轮步兵、哨兵）
 *  实现了基础的旋转、平移
 */

/*
 *  全向轮
 *  ^ vy
 *  |       LU              RU
 *  |           / ------ \
 *  |           |        |
 *  |           |        |
 *  |           \ ------ /
 *  |       LD              RD
 *  O------------------------------> vx
 *
 *  不改变轮子正方向（默认为顺时针方向）
 *
 *  v_LU = rotate + vy * sqrt(2) + vx * sqrt(2)
 *  v_RU = rotate - vy * sqrt(2) + vx * sqrt(2)
 *  v_RD = rotate - vy * sqrt(2) - vx * sqrt(2)
 *  v_LD = rotate + vy * sqrt(2) - vx * sqrt(2)
 *
 *  对于全向轮步兵来说，该视图下 C板 在 LU 和 LD 之间
 */

#ifdef COMPILE_CHASSIS_OMNI

OS::Task chassis_task;

MotorController <DJIMotor> LU("chassis_left_up", DJIMotor::M3508, { .id = 0x02, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RU("chassis_right_up", DJIMotor::M3508, { .id = 0x01, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RD("chassis_left_down", DJIMotor::M3508, { .id = 0x04, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> LD("chassis_right_down", DJIMotor::M3508, { .id = 0x03, .port = E_CAN1, .mode = DJIMotor::CURRENT },
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
        if(bsp_rc_data()->s_l == 1) rotate = 1500;
        else if(bsp_rc_data()->s_l == -1) rotate = -1500;
        else rotate = 1.0 * bsp_rc_data()->rc_r[0];

        double r = std::sqrt(vx * vx + vy * vy), theta = std::atan2(vy, vx);
        theta -= INS::data()->yaw / 180 * M_PI;
        vx = r * std::cos(theta), vy = r * std::sin(theta);

        LU.target = rotate + vy * M_SQRT2 + vx * M_SQRT2;
        RD.target = rotate - vy * M_SQRT2 - vx * M_SQRT2;
        LD.target = rotate + vy * M_SQRT2 - vx * M_SQRT2;
        RU.target = rotate - vy * M_SQRT2 + vx * M_SQRT2;

        app_msg_vofa_send(E_UART_DEBUG,{
            LU.device()->status.speed,
            LD.device()->status.speed,
            RD.device()->status.speed,
            RU.device()->status.speed,
            INS::data()->yaw
            });

        OS::Task::SleepMilliseconds(1);
    }
}

void app_chassis_init() {
    LU.init(); LD.init(); RU.init(); RD.init();
    // LU.relax(); LD.relax(); RU.relax(); RD.relax();
}

#endif
