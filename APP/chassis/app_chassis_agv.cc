//
// Created by fish on 2024/11/21.
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
#include <cstdio>
#include <array>

/*
 *  适用于舵轮（舵轮步兵）
 *  实现了基础的旋转、平移
 */

/*
 *  全向轮
 *  ^ vy
 *  |       1                4
 *  |           / ------ \
 *  |           |        |
 *  |           |        |
 *  |           \ ------ /
 *  |       2               3
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

#ifdef COMPILE_CHASSIS_AGV

OS::Task chassis_task;

MotorController <DJIMotor> S1("chassis_servo_1", DJIMotor::GM6020, { .id = 0x01, .port = E_CAN2, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 125, .Ki = 7.0, .Kd = 0.01, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 2.5, .Ki = 0.0, .Kd = 0.0, .out_limit = 150, .iout_limit = 0});
MotorController <DJIMotor> S2("chassis_servo_2", DJIMotor::GM6020, { .id = 0x02, .port = E_CAN2, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 125, .Ki = 7.0, .Kd = 0.01, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 2.5, .Ki = 0.0, .Kd = 0.0, .out_limit = 150, .iout_limit = 0});
MotorController <DJIMotor> S3("chassis_servo_3", DJIMotor::GM6020, { .id = 0x03, .port = E_CAN2, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 125, .Ki = 7.0, .Kd = 0.01, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 2.5, .Ki = 0.0, .Kd = 0.0, .out_limit = 150, .iout_limit = 0});
MotorController <DJIMotor> S4("chassis_servo_4", DJIMotor::GM6020, { .id = 0x04, .port = E_CAN2, .mode = DJIMotor::VOLTAGE },
    PID_SPEED | PID_ANGLE,
    { .Kp = 125, .Ki = 7.0, .Kd = 0.01, .out_limit = 25000, .iout_limit = 20000 },
    { .Kp = 2.5, .Ki = 0.0, .Kd = 0.0, .out_limit = 150, .iout_limit = 0});

MotorController <DJIMotor> W1("chassis_wheel_1", DJIMotor::M3508, { .id = 0x01, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> W2("chassis_wheel_2", DJIMotor::M3508, { .id = 0x02, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> W3("chassis_wheel_3", DJIMotor::M3508, { .id = 0x03, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> W4("chassis_wheel_4", DJIMotor::M3508, { .id = 0x04, .port = E_CAN1, .mode = DJIMotor::CURRENT },
    PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});

constexpr float eps = 1e-8;

template <typename T>
constexpr uint8_t sgn(T x) {
    return x > eps ? 1 : (x < -eps ? -1 : 0);
}

template <typename T>
constexpr T conv(T x) {
    return std::abs(x) < 10 * eps ? 0 : x;
}

// 给一个完整的执行机构一个结构体
struct SW {
    MotorController <DJIMotor> *servo = nullptr, *wheel = nullptr;
    bool reversed = false;
    double vx_ = 0, vy_ = 0, angle_ = 0;
    void upd(double vx, double vy) {
        vx *= -1;
        auto angle = std::atan2(vy, vx);
        if(std::min(std::abs(angle - angle_), 2 * M_PI - std::abs(angle - angle_)) > M_PI / 2) {
            reversed ^= 1;
        }
        auto cur_angle = static_cast <float> (angle < 0 ? 2 * M_PI + angle : angle), cur_speed = static_cast <float> (std::sqrt(vx * vx + vy * vy));
        cur_angle = static_cast <float> (cur_angle / M_PI * 180 + 270);

        if(reversed) {
            cur_angle += 180;
            cur_speed *= -1;
        }

        while(cur_angle >= 360) cur_angle -= 360;
        while(cur_angle < 0) cur_angle += 360;

        servo->target = cur_angle, wheel->target = cur_speed;
        vx_ = vx, vy_ = vy, angle_ = angle;
    }
};

SW SW1 { &S1, &W1 };
SW SW2 { &S2, &W2 };
SW SW3 { &S3, &W3 };
SW SW4 { &S4, &W4 };

// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
float vx = 0, vy = 0;
// 旋转速度
float rotate = 0;

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    float target;
    sscanf((char *) s, "%f", &target);
    S1.target = target;
}

std::array <double, 2> calc(const std::array <double, 2> &v1, const std::array <double, 2> &v2) {
    // 合成速度
    // auto [x1, y1] = v1; auto [x2, y2] = v2;
    // if(x1 == 0 and y1 == 0) return v2;
    // if(x2 == 0 and y2 == 0) return v1;
    // if(y1 == 0) y1 = eps;
    // if(y2 == 0) y2 = eps;
    // // y = kx + m
    // double k1 = -x1 / y1, k2 = -x2 / y2;
    // double m1 = y1 - k1 * x1, m2 = y2 - k2 * x2;
    // if(std::abs(k1 - k2) < 0.5) {
    //     // 共线
    //     return {x1 + x2, y1 + y2};
    // }
    // double x = (m2 - m1) / (k1 - k2), y = k1 * x + m1;
    // return {x, y};
    return {v1[0] + v2[0], v1[1] + v2[1]};
}

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *argument) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    S1.activate();
    S2.activate();
    S3.activate();
    S4.activate();
    W1.activate();
    W2.activate();
    W3.activate();
    W4.activate();

    uint16_t zero_count = 0;
    while(true) {
        vx = bsp_rc_data()->rc_l[0] * 2.5, vy = bsp_rc_data()->rc_l[1] * 2.5;
        rotate = bsp_rc_data()->s_l * 1500.0 - bsp_rc_data()->rc_r[0] * 7.0;

        double r = std::sqrt(vx * vx + vy * vy), theta = std::atan2(vy, vx);
        theta -= INS::data()->yaw / 180 * M_PI;
        vx = r * std::cos(theta), vy = r * std::sin(theta);

        if(rotate == 0) rotate = eps;

        if(vx == 0 and vy == 0) {
            if(rotate > eps or zero_count == 500) {
                SW1.upd(-rotate / M_SQRT2, -rotate / M_SQRT2);
                SW2.upd( rotate / M_SQRT2, -rotate / M_SQRT2);
                SW3.upd( rotate / M_SQRT2,  rotate / M_SQRT2);
                SW4.upd(-rotate / M_SQRT2,  rotate / M_SQRT2);
            } else {
                zero_count ++;
            }
        } else {
            zero_count = 0;
            // Servo 1, rotate v = (-1, -1)
            auto [x1, y1] = calc({-rotate / M_SQRT2, -rotate / M_SQRT2}, {vx, vy});
            // Servo 2, rotate v = ( 1, -1)
            auto [x2, y2] = calc({ rotate / M_SQRT2, -rotate / M_SQRT2}, {vx, vy});
            // Servo 3, rotate v = ( 1,  1)
            auto [x3, y3] = calc({ rotate / M_SQRT2,  rotate / M_SQRT2}, {vx, vy});
            // Servo 4, rotate v = (-1,  1)
            auto [x4, y4] = calc({-rotate / M_SQRT2,  rotate / M_SQRT2}, {vx, vy});

            SW1.upd(x1, y1); SW2.upd(x2, y2); SW3.upd(x3, y3); SW4.upd(x4, y4);
        }

        app_msg_vofa_send(E_UART_DEBUG,{
            S1.target,
            S2.target,
            S3.target,
            S4.target,
            INS::data()->yaw
        });

        OS::Task::SleepMilliseconds(1);
    }
}

void app_chassis_init() {
    S1.use_degree_angle = S2.use_degree_angle = S3.use_degree_angle = S4.use_degree_angle = true;
    S1.encoder_zero = 6715;
    S2.encoder_zero = 4776;
    S3.encoder_zero = 2765;
    S4.encoder_zero = 5514;

    W3.reverse = W4.reverse = true;

    SW1.upd(-eps / M_SQRT2, -eps / M_SQRT2);
    SW2.upd( eps / M_SQRT2, -eps / M_SQRT2);
    SW3.upd( eps / M_SQRT2,  eps / M_SQRT2);
    SW4.upd(-eps / M_SQRT2,  eps / M_SQRT2);

    S1.relax(); S2.relax(); S3.relax(); S4.relax(); W1.relax(); W2.relax(); W3.relax(); W4.relax();
    S1.init(); S2.init(); S3.init(); S4.init(); W1.init(); W2.init(); W3.init(); W4.init();
}

#endif
