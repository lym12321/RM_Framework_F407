//
// Created by fish on 2024/11/16.
//

#include "app_ins.h"

#include "alg_pid.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"
#include "sys_task.h"
#include "bsp_adc.h"
#include "alg_mahony.h"
#include "alg_quaternion_ekf.h"
#include "bsp_def.h"

#include "tim.h"

// 温度稳定计数
#define TEMPERATURE_COUNT 100

// 误差观测计数
#define ERR_TEST_COUNT 1500

// 观测误差阈值，若误差过大则使用 gyro_std_err，从而避免不受限制的误差积累
#define GYRO_ERR_THRESHOLD 0.01

// 标准误差，需要在静止状态下观测较长时间得到，且通常环境变化或传感器变化后需要重新观测
const float gyro_std_err[3] = { 0.00342450268, -0.00207025907, -0.0056360052 };

#define IMU_TEMPERATURE_CONTROL_TIMER &htim10
#define IMU_TEMPERATURE_CONTROL_CHANNEL TIM_CHANNEL_1

using namespace INS;

namespace INS {
    ins_data_t data_;
    OS::Task ins_task;
    Algorithm::PID temp_pid(Algorithm::PID::POSITION);
    // 这里跳过温度检测，在未达到指定温度的情况下直接开始算 err，会快一点
    // 后面考虑加校准功能，把参数存 flash 里。
    uint8_t imu_state = 1;
    bool inited_ = false;

    void task(void *args) {
        uint16_t count = 0, freq_cnt = 0;
        float gyro_err[3] = {0, 0, 0};
        while(true) {
            bsp_imu_raw_data_t *raw_data = bsp_imu_read();
            data_.imu_temp = raw_data->temp;

            if(++freq_cnt == 100) __HAL_TIM_SetCompare(IMU_TEMPERATURE_CONTROL_TIMER, IMU_TEMPERATURE_CONTROL_CHANNEL,
                std::max(0.0, temp_pid.update(data_.imu_temp, 40))), freq_cnt = 0;

            if(imu_state == 0) {
                count += (std::abs(data_.imu_temp - 40) < 0.5f);
                if(count == TEMPERATURE_COUNT) imu_state = 1, count = 0;
            } else if(imu_state == 1) {
                count ++;
                gyro_err[0] += raw_data->gyro[0], gyro_err[1] += raw_data->gyro[1], gyro_err[2] += raw_data->gyro[2];
                if(count == ERR_TEST_COUNT) {
                    gyro_err[0] /= (float) count, gyro_err[1] /= (float) count, gyro_err[2] /= (float) count;
                    if(std::abs(gyro_err[0]) > GYRO_ERR_THRESHOLD or std::abs(gyro_err[1]) > GYRO_ERR_THRESHOLD or std::abs(gyro_err[2]) > GYRO_ERR_THRESHOLD) {
                        memcpy(gyro_err, gyro_std_err, sizeof gyro_err);
                    }
                    imu_state = 2, count = 0;
                }
            } else if(imu_state == 2) {
                float gyro[3] = {
                    raw_data->gyro[0] - gyro_err[0],
                    raw_data->gyro[1] - gyro_err[1],
                    raw_data->gyro[2] - gyro_err[2]
                };
                // CHEAT: 忽略极小的变化，借此尽可能消除零漂
                if(std::abs(gyro[2]) < 0.1f) gyro[2] = 0;
                IMU_QuaternionEKF_Update(
                    gyro[0], gyro[1], gyro[2],
                    raw_data->accel[0], raw_data->accel[1], raw_data->accel[2]
                );
                Mahony_update(
                    gyro[0], gyro[1], gyro[2],
                    raw_data->accel[0], raw_data->accel[1], raw_data->accel[2],
                    0, 0, 0
                );
                data_.pitch = Get_Pitch(), data_.roll = Get_Roll(), data_.yaw = Get_Yaw();
            }
            osDelay(1);
        }
    }

    void init() {
        bsp_imu_init();
        // BSP_ASSERT(bsp_adc_vbus() > 0);
        // if(bsp_adc_vbus() > 22) {
            // temp_pid.set_para(100, 1, 0, 10000, 500);
        // } else if(bsp_adc_vbus() > 10) {
            // temp_pid.set_para(500, 2, 1, 10000, 1000);
        // } else {
            // temp_pid.set_para(4500, 5, 1, 10000, 5000);
        // }
        temp_pid.set_para(4500, 5, 1, 10000, 5000);
        HAL_TIM_PWM_Start(IMU_TEMPERATURE_CONTROL_TIMER, IMU_TEMPERATURE_CONTROL_CHANNEL);
        Mahony_Init(1000);
        IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);

        inited_ = true;
    }

    bool ready() { return imu_state == 2; }

    ins_data_t *data() { return &data_; }
}

void app_ins_task(void *argument) {
    while(!inited_) OS::Task::SleepMilliseconds(10);
    task(argument);
}
