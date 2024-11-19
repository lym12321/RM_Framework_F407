//
// Created by fish on 2024/11/16.
//

#pragma once

namespace INS {
    struct ins_data_t {
        float yaw, pitch, roll, imu_temp;
    };
    ins_data_t *data();
    void init();
    bool ready();
}

#ifdef __cplusplus
extern "C" {
#endif

void app_ins_task(void *argument);

#ifdef __cplusplus
}
#endif