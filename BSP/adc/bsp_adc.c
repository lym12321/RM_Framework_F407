//
// Created by fish on 2024/10/6.
//

#include "bsp_adc.h"

#include "adc.h"

#include "stdint.h"

/*
 *  bsp_adc
 *  读 vbus 电压用
 *  从 dm-mc02 搬过来的，C 板的这部分还没写。
 */

static uint16_t val[2];

void bsp_adc_init(void) {
    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *) val, 2);
}

float bsp_adc_vbus(void) {
    return (float) val[0] * 3.3f / 65535 * 11.0f;
}