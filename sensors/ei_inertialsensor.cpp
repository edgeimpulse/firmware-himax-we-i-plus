/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_inertialsensor.h"

#include "edge-impulse-sdk/porting/ei_logging.h"

#include "hx_drv_tflm.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_4G_TO_MS2    4.903325f

/* Private function prototypes --------------------------------------------- */
static float convert_raw_to_ms2(float axis);

/* Private variables ------------------------------------------------------- */
static float imu_data[INERTIAL_AXIS_SAMPLED];

/**
 * @brief      Setup SPI config and accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_inertial_init(void)
{
    hx_drv_accelerometer_initial();

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        EI_LOGE("Failed to register Inertial sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_inertial_read_data(int n_samples)
{
    float x, y, z;
    hx_drv_accelerometer_receive(&x, &y, &z);

    imu_data[0] = convert_raw_to_ms2(x);
    imu_data[1] = convert_raw_to_ms2(y);
    imu_data[2] = convert_raw_to_ms2(z);

    return imu_data;
}

/* Static functions -------------------------------------------------------- */

/**
 * @brief      Convert raw accelerometer axis data in 4g to ms2
 *
 * @param[in]  axis   The axis
 *
 * @return     Return mg value
 */
static float convert_raw_to_ms2(float axis)
{
    return((float)axis * CONVERT_4G_TO_MS2);
}
