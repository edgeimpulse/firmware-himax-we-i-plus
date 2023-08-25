/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
