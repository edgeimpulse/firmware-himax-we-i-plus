/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
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

#include "ei_config_types.h"
#include "ei_inertialsensor.h"
#include "ei_device_himax.h"
#include "sensor_aq.h"

#include "hx_drv_tflm.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_4G_TO_MS2    4.903325f

#define ACC_SAMPLE_TIME_MS  (1.f / 119.f)
#define FLASH_WRITE_TIME_MS 0.1f

extern ei_config_t *ei_config_get_config();
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

/* Private function prototypes --------------------------------------------- */
static float convert_raw_to_ms2(float axis);

/* Private variables ------------------------------------------------------- */
static uint32_t samplerate_divider;
static float imu_data[N_AXIS_SAMPLED];
static float scale_and_ms2_convert;

// static tIcm20602Cfg eiIcm20602Device;

sampler_callback  cb_sampler;


/**
 * @brief      Setup SPI config and accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_inertial_init(void)
{
    // eiIcm20602Device.iSpiInstance = (tSpiNum)ETA_BSP_ICM20602_SPI_NUM;
    // eiIcm20602Device.iSpiChipSel = ETA_BSP_ICM20602_SPI_CS_NUM;

    hx_drv_accelerometer_initial();

    scale_and_ms2_convert = 1.f;//ei_inertial_accel_sensitivity() * CONVERT_G_TO_MS2;

    if(scale_and_ms2_convert == 0.f) {
        return false;
    }
    else {
        return true;
    }
}

/**
 * @brief      Get data from sensor, convert and call callback to handle
 */
void ei_inertial_read_data(void)
{       
    float x, y, z;
    // volatile uint32_t div_sample_count;
    
    // for(div_sample_count = 0; div_sample_count < samplerate_divider; div_sample_count++) {        

        // while(hx_drv_accelerometer_available_count() == 0) {
        //     EiDevice.delay_ms(1);
        // };
        EiDevice.delay_ms(6);
        while(hx_drv_accelerometer_available_count() == 0) {};
            
        hx_drv_accelerometer_receive(&x, &y, &z);
        //}
    // }

    imu_data[0] = convert_raw_to_ms2(x);
    imu_data[1] = convert_raw_to_ms2(y);
    imu_data[2] = convert_raw_to_ms2(z);

    cb_sampler((const void *)&imu_data[0], SIZEOF_N_AXIS_SAMPLED);
}

/**
 * @brief      Setup timing and data handle callback function
 *
 * @param[in]  callsampler         Function to handle the sampled data
 * @param[in]  sample_interval_ms  The sample interval milliseconds
 *
 * @return     true
 */
bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    cb_sampler = callsampler;
    
    samplerate_divider = (int)(sample_interval_ms / ACC_SAMPLE_TIME_MS) - (FLASH_WRITE_TIME_MS / ACC_SAMPLE_TIME_MS);

    EiDevice.set_state(eiStateSampling);

    return true;
}

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_inertial_setup_data_sampling(void)
{

    if (ei_config_get_config()->sample_interval_ms < 0.001f) {
        ei_config_set_sample_interval(1.f/62.5f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        EiDevice.get_id_pointer(),
        // Device type (required), use the same device type for similar devices
        EiDevice.get_type_pointer(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" }, 
        /*{ "gyrX", "dps" }, { "gyrY", "dps" }, { "gyrZ", "dps" } */},        
    };	
    
    EiDevice.set_state(eiStateErasingFlash);
    ei_sampler_start_sampling(&payload, SIZEOF_N_AXIS_SAMPLED);
    EiDevice.set_state(eiStateIdle);

    return true;
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