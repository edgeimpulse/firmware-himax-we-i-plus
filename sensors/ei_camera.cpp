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

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "ei_camera.h"
#include "ei_device_himax.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

/* Global variables ------------------------------------------------------- */
hx_drv_sensor_image_config_t g_pimg_config;

/* Private variables ------------------------------------------------------- */
static bool is_initialised = false;

/*
** @brief used to store the raw frame
*/
static uint8_t *ei_camera_frame_buffer;

ei_device_snapshot_resolutions_t EiCameraHimax::resolutions[] = {
        { .width = 640, .height = 480 }
    };

EiCameraHimax::EiCameraHimax()
{
}


bool EiCameraHimax::is_camera_present(void)
{
    return true;
}

ei_device_snapshot_resolutions_t EiCameraHimax::get_min_resolution(void) {
    return resolutions[0];
}

void EiCameraHimax::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) {

    *res = &EiCameraHimax::resolutions[0];
    *res_num = sizeof(EiCameraHimax::resolutions) / sizeof(ei_device_snapshot_resolutions_t);

}

bool EiCameraHimax::set_resolution(const ei_device_snapshot_resolutions_t res) {

    // the driver does not support setting res
    // contrary to what it says in the docs
    //g_pimg_config.img_width = res.width;
    //g_pimg_config.img_height = res.height;

    return true;
}

bool EiCameraHimax::init(uint16_t width, uint16_t height)
{
    if (is_initialised == true) return true;

    ei_device_snapshot_resolutions_t res = search_resolution(width, height);
    set_resolution(res);

    if (hx_drv_sensor_initial(&g_pimg_config) != HX_DRV_LIB_PASS) {
        return false;
    }

    if(hx_drv_spim_init() != HX_DRV_LIB_PASS) {
        return false;
    }

    ei_camera_frame_buffer = (uint8_t*)g_pimg_config.raw_address;

    is_initialised = true;

    return true;
}

bool EiCameraHimax::deinit()
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    if(is_initialised) {
        hx_drv_sensor_stop_capture();
        is_initialised = false;
    }
    dev->set_state(eiStateFinished);
    return true;
}

 bool EiCameraHimax::get_fb_ptr(uint8_t** fb_ptr)
{
    if (!is_initialised || (ei_camera_frame_buffer == NULL)) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }
    *fb_ptr = ei_camera_frame_buffer;
    return true;
}

bool EiCameraHimax::ei_camera_capture_grayscale_packed_big_endian(
    uint8_t *image,
    uint32_t image_size)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();

    if (!is_initialised || (ei_camera_frame_buffer == NULL)) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }
    dev->set_state(eiStateSampling);

    int snapshot_response = hx_drv_sensor_capture(&g_pimg_config);
    if (snapshot_response != HX_DRV_LIB_PASS) {
        ei_printf("ERR: Failed to get snapshot (%d)\r\n", snapshot_response);
        return false;
    }

    dev->set_state(eiStateUploading);
    return true;
}

EiCamera *EiCamera::get_camera()
{
    static EiCameraHimax camera;
    return &camera;
}

// AT+RUNIMPULSE
// AT+RUNIMPULSEDEBUG=n