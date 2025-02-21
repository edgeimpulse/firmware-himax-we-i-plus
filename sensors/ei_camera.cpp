/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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