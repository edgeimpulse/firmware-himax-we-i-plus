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

#ifndef EI_CAMERA
#define EI_CAMERA

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_camera_interface.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "hx_drv_tflm.h"

/* Constants --------------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           640
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           480

/* Externally linked variables --------------------------------------------- */
extern hx_drv_sensor_image_config_t g_pimg_config;

class EiCameraHimax : public EiCamera {
private:

    static ei_device_snapshot_resolutions_t resolutions[];

    uint32_t width;
    uint32_t height;
    uint32_t output_width;
    uint32_t output_height;

    bool camera_present;

public:
    EiCameraHimax();
    bool init(uint16_t width, uint16_t height);
    bool deinit();
    bool ei_camera_capture_grayscale_packed_big_endian(uint8_t *image, uint32_t image_size);
    bool set_resolution(const ei_device_snapshot_resolutions_t res);
    ei_device_snapshot_resolutions_t get_min_resolution(void);
    bool is_camera_present(void);
    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num);
    bool get_fb_ptr(uint8_t**);
};

#endif