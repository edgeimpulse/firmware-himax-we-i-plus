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