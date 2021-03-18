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

#ifndef EI_CAMERA
#define EI_CAMERA

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "hx_drv_tflm.h"
#include "ei_classifier_porting.h"

/* Constants --------------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           640
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           480

/* Externally linked variables --------------------------------------------- */
extern hx_drv_sensor_image_config_t g_pimg_config;

/* Public function prototypes ---------------------------------------------- */
extern bool ei_camera_init(void);
extern void ei_camera_deinit(void);
extern bool ei_camera_capture(uint32_t img_width, uint32_t img_height, int8_t *buf);
extern bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height, bool use_max_baudrate);
extern bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height, bool use_max_baudrate);

/**
 * @brief      Retrieves (cut-out) float RGB image data from the frame buffer
 *
 * @param[in]  offset        offset within cut-out image
 * @param[in]  length        number of bytes to read
 * @param[int] out_ptr       pointer to output buffre
 *
 * @retval     0 if successful
 *
 * @note       This function is called by the classifier to get float RGB image data
 */
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr);

#endif // EI_CAMERA
