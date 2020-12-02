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

/* Global variables -------------------------------------------------------- */
extern hx_drv_sensor_image_config_t g_pimg_config;


/* Constants --------------------------------------------------------------- */
// raw frame buffer from the camera
#define FRAME_BUFFER_COLS           640
#define FRAME_BUFFER_ROWS           480

/* Function prototypes ----------------------------------------------------- */
extern bool ei_camera_init(void);
extern void ei_camera_deinit(void);
extern bool ei_camera_capture(uint32_t img_width, uint32_t img_height, int8_t *buf);
extern bool ei_camera_take_snapshot(size_t width, size_t height);


static int8_t *snapshot_image_data = NULL;
static bool snapshot_is_resized = false;

static uint32_t frame_buffer_cols;
static uint32_t frame_buffer_rows;
static uint32_t cutout_row_start;
static uint32_t cutout_col_start;

static uint32_t cutout_cols;
static uint32_t cutout_rows;

/* Helper functions -------------------------------------------------------- */
static int get_snapshot_image_data(size_t offset, size_t length, float *out_ptr) {
    for(size_t i = 0; i < length; i++) {
        int8_t mono_data = (int8_t)snapshot_image_data[offset + i];
        uint8_t v;
        if (snapshot_is_resized) {
            v = (uint8_t)mono_data + 128;
        }
        else {
            v = (uint8_t)mono_data;
        }
        out_ptr[i] = (float)((v << 16) | (v << 8) | (v));
    }

    return 0;
}

static void mono_to_rgb(uint8_t mono_data, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t v;
    v = (snapshot_is_resized) ? mono_data + 128 : mono_data;
    *r = *g = *b = v;
}

/**
 * This function is called by the classifier to get data
 * We don't want to have a separate copy of the cutout here, so we'll read from the frame buffer dynamically
 */
static int cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    // so offset and length naturally operate on the *cutout*, so we need to cut it out from the real framebuffer
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {
        // find location of the byte in the cutout
        size_t cutout_row = floor(offset / cutout_cols);
        size_t cutout_col = offset - (cutout_row * cutout_cols);

        // then read the value from the real frame buffer
        size_t frame_buffer_row = cutout_row + cutout_row_start;
        size_t frame_buffer_col = cutout_col + cutout_col_start;

        // grab the value and convert to r/g/b
        uint8_t pixel = (uint8_t) snapshot_image_data[(frame_buffer_row * frame_buffer_cols) + frame_buffer_col];

        uint8_t r, g, b;
        mono_to_rgb(pixel, &r, &g, &b);

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        offset++;
        bytes_left--;
    }

    // and done!
    return 0;
}


#endif // EI_CAMERA
