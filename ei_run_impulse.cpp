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
#include <vector>
#include <math.h>
#include "ei_device_himax.h"
#include "ei_himax_fs_commands.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_microphone.h"
#include "ei_inertialsensor.h"
#include "ei_camera.h"
#include "hx_drv_tflm.h"
#include "bitmap_helpers.h"
#include "at_base64.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/classifier/ei_aligned_malloc.h"
#include "model-parameters/model_metadata.h"
#include "tflite-model/20210910_134700.int8_io_int8.cpp.h"
#include "edge-impulse-sdk/classifier/ei_run_dsp.h"


typedef struct cube {
    size_t col;
    size_t row;
} cube_t;

void run_nn(bool debug) {

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

    TfLiteStatus status = model_init(ei_aligned_calloc);
    if (status != kTfLiteOk) {
        ei_printf("Failed to allocate TFLite arena (error code %d)\n", status);
        return;
    }

    TfLiteTensor *input_tensor = model_input(0);
    TfLiteTensor *output_tensor = model_output(0);
    if (!input_tensor || !output_tensor) {
        ei_printf("Failed to get input/output tensor\n");
        return;
    }

    if (input_tensor->dims->size != 4) {
        ei_printf("Invalid input_tensor dimensions, expected 4 but got %d\n", (int)input_tensor->dims->size);
        return;
    }

    if (input_tensor->dims->data[3] != 1) {
        ei_printf("Invalid input_tensor dimensions, expected 1 channel but got %d\n", (int)input_tensor->dims->data[3]);
        return;
    }

    int input_img_width = input_tensor->dims->data[1];
    int input_img_height = input_tensor->dims->data[2];

    if (input_img_width * input_img_height != EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT) {
        ei_printf("Invalid number of features, expected %d but received %d\n",
            input_img_width * input_img_height, EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);
        return;
    }

    ei_printf("Input dims size %d, bytes %d\n", (int)input_tensor->dims->size, (int)input_tensor->bytes);
    for (size_t ix = 0; ix < input_tensor->dims->size; ix++) {
        ei_printf("    dim %d: %d\n", (int)ix, (int)input_tensor->dims->data[ix]);
    }

    // one byte per value
    bool is_quantized = input_tensor->bytes == input_img_width * input_img_height;

    ei_printf("Is quantized? %d\n", is_quantized);
    if (!is_quantized) {
        ei_printf("ERR: Only support quantized models\n");
        return;
    }

    if (ei_camera_init() == false) {
        ei_printf("ERR: Failed to initialize image sensor\r\n");
        return;
    }

    int8_t *image_data = (int8_t*)ei_himax_fs_allocate_sampledata(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);
    if (!image_data) {
        ei_printf("ERR: Failed to allocate image buffer\r\n");
        return;
    }

    while(1) {
        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_cutout_get_data;

        uint64_t capture_start = ei_read_timer_ms();

        if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, image_data) == false) {
            ei_printf("Failed to capture image\r\n");
            break;
        }

        uint64_t capture_end = ei_read_timer_ms();

        uint64_t dsp_start = ei_read_timer_ms();

        ei::matrix_i8_t input_matrix(EI_CLASSIFIER_INPUT_HEIGHT, EI_CLASSIFIER_INPUT_WIDTH, input_tensor->data.int8);

        int f = extract_image_features_quantized(&signal, &input_matrix, (void*)&ei_dsp_config_36, 0.0f);
        if (f != ei::EIDSP_OK) {
            ei_printf("ERR: Failed to extract features (%d)\n", f);
            EiDevice.set_state(eiStateIdle);
            break;
        }

        uint64_t dsp_end = ei_read_timer_ms();

        uint64_t nn_start = ei_read_timer_ms();

        status = model_invoke();
        if (status != kTfLiteOk) {
            ei_printf("Failed to invoke model (error code %d)\n", status);
            return;
        }

        uint64_t nn_end = ei_read_timer_ms();

        uint64_t post_start = ei_read_timer_ms();

        std::vector<cube_t> cubes;

        for (size_t row = 0; row < output_tensor->dims->data[1]; row++) {
            // ei_printf("    [ ");
            for (size_t col = 0; col < output_tensor->dims->data[2]; col++) {
                size_t loc = ((row * output_tensor->dims->data[2]) + col) * 2;

                float v1f, v2f;

                if (is_quantized) {
                    int8_t v1 = output_tensor->data.int8[loc+0];
                    int8_t v2 = output_tensor->data.int8[loc+1];

                    float zero_point = output_tensor->params.zero_point;
                    float scale = output_tensor->params.scale;

                    v1f = static_cast<float>(v1 - zero_point) * scale;
                    v2f = static_cast<float>(v2 - zero_point) * scale;
                }
                else {
                    v1f = output_tensor->data.f[loc+0];
                    v2f = output_tensor->data.f[loc+1];
                }

                float v[2] = { v1f, v2f };
                // ei_printf("%f ", v[1]);

                if (v[1] > 0.3f) { // cube
                    cube_t cube = { 0 };
                    cube.row = row;
                    cube.col = col;
                    bool found_overlapping_cube = false;
                    // for (auto other_cube : cubes) {
                    //     if (abs((int)(cube.row - other_cube.row)) <= 1 && abs((int)(cube.col - other_cube.col)) <= 1) {
                    //         // overlapping
                    //         found_overlapping_cube = true;
                    //     }
                    // }
                    if (!found_overlapping_cube) {
                        cubes.push_back(cube);
                    }

                    // ei_printf("1");
                }
                else {
                    // ei_printf("0");
                }

                // ei_printf("%.2f", v[1]);

                // ei_printf("[ %.2f, %.2f ]", v[0], v[1]);
                // ei_printf("[ %f, %f ]", v1f, v2f);
                if (col != output_tensor->dims->data[2] - 1) {
                    // ei_printf(", ");
                }
            }
            // ei_printf("]");
            if (row != output_tensor->dims->data[1] - 1) {
                // ei_printf(", ");
            }
            // ei_printf("\n");
        }
        // ei_printf("]\n")

        uint64_t post_end = ei_read_timer_ms();

        ei_printf("Begin output\n");

        uint64_t framebuffer_start = ei_read_timer_ms();

        if (debug) {
            ei_printf("Framebuffer: ");

            size_t signal_chunk_size = 1024;

            // loop through the signal
            float *signal_buf = (float*)ei_malloc(signal_chunk_size * sizeof(float));
            if (!signal_buf) {
                ei_printf("ERR: Failed to allocate signal buffer\n");
                return;
            }

            uint8_t *per_pixel_buffer = (uint8_t*)ei_malloc(513); // 171 x 3 pixels
            if (!per_pixel_buffer) {
                ei_free(signal_buf);
                ei_printf("ERR: Failed to allocate per_pixel buffer\n");
                return;
            }

            size_t per_pixel_buffer_ix = 0;

            size_t pixel_ix = 0;

            for (size_t ix = 0; ix < signal.total_length; ix += signal_chunk_size) {
                size_t items_to_read = signal_chunk_size;
                if (items_to_read > signal.total_length - ix) {
                    items_to_read = signal.total_length - ix;
                }

                int r = signal.get_data(ix, items_to_read, signal_buf);
                if (r != 0) {
                    ei_printf("ERR: Failed to get data from signal (%d)\n", r);
                    break;
                }

                for (size_t px = 0; px < items_to_read; px++) {
                    uint32_t pixel = static_cast<uint32_t>(signal_buf[px]);

                    // grab rgb
                    uint8_t r = static_cast<float>(pixel >> 16 & 0xff);
                    uint8_t g = static_cast<float>(pixel >> 8 & 0xff);
                    uint8_t b = static_cast<float>(pixel & 0xff);

                    // is monochrome anyway now, so just print 1 pixel at a time
                    const bool print_rgb = false;

                    if (print_rgb) {
                        per_pixel_buffer[per_pixel_buffer_ix + 0] = r;
                        per_pixel_buffer[per_pixel_buffer_ix + 1] = g;
                        per_pixel_buffer[per_pixel_buffer_ix + 2] = b;
                        per_pixel_buffer_ix += 3;
                    }
                    else {
                        per_pixel_buffer[per_pixel_buffer_ix + 0] = r;
                        per_pixel_buffer_ix++;
                    }

                    if (per_pixel_buffer_ix >= 513) {
                        const size_t base64_output_size = 684;

                        char *base64_buffer = (char*)ei_malloc(base64_output_size);
                        if (!base64_buffer) {
                            ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\n", base64_output_size);
                            ei_free(signal_buf);
                            ei_free(per_pixel_buffer);
                            return;
                        }

                        int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, base64_output_size);
                        if (r < 0) {
                            ei_printf("ERR: Failed to base64 encode (%d)\n", r);
                            ei_free(signal_buf);
                            ei_free(per_pixel_buffer);
                            return;
                        }

                        ei_write_string(base64_buffer, r);
                        per_pixel_buffer_ix = 0;
                        ei_free(base64_buffer);
                    }
                    pixel_ix++;
                    EiDevice.set_state(eiStateUploading);
                }
            }

            const size_t new_base64_buffer_output_size = floor(per_pixel_buffer_ix / 3 * 4) + 4;
            char *base64_buffer = (char*)ei_malloc(new_base64_buffer_output_size);
            if (!base64_buffer) {
                ei_free(signal_buf);
                ei_free(per_pixel_buffer);
                ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\n", new_base64_buffer_output_size);
                return;
            }

            int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, new_base64_buffer_output_size);
            if (r < 0) {
                ei_free(signal_buf);
                ei_free(per_pixel_buffer);
                ei_printf("ERR: Failed to base64 encode (%d)\n", r);
                return;
            }

            ei_write_string(base64_buffer, r);
            ei_printf("\r\n");

            ei_free(signal_buf);
            ei_free(per_pixel_buffer);
            ei_free(base64_buffer);

        }

        uint64_t framebuffer_end = ei_read_timer_ms();

        ei_printf("Found %lu cubes (capture=%dms., dsp=%dms., nn=%dms., post=%dms., framebuffer=%dms.)\n", cubes.size(),
            (int)(capture_end - capture_start), (int)(dsp_end - dsp_start), (int)(nn_end - nn_start),
            (int)(post_end - post_start), (int)(framebuffer_end - framebuffer_start));
        for (auto cube : cubes) {
            ei_printf("    At x=%lu, y=%lu\n", cube.col * 8, cube.row * 8);
        }

        ei_printf("End output\n");

        if (ei_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            EiDevice.set_state(eiStateIdle);
            break;
        }
    }


    status = model_reset(ei_aligned_free);
    if (status != kTfLiteOk) {
        ei_printf("Failed to free model (error code %d)\n", status);
        return;
    }

    ei_camera_deinit();
}

void run_nn_normal(void) {
    run_nn(false);
}

void run_nn_debug(void) {
    run_nn(true);
}

void run_nn_continuous_normal()
{
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE
    run_nn_continuous(false);
#else
    ei_printf("Error no continuous classification available for current model\r\n");
#endif
}
