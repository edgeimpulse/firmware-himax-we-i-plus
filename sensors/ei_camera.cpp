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
#include "ei_camera.h"
#include "ei_device_himax.h"
#include "ei_himax_fs_commands.h"
#include "ei_classifier_porting.h"
#include "at_base64.h"
#include "numpy_types.h"


/* Global variables ------------------------------------------------------- */
hx_drv_sensor_image_config_t g_pimg_config;

/* Private variables ------------------------------------------------------- */
static bool is_initialised = false;

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Calculate the desired frame buffer resolution
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[out] fb_cols       pointer to frame buffer's column/width value
 * @param[out] fb_rows       pointer to frame buffer's rows/height value
 *
 */
static void calculate_rescaled_fb_resolution (uint32_t img_width, uint32_t img_height, uint32_t *fb_cols, uint32_t *fb_rows)
{

    const ei_device_snapshot_resolutions_t *list;
    size_t list_size;

    int dl = EiDevice.get_snapshot_list((const ei_device_snapshot_resolutions_t **)&list, &list_size);
    if (dl) { /* apparently false is OK here?! */
        ei_printf("ERR: Device has no snapshot feature\n");
        return;
    }

    uint32_t fb_width;
    uint32_t fb_height;
    bool fb_resolution_found = false;
    for (size_t ix = 0; ix < list_size; ix++) {
        if ((img_width <= list[ix].width) || (img_height <= list[ix].height)) {
            fb_width  = list[ix].width;
            fb_height = list[ix].height;
            fb_resolution_found = true;
            break;
        }
    }

    if (fb_resolution_found) {
        *fb_cols = fb_width;
        *fb_rows = fb_height;
    } else {
        *fb_cols = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
        *fb_rows = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    }
}

static bool verify_inputs(size_t width, size_t height)
{
    const ei_device_snapshot_resolutions_t *list;
    size_t list_size;

    int dl = EiDevice.get_snapshot_list((const ei_device_snapshot_resolutions_t **)&list, &list_size);
    if (dl) { /* apparently false is OK here?! */
        ei_printf("ERR: Device has no snapshot feature\n");
        return false;
    }

    bool found_res = false;
    for (size_t ix = 0; ix < list_size; ix++) {
        if (list[ix].width == width && list[ix].height == height) {
            found_res = true;
        }
    }

    if (!found_res) {
        ei_printf("ERR: Invalid resolution %lux%lu\n", width, height);
        return false;
    }

    ei_printf("\tImage resolution: %dx%d\n", width, height);

    return true;
}


static bool prepare_snapshot(size_t width, size_t height)
{
    if (!verify_inputs(width, height)) { return false; }

    if (ei_camera_init() == false) {
        ei_printf("ERR: Failed to initialize image sensor\r\n");
        return false;
    }

    // must be called after ei_camera_init()
    ei_camera_snapshot_image_data = (int8_t*)ei_himax_fs_allocate_sampledata(width * height);
    if (!ei_camera_snapshot_image_data) {
        ei_printf("ERR: Failed to allocate image buffer for (%dx%d): 0x%x\n", width, height, ei_camera_snapshot_image_data);
        return false;
    }

    return true;
}

static inline void finish_snapshot()
{
    ei_camera_snapshot_image_data = NULL;
    ei_camera_deinit();
}

/**
 * @brief      Helper function: Takes a snapshot, base64 encodes and prints it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  buf           pointer to store output image
 *
 * @retval     bool
 *
 * @note       Expects the camera and `ei_camera_snapshot_image_data` buffer to be
 * initialised
 */
static bool take_snapshot(size_t width, size_t height)
{
    if (ei_camera_capture(width, height, ei_camera_snapshot_image_data) == false) {
        ei_printf("ERR: Failed to capture image\r\n");
        return false;
    }

    ei::signal_t signal;
    signal.total_length = width * height;
    signal.get_data = &ei_camera_cutout_get_data;

    size_t signal_chunk_size = 1024;

    // loop through the signal
    float *signal_buf = (float*)ei_malloc(signal_chunk_size * sizeof(float));
    if (!signal_buf) {
        ei_printf("ERR: Failed to allocate signal buffer\n");
        return false;
    }

    uint8_t *per_pixel_buffer = (uint8_t*)ei_malloc(513); // 171 x 3 pixels
    if (!per_pixel_buffer) {
        free(signal_buf);
        ei_printf("ERR: Failed to allocate per_pixel buffer\n");
        return false;
    }

    size_t per_pixel_buffer_ix = 0;

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

        // setup high buadrate
        hx_drv_uart_initial(UART_BR_921600);

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
                    free(signal_buf);
                    return false;
                }

                int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, base64_output_size);
                free(base64_buffer);

                if (r < 0) {
                    ei_printf("ERR: Failed to base64 encode (%d)\n", r);
                    free(signal_buf);
                    return false;
                }

                ei_write_string(base64_buffer, r);
                per_pixel_buffer_ix = 0;
            }
            EiDevice.set_state(eiStateUploading);
        }

        // lower buadrate
        ei_printf("\r\nOK");
        hx_drv_uart_initial(UART_BR_115200);
    }

    const size_t new_base64_buffer_output_size = floor(per_pixel_buffer_ix / 3 * 4) + 4;
    char *base64_buffer = (char*)ei_malloc(new_base64_buffer_output_size);
    if (!base64_buffer) {
        ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\n", new_base64_buffer_output_size);
        return false;
    }

    int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, new_base64_buffer_output_size);
    free(base64_buffer);
    if (r < 0) {
        ei_printf("ERR: Failed to base64 encode (%d)\n", r);
        return false;
    }

    ei_write_string(base64_buffer, r);
    ei_printf("\r\n");

    free(signal_buf);
    EiDevice.set_state(eiStateIdle);

    return true;
}



/* Public functions -------------------------------------------------------- */

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void)
{
    if (is_initialised == true) return true;

    if (hx_drv_sensor_initial(&g_pimg_config) != HX_DRV_LIB_PASS) {
        return false;
    }

    if(hx_drv_spim_init() != HX_DRV_LIB_PASS) {
        return false;
    }

    is_initialised = true;

    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void)
{
    if(is_initialised) {

        hx_drv_sensor_stop_capture();
        is_initialised = false;
    }
}


/**
 * @brief      Capture and rescale image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  buf           pointer to store output image
 *
 * @retval     false if not initialised, image captured or rescaled failed
 *
 * @note       original capture is 640x480
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, int8_t *buf)
{
    if (is_initialised == false) return false;

    EiDevice.set_state(eiStateSampling);

    if (hx_drv_sensor_capture(&g_pimg_config) != HX_DRV_LIB_PASS) {
        return false;
    }

    // determine what the scaled output image buffer size should be
    calculate_rescaled_fb_resolution(img_width, img_height, &ei_camera_frame_buffer_cols, &ei_camera_frame_buffer_rows);

    // The following variables should always be assigned
    // if this routine is to return true
    ei_camera_cutout_row_start = (ei_camera_frame_buffer_rows - img_height) / 2;
    ei_camera_cutout_col_start = (ei_camera_frame_buffer_cols - img_width) / 2;
    ei_camera_cutout_cols = img_width;
    ei_camera_cutout_rows = img_height;
    ei_camera_snapshot_is_resized = (ei_camera_frame_buffer_cols != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (ei_camera_frame_buffer_rows != EI_CAMERA_RAW_FRAME_BUFFER_ROWS);

    //  skip scaling if frame buffer's width and height matches the original resolution
    if ((ei_camera_frame_buffer_cols == EI_CAMERA_RAW_FRAME_BUFFER_COLS) && (ei_camera_frame_buffer_rows == EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) return true;

    if (hx_drv_image_rescale((uint8_t*)g_pimg_config.raw_address,
                             g_pimg_config.img_width, g_pimg_config.img_height,
                             buf, ei_camera_frame_buffer_cols, ei_camera_frame_buffer_rows) != HX_DRV_LIB_PASS) {
        return false;
    }

    return true;
}

/**
 * @brief      Takes a snapshot, base64 encodes and outputs it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 *
 * @retval     true if snapshot was taken successfully
 *
 */
bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height)
{
    bool result = true;

    if (!prepare_snapshot(width, height)) result = false;

    if (result) {
        if (!take_snapshot(width, height)) {
            result = false;
        }
    }

    finish_snapshot();

    return result;
}

/**
 * @brief      Starts a snapshot stream, base64 encodes and outputs it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 *
 * @retval     true if successful and/or terminated gracefully
 *
 */
bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height)
{
    bool result = true;

    ei_printf("Starting snapshot stream...\n");

    if (!prepare_snapshot(width, height)) result = false;

    while (result) {
        if (ei_user_invoke_stop()) {
            ei_printf("Snapshot streaming stopped by user\r\n");
            EiDevice.set_state(eiStateIdle);
            break;
        }
        if (!take_snapshot(width, height)) {
            result = false;
        }
    }

    finish_snapshot();

    return result;
}

