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
#include "edge-impulse-sdk/dsp/image/image.hpp"

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

/* Global variables ------------------------------------------------------- */
hx_drv_sensor_image_config_t g_pimg_config;

/* Private variables ------------------------------------------------------- */
static bool is_initialised = false;
/*
** @brief used to store the raw frame
*/
static uint8_t *ei_camera_frame_buffer;

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;

/* Functions Prototypes --------------------------------------------------- */

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Determine whether to resize and to which dimension
 *
 * @param[in]  out_width     width of output image
 * @param[in]  out_height    height of output image
 * @param[out] resize_col_sz       pointer to frame buffer's column/width value
 * @param[out] resize_row_sz       pointer to frame buffer's rows/height value
 * @param[out] do_resize     returns whether to resize (or not)
 *
 */
static int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize)
{
    const ei_device_resize_resolutions_t *list;
    size_t list_size;

    int dl = EiDevice.get_resize_list((const ei_device_resize_resolutions_t **)&list, &list_size);
    if (dl) { /* apparently false is OK here?! */
        ei_printf("ERR: Device has no image resize feature\n");
        return 1;
    }

    // (default) conditions
    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }

    return 0;
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


static bool prepare_snapshot(size_t width, size_t height, bool use_max_baudrate)
{
    if (!verify_inputs(width, height)) { return false; }

    if (ei_camera_init() == false) {
        ei_printf("ERR: Failed to initialize image sensor\r\n");
        return false;
    }

    // setup data output baudrate
    if (use_max_baudrate) {

        // sleep a little to let the daemon attach on the new baud rate...
        ei_printf("OK\r\n");
        ei_sleep(100);

        EiDevice.set_max_data_output_baudrate();
    }

    return true;
}

static inline void finish_snapshot()
{
    // lower baud rate
    ei_printf("OK\r\n");
    EiDevice.set_default_data_output_baudrate();

    // sleep a little to let the daemon attach on baud rate 115200 again...
    ei_sleep(100);

    ei_camera_capture_out = NULL;
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
 * @note       Expects the camera and `ei_camera_frame_buffer` buffer to be
 * initialised
 */
static bool take_snapshot(size_t width, size_t height)
{
    // passing NULL means internal camera buffers will be used
    if (ei_camera_capture(width, height, (int8_t *) NULL) == false) {
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
        ei_free(signal_buf);
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
                    return false;
                }

                int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, base64_output_size);
                if (r < 0) {
                    ei_printf("ERR: Failed to base64 encode (%d)\n", r);
                    ei_free(signal_buf);
                    ei_free(per_pixel_buffer);
                    return false;
                }

                ei_write_string(base64_buffer, r);
                ei_free(base64_buffer);
                per_pixel_buffer_ix = 0;
            }
            EiDevice.set_state(eiStateUploading);
        }
    }

    const size_t new_base64_buffer_output_size = floor(per_pixel_buffer_ix / 3 * 4) + 4;
    char *base64_buffer = (char*)ei_malloc(new_base64_buffer_output_size);
    if (!base64_buffer) {
        ei_free(signal_buf);
        ei_free(per_pixel_buffer);
        ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\n", new_base64_buffer_output_size);
        return false;
    }

    int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, new_base64_buffer_output_size);
    if (r < 0) {
        ei_printf("ERR: Failed to base64 encode (%d)\n", r);
        ei_free(signal_buf);
        ei_free(per_pixel_buffer);
        return false;
    }

    ei_write_string(base64_buffer, r);
    ei_printf("\r\n");

    ei_free(base64_buffer);
    ei_free(signal_buf);
    ei_free(per_pixel_buffer);
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

    ei_camera_frame_buffer = (uint8_t*)g_pimg_config.raw_address;

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
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           when full resolution is expected.
 *
 * @retval     false if not initialised, image captured or rescaled failed
 *
 * @note       original capture is 640x480
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, int8_t *out_buf)
{
    bool do_resize = false;
    bool do_crop = false;

    if (!is_initialised || (ei_camera_frame_buffer == NULL)) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    EiDevice.set_state(eiStateSampling);

    int snapshot_response = hx_drv_sensor_capture(&g_pimg_config);
    if (snapshot_response != HX_DRV_LIB_PASS) {
        ei_printf("ERR: Failed to get snapshot (%d)\r\n", snapshot_response);
        return false;
    }

    uint32_t resize_col_sz;
    uint32_t resize_row_sz;
    // determine resize dimensions
    int res = calculate_resize_dimensions(img_width, img_height, &resize_col_sz, &resize_row_sz, &do_resize);
    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        return false;
    }

    if ((img_width != resize_col_sz)
        || (img_height != resize_row_sz)) {
        do_crop = true;
    }

    // The following variables should always be assigned
    // if this routine is to return true
    // cutout values
    ei_camera_capture_out = ei_camera_frame_buffer;
    //ei_printf("frame buffer: 0x%08x\r\n", ei_camera_frame_buffer);

    if (do_resize) {

        // if only resizing then and out_buf provided then use it instead.
        if (out_buf && !do_crop) ei_camera_capture_out = (uint8_t*) out_buf;

        //ei_printf("resize cols: %d, rows: %d\r\n", resize_col_sz,resize_row_sz);
        ei::image::processing::resize_image(
            ei_camera_frame_buffer,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            ei_camera_capture_out,
            resize_col_sz,
            resize_row_sz,
            1); // bytes per pixel
    }

    if (do_crop) {
        uint32_t crop_col_sz;
        uint32_t crop_row_sz;
        uint32_t crop_col_start;
        uint32_t crop_row_start;
        crop_row_start = (resize_row_sz - img_height) / 2;
        crop_col_start = (resize_col_sz - img_width) / 2;
        crop_col_sz = img_width;
        crop_row_sz = img_height;

        // if (also) cropping and out_buf provided then use it instead.
        if (out_buf) ei_camera_capture_out = (uint8_t*) out_buf;

        //ei_printf("crop cols: %d, rows: %d\r\n", crop_col_sz,crop_row_sz);
        ei::image::processing::cropImage(
            ei_camera_frame_buffer,
            resize_col_sz,
            resize_row_sz,
            crop_col_start,
            crop_row_start,
            ei_camera_capture_out,
            crop_col_sz,
            crop_row_sz,
            8); // bits per pixel
    }

    //ei_printf("camera_out buf: 0x%08x\r\n", ei_camera_capture_out);
    EiDevice.set_state(eiStateIdle);

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
bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height, bool use_max_baudrate)
{
    bool result = true;

    if (!prepare_snapshot(width, height, use_max_baudrate)) result = false;

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
bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height, bool use_max_baudrate)
{
    bool result = true;

    ei_printf("Starting snapshot stream...\n");

    if (!prepare_snapshot(width, height, use_max_baudrate)) result = false;

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
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {

        // grab the value and convert to r/g/b
        uint8_t pixel = ei_camera_capture_out[offset];

        uint8_t r, g, b;
        r = g = b = pixel;

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
