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
#include "ei_device_himax.h"
#include "ei_classifier_porting.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"

#include "hx_drv_tflm.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cstdint>
#include <math.h>

/** Memory location for the arduino device address */
#define DEVICE_ID_LSB_ADDR  ((uint32_t)0x100000A4)
#define DEVICE_ID_MSB_ADDR  ((uint32_t)0x100000A8)

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE  32

#define SIZE_RAM_BUFFER			0x20800
#define RAM_BLOCK_SIZE			1024
#define RAM_N_BLOCKS			(SIZE_RAM_BUFFER / RAM_BLOCK_SIZE)

/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE] = {"00:00:00:00:00:00"};

/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    "921600",
    UART_BR_921600,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    "115200",
    UART_BR_115200,
};

/** Global flag */
bool stop_sampling;

/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static int set_id_c(char *device_id);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);
static void timer_callback(void *arg);
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t));
static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate);
static void set_max_data_output_baudrate_c();
static void set_default_data_output_baudrate_c();

/* Public functions -------------------------------------------------------- */

EiDeviceHimax::EiDeviceHimax(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();

    load_config();

    device_type = "HIMAX_WE_I_PLUS";

    cam = static_cast<EiCameraHimax*>(EiCameraHimax::get_camera());
    camera_present = cam->is_camera_present();

    // microphone is not handled by fusion system
    standalone_sensor_list[0].name = "Built-in microphone";
    standalone_sensor_list[0].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * 2);
    standalone_sensor_list[0].frequencies[0] = 16000.0f;
}

EiDeviceHimax::~EiDeviceHimax()
{

}

void EiDeviceHimax::init_device_id(void)
{
    device_id = std::string(ei_device_id);
    mac_address = std::string(ei_device_id);
}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiDeviceRAM<RAM_BLOCK_SIZE, RAM_N_BLOCKS> memory(sizeof(EiConfig));
    static EiDeviceHimax dev(&memory);

    return &dev;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceHimax::delay_ms(uint32_t milliseconds)
{
    ei_sleep(milliseconds);
}

void EiDeviceHimax::set_state(EiState state)
{
    static uint8_t upload_toggle = 0;

    switch(state) {
    case eiStateErasingFlash:
        break;
    case eiStateSampling:
        hx_drv_led_on(HX_DRV_LED_GREEN);
        break;
    case eiStateUploading:
        if(upload_toggle) {
            hx_drv_led_on(HX_DRV_LED_RED);
        }
        else {
            hx_drv_led_off(HX_DRV_LED_RED);
            hx_drv_led_off(HX_DRV_LED_GREEN);
        }
        upload_toggle ^= 1;
        break;
    case eiStateFinished:
        hx_drv_led_off(HX_DRV_LED_GREEN);
        hx_drv_led_off(HX_DRV_LED_RED);
        upload_toggle = 0;
        break;
    default:
        break;
    }
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceHimax::get_sensor_list(
    const ei_device_sensor_t **sensor_list,
    size_t *sensor_list_size)
{
    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;
    return false;
}

uint32_t EiDeviceHimax::get_data_output_baudrate(void)
{
    size_t length = strlen(ei_dev_max_data_output_baudrate.str);

    if(length < 32) {
        uint32_t baud_num = strtoul (ei_dev_max_data_output_baudrate.str, NULL, 0);
        return baud_num;
    }
    else {
        return -1;
    }
}

void EiDeviceHimax::set_default_data_output_baudrate(void)
{
    ei_serial_set_baudrate(ei_dev_default_data_output_baudrate.val);
}

void EiDeviceHimax::set_max_data_output_baudrate(void)
{
    ei_serial_set_baudrate(ei_dev_max_data_output_baudrate.val);
}

/**
 * @brief Setup timer or thread with given interval and call cb function each period
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return true
 */
bool EiDeviceHimax::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    stop_sampling = false;

    while (!stop_sampling) {

        sample_read_cb();
        ei_sleep(sample_interval_ms);
    }
    return true;
}

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceHimax::stop_sample_thread(void)
{
    stop_sampling = true;
    return true;
}

// to preserve compatibility with no memory interface devices using firmware-sdk
// supposed to be deprecating

/**
 * @brief Get byte size of memory block
 *
 * @return uint32_t size in bytes
 */
uint32_t EiDeviceHimax::filesys_get_block_size(void)
{
    return this->memory->block_size;
}

/**
 * @brief Get number of available blocks
 *
 * @return uint32_t
 */
uint32_t EiDeviceHimax::filesys_get_n_available_sample_blocks(void)
{
    return this->memory->get_available_sample_blocks();
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */

EiSnapshotProperties EiDeviceHimax::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;
    uint8_t res_num = 0;

    //TODO: move the getting of snapshot to camera device
    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = 0,
        .resolutions = res
    };

    if(this->cam->is_camera_present() == true) {
        this->cam->get_resolutions(&res, &res_num);

        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "Grayscale";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }

    return props;
}

/**
 * @brief      Call this function periocally during inference to
 *             detect a user stop command
 *
 * @return     true if user requested stop
 */
bool ei_user_invoke_stop_lib(void)
{
    bool stop_found = false;
    uint8_t character;

    while(hx_drv_uart_getchar(&character) == HX_DRV_LIB_PASS) {
        if(character == 'b') {

            stop_found = true;
            break;
        }
    }

    return stop_found;
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length)
{
    for(int i=0; i<length; i++) {
        hx_drv_uart_print("%c", *(data++));
    }
}

char ei_getchar()
{
	uint8_t ch;
    if (hx_drv_uart_getchar(&ch) == HX_DRV_LIB_PASS) {
        return (char)ch;
    }
    return 0;
}

/* Private functions ------------------------------------------------------- */
