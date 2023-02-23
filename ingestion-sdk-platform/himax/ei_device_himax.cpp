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
#include "ei_himax_fs_commands.h"
#include "ei_classifier_porting.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
//#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "repl.h"

#include "hx_drv_tflm.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <math.h>


/** Memory location for the arduino device address */
#define DEVICE_ID_LSB_ADDR  ((uint32_t)0x100000A4)
#define DEVICE_ID_MSB_ADDR  ((uint32_t)0x100000A8)

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE  32

/** Sensors */
typedef enum
{
    ACCELEROMETER = 0,
    MICROPHONE

}used_sensors_t;

/** Device type */
static const char *ei_device_type = "HIMAX_WE_I_PLUS";

/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE] = {"00:00:00:00:00:00"};

/** Device object, for this class only 1 object should exist */
EiDeviceHimax EiDevice;

static tEiState ei_program_state = eiStateIdle;

/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    "921600",
    UART_BR_921600,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    "115200",
    UART_BR_115200,
};

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

EiDeviceHimax::EiDeviceHimax(void)
{
    // uint32_t *id_msb = (uint32_t *)DEVICE_ID_MSB_ADDR;
    // uint32_t *id_lsb = (uint32_t *)DEVICE_ID_LSB_ADDR;

    /* Doesn't work here */
    // const uint32_t ID_MSB = 0x12345678;
    // const uint32_t ID_LSB = 0x9ABCDEFF;
    // uint32_t *id_msb = (uint32_t *)&ID_MSB;
    // uint32_t *id_lsb = (uint32_t *)&ID_LSB;

    // /* Setup device ID */
    // snprintf(&ei_device_id[0], DEVICE_ID_MAX_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X"
    //     ,(*id_msb >> 8) & 0xFF
    //     ,(*id_msb >> 0) & 0xFF
    //     ,(*id_lsb >> 24)& 0xFF
    //     ,(*id_lsb >> 16)& 0xFF
    //     ,(*id_lsb >> 8) & 0xFF
    //     ,(*id_lsb >> 0) & 0xFF
    //     );
    //
    //
}

/**
 * @brief      For the device ID, the BLE mac address is used.
 *             The mac address string is copied to the out_buffer.
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0
 */
int EiDeviceHimax::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    return get_id_c(out_buffer, out_size);
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceHimax::get_id_pointer(void)
{

    // const uint32_t ID_MSB = 0x12345678;
    // const uint32_t ID_LSB = 0x9ABCDEFF;
    // uint32_t *id_msb = (uint32_t *)&ID_MSB;
    // uint32_t *id_lsb = (uint32_t *)&ID_LSB;

    // /* Setup device ID */
    // snprintf(&ei_device_id[0], DEVICE_ID_MAX_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X"
    //     ,(*id_msb >> 8) & 0xFF
    //     ,(*id_msb >> 0) & 0xFF
    //     ,(*id_lsb >> 24)& 0xFF
    //     ,(*id_lsb >> 16)& 0xFF
    //     ,(*id_lsb >> 8) & 0xFF
    //     ,(*id_lsb >> 0) & 0xFF
    //     );

    return (const char *)ei_device_id;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceHimax::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceHimax::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      Set the device ID
 *
 * @param      device_id   MAC address
 *
 * @return     0
 */
int EiDeviceHimax::set_id(char *device_id)
{
    return set_id_c(device_id);
}

/**
 * @brief      Get the data output baudrate
 *
 * @param      baudrate    Baudrate used to output data
 *
 * @return     0
 */
int EiDeviceHimax::get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate)
{
    return get_data_output_baudrate_c(baudrate);
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceHimax::set_max_data_output_baudrate()
{
    set_max_data_output_baudrate_c();
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceHimax::set_default_data_output_baudrate()
{
    set_default_data_output_baudrate_c();
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceHimax::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceHimax::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceHimax::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = (ei_himax_fs_get_n_available_sample_blocks()-1) * ei_himax_fs_get_block_size();

    sensors[ACCELEROMETER].name = "Built-in accelerometer";
    sensors[ACCELEROMETER].start_sampling_cb = &ei_inertial_setup_data_sampling;
    sensors[ACCELEROMETER].max_sample_length_s = available_bytes / (100 * SIZEOF_N_AXIS_SAMPLED);
    sensors[ACCELEROMETER].frequencies[0] = 25.0f;
    sensors[ACCELEROMETER].frequencies[1] = 50.0f;
    sensors[ACCELEROMETER].frequencies[2] = 62.5f;
    sensors[ACCELEROMETER].frequencies[3] = 104.0f;

    sensors[MICROPHONE].name = "Built-in microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (16000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;

    *sensor_list      = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceHimax::get_snapshot_list(const ei_device_snapshot_resolutions_t **snapshot_list, size_t *snapshot_list_size)
{
    snapshot_resolutions[0].width = 128;
    snapshot_resolutions[0].height = 96;

    snapshot_resolutions[1].width = 320;
    snapshot_resolutions[1].height = 240;

    snapshot_resolutions[2].width = 640;
    snapshot_resolutions[2].height = 480;

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
    snapshot_resolutions[3].width = EI_CLASSIFIER_INPUT_WIDTH;
    snapshot_resolutions[3].height = EI_CLASSIFIER_INPUT_HEIGHT;
#endif

    *snapshot_list      = snapshot_resolutions;
    *snapshot_list_size = EI_DEVICE_N_RESOLUTIONS;

    return false;
}

/**
 * @brief      Create resolution list for resizing
 * @param      resize_list       Place pointer to resolution list
 * @param      resize_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceHimax::get_resize_list(const ei_device_resize_resolutions_t **resize_list, size_t *resize_list_size)
{
    resize_resolutions[0].width = 128;
    resize_resolutions[0].height = 96;

    resize_resolutions[1].width = 160;
    resize_resolutions[1].height = 120;

    resize_resolutions[2].width = 200;
    resize_resolutions[2].height = 150;

    resize_resolutions[3].width = 256;
    resize_resolutions[3].height = 192;

    resize_resolutions[4].width = 320;
    resize_resolutions[4].height = 240;

    resize_resolutions[5].width = 400;
    resize_resolutions[5].height = 300;

    resize_resolutions[6].width = 512;
    resize_resolutions[6].height = 384;

    *resize_list      = resize_resolutions;
    *resize_list_size = EI_DEVICE_N_RESIZE_RESOLUTIONS;

    return false;
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


void EiDeviceHimax::setup_led_control(void)
{
    // tHalTmr *AppTmr = NULL;
    /* Create Periodic timer */
    // AppTmr = HalTmrCreate(HalTmrCh0, HalTmrPeriodic, 1000, timer_callback, AppTmr);
    // if (!AppTmr)
    //     ei_printf("TImer Creatation Failed\n");
    // else
    //     HalTmrStart(AppTmr);
}

void EiDeviceHimax::set_state(tEiState state)
{
    ei_program_state = state;
    static uint8_t upload_toggle = 0;

    if((state == eiStateFinished) || (state == eiStateIdle)){
        ei_program_state = eiStateIdle;

        hx_drv_led_off(HX_DRV_LED_GREEN);
        hx_drv_led_off(HX_DRV_LED_RED);
        upload_toggle = 0;

    }
    else if (state == eiStateSampling) {
        hx_drv_led_on(HX_DRV_LED_GREEN);
    }
    else if (state == eiStateUploading) {

        if(upload_toggle) {
            hx_drv_led_on(HX_DRV_LED_RED);
        }
        else {
            hx_drv_led_off(HX_DRV_LED_RED);
            hx_drv_led_off(HX_DRV_LED_GREEN);
        }
        upload_toggle ^= 1;
    }


}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceHimax::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceHimax::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the set_id method
 *
 * @return     Pointer to c get function
 */
c_callback_set_id EiDeviceHimax::set_id_function(void)
{
    return &set_id_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceHimax::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceHimax::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceHimax::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      Get characters for uart pheripheral and send to repl
 */
void ei_command_line_handle(void *args)
{
    uint8_t character;

    while (1) {
        if (hx_drv_uart_getchar(&character) == HX_DRV_LIB_PASS) {
            rx_callback(character);
        }
        // handle at least once every ~10.7 sec
        ei_read_timer_ms();
    }
}

/**
 * @brief      Call this function periocally during inference to
 *             detect a user stop command
 *
 * @return     true if user requested stop
 */
bool ei_user_invoke_stop(void)
{
    bool stop_found = false;
    uint8_t character;

    while(hx_drv_uart_getchar(&character) == HX_DRV_LIB_PASS) {
        if(character == 'b') {

            stop_found = true;
            break;
        }
    }

    // do {
    //     hx_drv_uart_getchar(&character);

    //     if(character == 'b') {

    //         stop_found = true;
    //         break;
    //     }
    // }
    // while(character);

    return stop_found;
}

/**
 * @brief      Setup the serial port
 */
void ei_serial_setup(void)
{

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


/* Private functions ------------------------------------------------------- */
static void timer_callback(void *arg)
{
    static char toggle = 0;

    if(toggle) {
        switch(ei_program_state)
        {
            // case eiStateErasingFlash:   EtaBspLedSet(ETA_BSP_LED1); break;
            // case eiStateSampling:       EtaBspLedSet(ETA_BSP_LED2); break;
            // case eiStateUploading:      EtaBspLedSet(ETA_BSP_LED3); break;
            default: break;
        }
    }
    // else {
    //     if(ei_program_state != eiStateFinished) {
    //         EtaBspLedsClearAll();
    //     }
    // }
    toggle ^= 1;
}


static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_id);

    if(length < 32) {
        memcpy(out_buffer, ei_device_id, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if(length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}


static int set_id_c(char *device_id)
{
    size_t length = strlen(ei_device_id);
    if (length > 31) {
        return -1;
    }

    memcpy(ei_device_id, device_id, strlen(device_id) + 1);

    return 0;
}

static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate)
{
    size_t length = strlen(ei_dev_max_data_output_baudrate.str);

    if(length < 32) {
        memcpy(baudrate, &ei_dev_max_data_output_baudrate, sizeof(ei_device_data_output_baudrate_t));
        return 0;
    }
    else {
        return -1;
    }
}

static void set_max_data_output_baudrate_c()
{
    ei_serial_set_baudrate(ei_dev_max_data_output_baudrate.val);
}

static void set_default_data_output_baudrate_c()
{
    ei_serial_set_baudrate(ei_dev_default_data_output_baudrate.val);
}


static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  use_max_baudrate    whether to switch to max baudrate or not
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t))
{
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {

        EiDevice.set_state(eiStateUploading);

        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        int r = ei_himax_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            retVal = false;
            break;

        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    EiDevice.set_state(eiStateFinished);

    return retVal;
}
