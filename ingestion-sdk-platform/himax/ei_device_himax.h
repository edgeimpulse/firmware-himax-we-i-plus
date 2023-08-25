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

#ifndef EI_DEVICE_HIMAX
#define EI_DEVICE_HIMAX

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_device_info_lib.h"
#include "ei_classifier_porting.h"
#include <cstdlib>
#include <cstdint>
#include "ei_camera.h"

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
#define EI_DEVICE_N_RESOLUTIONS		4
#else
#define EI_DEVICE_N_RESOLUTIONS		3
#endif

typedef struct {
	size_t width;
	size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Class description and implementation of device specific
 * 			   characteristics
 */
class EiDeviceHimax : public EiDeviceInfo {
private:
    static const int standalone_sensor_num = 1;
    ei_device_sensor_t standalone_sensor_list[standalone_sensor_num];

	static ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS];

    bool camera_present;
    EiCameraHimax *cam;

    bool network_present;
    bool network_connected;
    //EiNetworkDevice *net;

    EiDeviceHimax() = delete;
    std::string mac_address = "00:11:22:33:44:55:66";
    EiState state;

public:

    EiDeviceHimax(EiDeviceMemory* mem);
    ~EiDeviceHimax();

    void delay_ms(uint32_t milliseconds);

    std::string get_mac_address(void);

    void (*sample_read_callback)(void);
    void init_device_id(void) override;

    bool is_camera_present(void);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;
    void set_state(EiState state) override;
    EiState get_state(void);
    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;
    EiSnapshotProperties get_snapshot_list(void) override;
    uint32_t filesys_get_block_size(void) override;
    uint32_t filesys_get_n_available_sample_blocks(void) override;
    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    uint32_t get_data_output_baudrate(void) override;

};

/* Function prototypes ----------------------------------------------------- */
void ei_command_line_handle(void *args);
bool ei_user_invoke_stop_lib(void);
void ei_serial_setup(void);
void ei_write_string(char *data, int length);

/* Reference to object for external usage ---------------------------------- */
extern EiDeviceHimax EiDevice;

#endif // EI_DEVICE_HIMAX
