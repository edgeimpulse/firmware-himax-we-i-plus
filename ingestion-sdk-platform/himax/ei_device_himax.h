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
