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

/* Include ----------------------------------------------------------------- */
#include "ei_device_himax.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "ei_at_handlers.h"

#include "hx_drv_tflm.h"

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceHimax::get_device());
static ATServer *at;

int main(void)
{
    hx_drv_uart_initial(UART_BR_115200);
    hx_drv_tick_start();

    /* Initialize Edge Impulse sensors and commands */

    EiDeviceHimax* dev = static_cast<EiDeviceHimax*>(EiDeviceHimax::get_device());
    dev->set_default_data_output_baudrate();

    ei_printf(
        "Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);

    /* Setup the inertial sensor */
    if (ei_inertial_init() == false) {
        ei_printf("Inertial sensor initialization failed\r\n");
    }

    /* Setup the microphone */
    if (ei_microphone_init() == false) {
        ei_printf("Microphone initialization failed\r\n");
    }

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();

    // you might think this is not necessary, but without this line the AT+SNAPSHOT commands first baud rate switch
    // is ignored by the Himax HAL
    dev->set_default_data_output_baudrate();

    while (1) {
        uint8_t data;
        if (hx_drv_uart_getchar(&data) == HX_DRV_LIB_PASS) {
            at->handle(data);
        }
        // handle at least once every ~10.7 sec
        ei_read_timer_ms();
    }

    return 0;
}
