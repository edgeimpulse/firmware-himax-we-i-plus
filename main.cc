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
