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
#include "ei_inertialsensor.h"
#include "ei_run_impulse.h"
#include "at_cmds.h"

#include "hx_drv_tflm.h"

int main(void)
{
	hx_drv_uart_initial(UART_BR_115200);
	hx_drv_tick_start();

    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n", __DATE__, __TIME__);

    /* Setup the inertial sensor */
    if (ei_inertial_init() == false) {
        ei_printf("Inertial sensor communication error occured\r\n");
    }

    /* Intialize configuration */
    static ei_config_ctx_t config_ctx = {0};
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function();
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();
    config_ctx.load_config = &ei_himax_fs_load_config;
    config_ctx.save_config = &ei_himax_fs_save_config;
    config_ctx.list_files = NULL;
    config_ctx.read_buffer = EiDevice.get_read_sample_buffer_function();
    config_ctx.take_snapshot = &ei_himax_take_snapshot;

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    } else {
        ei_printf("Loaded configuration\n");
    }

    /* Setup the command line commands */
    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_normal);
    ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse", run_nn_continuous_normal);
    ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with extra debug output", run_nn_debug);
    ei_printf("Type AT+HELP to see a list of commands.\r\n> ");

    ei_command_line_handle(0);

    return 0;
}
