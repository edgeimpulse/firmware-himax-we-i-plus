#include "ei_camera.h"
#include "hx_drv_tflm.h"

static bool is_initialised = false;
static hx_drv_sensor_image_config_t g_pimg_config;

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
 * @brief      Capture and rescale image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[out] buf           pointer to store output image
 *
 * @retval     false if not initialised, image capture or rescaled failed
 *
 * @note       original capture is 640x480
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, int8_t *buf)
{
    if (is_initialised == false) return false;

    if (hx_drv_sensor_capture(&g_pimg_config) != HX_DRV_LIB_PASS) {
        return false;
    }

    if (hx_drv_image_rescale((uint8_t*)g_pimg_config.raw_address,
                             g_pimg_config.img_width, g_pimg_config.img_height,
                             buf, img_width, img_height) != HX_DRV_LIB_PASS) {
        return false;
    }

    return true;
}
