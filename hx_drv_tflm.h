#ifndef HX_DRV_P021_H_
#define HX_DRV_P021_H_

#include <stdint.h>

/****************************************************
 * ENUM Declaration                                 *
 ***************************************************/
/**
 * \enum HX_DRV_ERROR_E
 * \brief Himax driver error code
 */
typedef enum {
    HX_DRV_LIB_PASS = 0,  /**< Error code - PASS */
    HX_DRV_LIB_ERROR = -1, /**< Error code - FAIL */
} HX_DRV_ERROR_E;

/**
 * \enum HX_DRV_GPIO_E
 * \brief Himax driver GPIO selection
 */
typedef enum {
    HX_DRV_PGPIO_0 = 0,  /**< Select GPIO number 0  */
    HX_DRV_PGPIO_1 = 1,  /**< Select GPIO number 1 */
    HX_DRV_PGPIO_2 = 2,  /**< Select GPIO number 2 */
} HX_DRV_GPIO_E;

/**
 * \enum HX_DRV_GPIO_DIRCTION_E
 * \brief Himax driver GPIO direction, including input and output direction
 */
typedef enum {
    HX_DRV_GPIO_INPUT = 2,    /**< Select GPIO as input direction  */
    HX_DRV_GPIO_OUTPUT = 3,   /**< Select GPIO as output direction  */
} HX_DRV_GPIO_DIRCTION_E;

/**
 * \enum HX_DRV_LED_SELECT_E
 * \brief Himax driver LED selection
 */
typedef enum {
    HX_DRV_LED_RED = 6,    /**< Select Red LED  */
    HX_DRV_LED_GREEN = 8,    /**< Select Green LED  */
} HX_DRV_LED_SELECT_E;

/**
 * \enum HX_DRV_LED_ONOFF_E
 * \brief Himax driver LED on off operation
 */
typedef enum {
    HX_DRV_LED_OFF = 0,    /**< LED on operation  */
    HX_DRV_LED_ON = 1,     /**< LED off operation  */
} HX_DRV_LED_ONOFF_E;

typedef enum {
	SPI_TYPE_JPG               = 0x01,    /**< transfer data type is JPEG image */
	SPI_TYPE_RAW               = 0x02,    /**< transfer data type is RAW image  */
	SPI_TYPE_META_DATA         = 0x03,    /**< transfer data type is meta-data  */
	SPI_TYPE_PDM     	 	   = 0x04,    /**< transfer data type is audio PDM data  */
}HX_DRV_SPI_TYPE;

typedef enum {
	SHARE_MODE_SPIM            = 0x01,    /**< switch share pin to spi master mode */
	SHARE_MODE_I2CM            = 0x02,    /**< switch share pin to i2c master mode */
}HX_DRV_SHARE_MODE;

/****************************************************
 * Structure Definition                             *
 ***************************************************/

/**
 * \struct hx_drv_sensor_image_config_t
 * \brief Himax driver image sensor configuration
 */
typedef struct {
	uint32_t img_width;    /**< image width, assigned by Himax driver */
	uint32_t img_height;   /**< image height, assigned by Himax driver */
    uint32_t jpeg_address; /**< JPEG image address, assigned by Himax driver */
    uint32_t jpeg_size;    /**< JPEG image size, assigned by Himax driver */
    uint32_t raw_address;  /**< RAW image address, assigned by Himax driver */
    uint32_t raw_size;     /**< RAW image size, assigned by Himax driver */

} hx_drv_sensor_image_config_t;

/**
 * \struct hx_drv_mic_data_config_t
 * \brief Himax driver microphone configuration
 */
typedef struct {
    uint32_t data_address;  /**< microphone data array address, assigned by Himax driver */
    uint32_t data_size;      /**< microphone data size about how many samples, assigned by Himax driver */

} hx_drv_mic_data_config_t;

/**
 * \struct hx_drv_gpio_config_t
 * \brief Himax driver GPIO configuration
 */
typedef struct {
	HX_DRV_GPIO_E gpio_pin;                 /**< GPIO pin, assigned by user */
    uint8_t gpio_data;                      /**< GPIO data. Assigned by user when as output direction. Assigned by Himax driver when as input direction */
    HX_DRV_GPIO_DIRCTION_E gpio_direction;  /**< GPIO direction, assigned by user */

} hx_drv_gpio_config_t;


/****************************************************
 * Function Declaration                             *
 ***************************************************/

#ifdef __cplusplus
extern "C" {
#endif
/**
 * \brief	Check current himax driver version.
 *
 * \param[out] major_ver		Major version of the driver
 * \param[out] minor_ver		Minor version of the driver
 * \retval	HX_DRV_LIB_PASS		Success
 */
extern HX_DRV_ERROR_E hx_drv_lib_version(uint32_t *major_ver, uint32_t *minor_ver);

/**
 * \brief	Image sensor initialization, sensor start streaming after initial.
 *          Current image sensor use for himax_we1_evb is HM0360, image resolution is 640x480.
 *
 * \param[out] pimg_config		Img_width, img_height, jpeg_address, raw_address will be assigned by driver after initial success
 * \retval	HX_DRV_LIB_PASS		Initial success
 * \retval	HX_DRV_LIB_FAIL		Initial fail
 */
extern HX_DRV_ERROR_E hx_drv_sensor_initial(hx_drv_sensor_image_config_t *pimg_config);

/**
 * \brief	Image sensor capture, it will capture one frame and return the frame information.
 * 			both RAW image and JPEG image will be provided.
 *
 * \param[out] pimg_config		Jpeg_size, raw_size of the captured frame will be given after capture success
 * \retval	HX_DRV_LIB_PASS		Capture one frame success
 * \retval	HX_DRV_LIB_FAIL		Capture operation fail
 */
extern HX_DRV_ERROR_E hx_drv_sensor_capture(hx_drv_sensor_image_config_t *pimg_config);

/**
 * \brief	WE1 Stop capture and Sensor stop streaming
 *
 * \retval	HX_DRV_LIB_PASS		Stop capture and streaming success
 * \retval	HX_DRV_LIB_FAIL		Stop capture and streaming fail
 */
extern HX_DRV_ERROR_E hx_drv_sensor_stop_capture();

/**
 * \brief	Image sensor start streaming, it often use after stream off and want to re-start streaming.
 *
 * \retval	HX_DRV_LIB_PASS		Start streaming success
 * \retval	HX_DRV_LIB_FAIL		Start streaming fail
 */
extern HX_DRV_ERROR_E hx_drv_sensor_stream_on();

/**
 * \brief	Image sensor stop streaming.
 *
 * \retval	HX_DRV_LIB_PASS		Stop streaming success
 * \retval	HX_DRV_LIB_FAIL		Stop streaming fail
 */
extern HX_DRV_ERROR_E hx_drv_sensor_stream_off();

/**
 * \brief	Image scale down function, make image resolution smaller.
 *
 * \param[in] in_image			Input image address
 * \param[in] in_image_width	Input image width in pixel
 * \param[in] in_image_height	Input image height in pixel
 * \param[in] out_image			Output image address
 * \param[in] out_image_width	Output image width in pixel, it should be smaller than input image width
 * \param[in] out_image_height	Output image height in pixel, it should be smaller than input image height
 * \retval	HX_DRV_LIB_PASS		Stop streaming success
 * \retval	HX_DRV_LIB_FAIL		Stop streaming fail
 */
extern HX_DRV_ERROR_E hx_drv_image_rescale(uint8_t*in_image, int32_t in_image_width, int32_t in_image_height,  int8_t*out_image, int32_t out_image_width, int32_t out_image_height);

/**
 * \brief	3-axis accelerometer initialization, it start to retrieve data after initial.
 *          It will initial accelerometer with sampling rate 119 Hz, bandwidth 50 Hz, scale selection 4g at continuous mode.
 *
 * \retval	HX_DRV_LIB_PASS		Initial success
 * \retval	HX_DRV_LIB_FAIL		Initial fail
 */
extern HX_DRV_ERROR_E hx_drv_accelerometer_initial();

/**
 * \brief	Receive data from 3-axis accelerometer.
 *
 * \param[out] x				Data in x-axis
 * \param[out] y				Data in y-axis
 * \param[out] z				Data in z-axis
 * \retval	HX_DRV_LIB_PASS		Receive data success
 * \retval	HX_DRV_LIB_FAIL		Receive data fail
 */
extern HX_DRV_ERROR_E hx_drv_accelerometer_receive(float *x, float *y, float *z);

/**
 * \brief	Check how many data in the accelerometer FIFO. each count represent 1 set of x-axis,y-axis,z-axis data.
 *
 * \retval	HX_DRV_LIB_PASS		Check success
 * \retval	HX_DRV_LIB_FAIL		Check fail
 */
extern uint8_t hx_drv_accelerometer_available_count();

/**
 * \brief	Microphone initialization, it will initial microphone setting. Please use hx_drv_mic_on() to start record audio after initial.
 *          Receive sampling rate is 16KHz and data format is PDM.
 *
 * \retval	HX_DRV_LIB_PASS		Initial success
 * \retval	HX_DRV_LIB_FAIL		Initial fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_initial();

/**
 * \brief	Capture Single channel audio data from Microphone. Each sample for mono PDM is 16bits little-endian signed data.
 *          During each millisecond, there will be 16 samples(32 bytes) of audio data storage to target address.
 * 			This API is provided for TFLM micro_speech example, which includes some previous audio data for sliding window usage.
 * 			Please use API "hx_drv_mic_capture_dual" for normal case.
 *
 * \param[out] pmic_config		Received data will be assigned by driver with address and size count in bytes about samples.
 * \retval	HX_DRV_LIB_PASS		Capture success
 * \retval	HX_DRV_LIB_FAIL		Capture fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_capture(hx_drv_mic_data_config_t *pmic_config);

/**
 * \brief	Get current time-stamp from audio buffer in driver.
 *
 * \param[out] time				time-stamp represent in millisecond
 * \retval	HX_DRV_LIB_PASS		Capture success
 * \retval	HX_DRV_LIB_FAIL		Capture fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_timestamp_get(int32_t *time);

/**
 * \brief	Turn on microphone, it will start to record audio.
 *
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_on();

/**
 * \brief	Turn off microphone, it will stop receive audio data and time-stamp reset back to zero.
 *
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_off();

/**
 * \brief	Capture Dual channel audio data from Microphone. Each sample for dual PDM are 32 bits, includes Left channel 16bits little-endian signed data
 * 			and right channel 16bits little-endian signed data.
 *          During each millisecond, there will be 16 samples(64 bytes) of audio data storage to target address.
 *          This API often called when get changes by API "hx_drv_mic_timestamp_get"
 *
 * \param[out] pmic_config		Received data will be assigned by driver with address and size count in bytes about samples. For example,
 * 								if data_size is 6400, that means 1600 samples(100ms) of audio data in address.
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_mic_capture_dual(hx_drv_mic_data_config_t *pmic_config);
/**
 * \brief	UART initialization, default speed is 115200 bps. This is for message output.
 *
 * \retval	HX_DRV_LIB_PASS		Initial success
 * \retval	HX_DRV_LIB_FAIL		Initial fail
 */
extern HX_DRV_ERROR_E hx_drv_uart_initial();

/**
 * \brief	Print message to UART port.
 *
 * \param[in] fmt				Data to print through UART
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_uart_print(const char*fmt, ...);

/**
 * \brief	Get input from UART port.
 *
 * \param[out] pch				It catch one character from UART
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_uart_getchar(uint8_t *pch);

/**
 * \brief	GPIO initialization, please set configuration direction to input/output with dedicated GPIO pin.
 *
 * \param[in/out] pgpio_config	It catch one character from UART
 * \retval	HX_DRV_LIB_PASS		Initial success
 * \retval	HX_DRV_LIB_FAIL		Initial fail
 */
extern HX_DRV_ERROR_E hx_drv_gpio_initial(hx_drv_gpio_config_t *pgpio_config);

/**
 * \brief	For GPIO output setting, set target gpio_data (0 or 1) to dedicated GPIO pin.
 *
 * \param[in/out] pgpio_config	gpio_data should be set to 0 or 1 for GPIO output
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_gpio_set(hx_drv_gpio_config_t *pgpio_config);

/**
 * \brief	Get value from dedicated GPIO pin.
 *
 * \param[out] pgpio_config		Read back data will be assigned to gpio_data by driver
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_gpio_get(hx_drv_gpio_config_t *pgpio_config);

/**
 * \brief	Turn target LED on.
 *
 * \param[in] led				selected LED to turn on
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_led_on(HX_DRV_LED_SELECT_E led);

/**
 * \brief	Turn target LED off.
 *
 * \param[in] led				selected LED to turn off
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_led_off(HX_DRV_LED_SELECT_E led);

/**
 * \brief	Timer initialization. It start to count tick after start operation.
 *
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_tick_start(void);

/**
 * \brief	Get current tick in timer.
 *
 * \param[out] tick				fill tick address in the parameter and driver will fill tick count data to the address
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_tick_get(uint32_t *tick);

/**
 * \brief	I2c master control. use to set i2c with target ID, target address with data
 *
 * \param[in] slave_addr_sft	target device ID represent in 7-bits mode
 * \param[in] addr				target address to set data
 * \param[in] addr_len			target address size in bytes
 * \param[in] data				data array pointer
 * \param[in] data_len			data array size in bytes
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_i2cm_set_data(uint8_t slave_addr_sft, uint8_t *addr, uint32_t addr_len, uint8_t *data, uint32_t data_len);

/**
 * \brief	I2c master control. use to read i2c data with target ID
 *
 * \param[in] slave_addr_sft	target device ID represent in 7-bits mode
 * \param[in] addr				target address to read data
 * \param[in] addr_len			target address size in bytes
 * \param[out] data				data array pointer
 * \param[out] data_len			data array size in bytes
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_i2cm_get_data(uint8_t slave_addr_sft, uint8_t *addr, uint32_t addr_len, uint8_t *data, uint32_t data_len);

/**
 * \brief	spi master control initial
 *
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_spim_init();

/**
 * \brief	spi master send data from dedicated memory address
 *
 * \param[in] addr				data array address in memory to send
 * \param[in] size				data array size
 * \param[in] data_type			data type
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_spim_send(uint32_t addr, uint32_t size, HX_DRV_SPI_TYPE data_type);

/**
 * \brief	SPI master and I2C master share the same output pin, we need to switch to needed output mode.
 *          Default output mode is SHARE_MODE_SPIM
 *
 * \param[in] mode				select which device to output
 * \retval	HX_DRV_LIB_PASS		Operation success
 * \retval	HX_DRV_LIB_FAIL		Operation fail
 */
extern HX_DRV_ERROR_E hx_drv_share_switch(HX_DRV_SHARE_MODE mode);

#ifdef __cplusplus
}
#endif

#endif /* HX_DRV_P021_H_ */
