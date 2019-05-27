/**
 * @file XPT2046.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "xpt2046.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <stddef.h>

/*********************
 *      DEFINES
 *********************/
#define CMD_X_READ  0b10010000
#define CMD_Y_READ  0b11010000

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR touch_spi_ready (spi_transaction_t *trans);
static void tp_spi_init(spi_host_device_t spihost);
static uint8_t tp_spi_xchg(uint8_t data_send);
static void xpt2046_corr(int16_t * x, int16_t * y);
static void xpt2046_avg(int16_t * x, int16_t * y);


/**********************
 *  STATIC VARIABLES
 **********************/
int16_t avg_buf_x[XPT2046_AVG];
int16_t avg_buf_y[XPT2046_AVG];
uint8_t avg_last;
static spi_device_handle_t spi;
static volatile bool spi_trans_in_progress;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the XPT2046
 */
void xpt2046_init(spi_host_device_t spihost)
{

    tp_spi_init(spihost);

    gpio_set_direction(XPT2046_IRQ, GPIO_MODE_INPUT);
    gpio_set_direction(XPT2046_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(XPT2046_CS, 1);

}

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
bool xpt2046_read(lv_indev_data_t * data)
{
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    bool valid = true;
    uint8_t buf;

    int16_t x = 0;
    int16_t y = 0;

    uint8_t irq = gpio_get_level(XPT2046_IRQ);

    if(irq == 0) {
	// Acquire the bus first
	spi_device_acquire_bus(spi, portMAX_DELAY);

        gpio_set_level(XPT2046_CS, 0);
        tp_spi_xchg(CMD_X_READ);         /*Start x read*/

        buf = tp_spi_xchg(0);           /*Read x MSB*/
        x = buf << 8;
        buf = tp_spi_xchg(CMD_Y_READ);  /*Until x LSB converted y command can be sent*/
        x += buf;

        buf =  tp_spi_xchg(0);   /*Read y MSB*/
        y = buf << 8;

        buf =  tp_spi_xchg(0);   /*Read y LSB*/
        y += buf;
        gpio_set_level(XPT2046_CS, 1);

	// Release the bus
	spi_device_release_bus(spi);

        /*Normalize Data*/
        x = x >> 3;
        y = y >> 3;
        xpt2046_corr(&x, &y);
        xpt2046_avg(&x, &y);
        last_x = x;
        last_y = y;


    } else {
        x = last_x;
        y = last_y;
        avg_last = 0;
        valid = false;
    }

    data->point.x = x;
    data->point.y = y;
    data->state = valid == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

    return valid;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void IRAM_ATTR touch_spi_ready (spi_transaction_t *trans)
{
   spi_trans_in_progress = false;
}

static void tp_spi_init(spi_host_device_t spihost)
{

	esp_err_t ret;


	spi_device_interface_config_t devcfg={
		.clock_speed_hz = 10*1000*1000,           //Clock out at 80 MHz
		.mode = 0,                                //SPI mode 0

		// Set the CS pin here to -1, as we are doing 
		// continuous SPI transactions during xpt2046_read()
		.spics_io_num = -1,              //CS pin
		.queue_size = 1,
		.pre_cb = NULL,
		.post_cb = touch_spi_ready,
	};

	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(spihost, &devcfg, &spi);
	assert(ret==ESP_OK);
}

static uint8_t tp_spi_xchg(uint8_t data_send)
{
	uint8_t data_rec = 0;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = &data_send;            //Data
	t.rx_buffer = &data_rec;
	esp_err_t ret;

	// Set a variable to watch
	spi_trans_in_progress = true;
	spi_device_queue_trans(spi, &t, portMAX_DELAY);

	/* We can either call spi_device_get_trans_result()
	 * or check for a variable that is being set when
	 * a DMA or SPI completes the transaction
	 **/
	/*
	   spi_transaction_t * rt;
	   spi_device_get_trans_result(spi, &rt, portMAX_DELAY);
	   */
	while(spi_trans_in_progress == true);

	return data_rec;
}


static void xpt2046_corr(int16_t * x, int16_t * y)
{
#if XPT2046_XY_SWAP != 0
    int16_t swap_tmp;
    swap_tmp = *x;
    *x = *y;
    *y = swap_tmp;
#endif

    if((*x) > XPT2046_X_MIN)(*x) -= XPT2046_X_MIN;
    else(*x) = 0;

    if((*y) > XPT2046_Y_MIN)(*y) -= XPT2046_Y_MIN;
    else(*y) = 0;

    (*x) = (uint32_t)((uint32_t)(*x) * LV_HOR_RES) /
           (XPT2046_X_MAX - XPT2046_X_MIN);

    (*y) = (uint32_t)((uint32_t)(*y) * LV_VER_RES) /
           (XPT2046_Y_MAX - XPT2046_Y_MIN);

#if XPT2046_X_INV != 0
    (*x) =  LV_HOR_RES - (*x);
#endif

#if XPT2046_Y_INV != 0
    (*y) =  LV_VER_RES - (*y);
#endif


}


static void xpt2046_avg(int16_t * x, int16_t * y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for(i = XPT2046_AVG - 1; i > 0 ; i--) {
        avg_buf_x[i] = avg_buf_x[i - 1];
        avg_buf_y[i] = avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    avg_buf_x[0] = *x;
    avg_buf_y[0] = *y;
    if(avg_last < XPT2046_AVG) avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for(i = 0; i < avg_last ; i++) {
        x_sum += avg_buf_x[i];
        y_sum += avg_buf_y[i];
    }

    /*Normalize the sums*/
    (*x) = (int32_t)x_sum / avg_last;
    (*y) = (int32_t)y_sum / avg_last;
}


