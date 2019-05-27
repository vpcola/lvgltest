/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "demo.h"
#include "esp_freertos_hooks.h"


#include "ili9341.h"
#include "xpt2046.h"

#define LVGL_SPI_MISO CONFIG_LVGL_SPI_MISO
#define LVGL_SPI_MOSI CONFIG_LVGL_SPI_MOSI
#define LVGL_SPI_SCLK CONFIG_LVGL_SPI_SCLK

#if defined(LVGL_SPI_VSPI)
#define LVGL_SPI_HOST VSPI_HOST
#else
#define LVGL_SPI_HOST HSPI_HOST
#endif

static void IRAM_ATTR lv_tick_task(void);

void init_spi()
{
	esp_err_t err;

	spi_bus_config_t spi_bus_config;
	spi_bus_config.miso_io_num = LVGL_SPI_MISO;
	spi_bus_config.mosi_io_num = LVGL_SPI_MOSI;
	spi_bus_config.sclk_io_num = LVGL_SPI_SCLK;
	spi_bus_config.quadwp_io_num = -1;
	spi_bus_config.quadhd_io_num = -1;
	spi_bus_config.max_transfer_sz = LV_VDB_SIZE * 2;
	err = spi_bus_initialize(LVGL_SPI_HOST, &spi_bus_config, 1);
	ESP_ERROR_CHECK(err);
}

void app_main()
{

	init_spi();

	lv_init();

	ili9341_init(LVGL_SPI_HOST);
	xpt2046_init(LVGL_SPI_HOST);

	// Setup the lvgl display device
	lv_disp_drv_t disp;
	lv_disp_drv_init(&disp);
	disp.disp_flush = ili9341_flush;
	disp.disp_fill = ili9341_fill;
	lv_disp_drv_register(&disp);

	// Setup lvgl input device
	lv_indev_drv_t indev;
	lv_indev_drv_init(&indev);
	indev.read = xpt2046_read;
	indev.type = LV_INDEV_TYPE_POINTER;
	lv_indev_drv_register(&indev);

	esp_register_freertos_tick_hook(lv_tick_task);

	demo_create();

	while(1) {
		vTaskDelay(1);
		lv_task_handler();
	}
}

static void IRAM_ATTR lv_tick_task(void)
{
	lv_tick_inc(portTICK_RATE_MS);
}
