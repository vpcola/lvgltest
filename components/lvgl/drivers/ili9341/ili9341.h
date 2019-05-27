/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9341_H
#define ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/*********************
 *      DEFINES
 *********************/

#define ILI9341_CS   CONFIG_ILI9341_CS 
#define ILI9341_DC   CONFIG_ILI9341_DC 
#define ILI9341_RST  CONFIG_ILI9341_RESET
#define ILI9341_BCKL CONFIG_ILI9341_BCKL

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ili9341_init(spi_host_device_t spihost);
void ili9341_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color);
void ili9341_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9341_H*/
