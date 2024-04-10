/**
 * @file LVGL_DMD.c
 *
 * @brief LVGL display driver for the Freetronics DMD display.
 */

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"  
#include "LVGL_DMD/LVGL_DMD.h"
#include "lvgl/src/misc/lv_types.h"
#include "lvgl/src/display/lv_display.h"
#include <stdbool.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/

// assuming a 8x8 font size
#define FONT_WIDTH 8
#define FONT_HEIGHT 8

// display resolution defines
#define MY_DISP_HOR_RES 32
#define MY_DISP_VER_RES 32

// SPI bus configuration
#define SPI_HOST        VSPI_HOST
#define SPI_MISO        -1  // not used
#define SPI_MOSI        23
#define SPI_CLK         18
#define SPI_CS          5

#define DMD_REFRESH_RATE 60 // in Hz

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void disp_init(void);

void dmd_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map);

static void dmd_refresh_task(void *arg);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_dmd_init(void)
{
    /*-------------------------
    * Initialize display
    * -----------------------*/
    disp_init();

    /*------------------------------------
    * Create a display and set a flush_cb
    * -----------------------------------*/
    lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    lv_display_set_flush_cb(disp, dmd_flush_cb);

    /* Example 1
    * One buffer for partial rendering*/
    static lv_color_t buf_1_1[MY_DISP_HOR_RES * 10];                          /*A buffer for 10 rows*/
    lv_display_set_buffers(disp, buf_1_1, NULL, sizeof(buf_1_1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // create a task to periodically refresh the DMD display
    xTaskCreate(dmd_refresh_task, "DMD Refresh", 2048, NULL, configMAX_PRIORITIES - 1, NULL);

    /* Example 2
    * Two buffers for partial rendering
    * In flush_cb DMA or similar hardware should be used to update the display in the background.*/
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * 10];
    lv_display_set_buffers(disp, buf_2_1, buf_2_2, sizeof(buf_2_1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* Example 3
    * Two buffers screen sized buffer for double buffering.
    * Both LV_DISPLAY_RENDER_MODE_DIRECT and LV_DISPLAY_RENDER_MODE_FULL works, see their comments*/
    static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];
    static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];
    lv_display_set_buffers(disp, buf_3_1, buf_3_2, sizeof(buf_3_1), LV_DISPLAY_RENDER_MODE_DIRECT);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

// DMA buffer for display updates
static uint8_t dma_buffer[MY_DISP_HOR_RES * 8];
static bool dma_transfer_in_progress = false;

static spi_device_handle_t spi_handle;

/*Initialize the display and the required peripherals.*/
static void disp_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(dma_buffer),
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10 * 1000 * 1000,
        .spics_io_num = SPI_CS,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_RXBIT_LSBFIRST | SPI_DEVICE_TXBIT_LSBFIRST
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle));
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
*/
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
*/
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display.*/

void put_px(int32_t x, int32_t y, uint16_t color) 
{
    // convert the 16-bit color value to a single-bit pixel value (0x00 or 0xFF)
    uint8_t pixel_value = (color > 0) ? 0xFF : 0x00;
    uint8_t pixel_data = 0;

    // pack the 8 pixels (corresponding to the 8-bit font width) into a single byte
    for (int i = 0; i < 8; i++) {
        // check if the current pixel index is within the display's horizontal resolution
        pixel_data |= ((x + i) < MY_DISP_HOR_RES ? ((pixel_value >> i) & 0x01) << i : 0);
    }

    // store the packed pixel data in the DMA buffer
    dma_buffer[(y * MY_DISP_HOR_RES + x) / 8] = pixel_data;
}


void dmd_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map) 
{
    // check if the display update is enabled
    if (!disp_flush_enabled) {
        // if not enabled, simply mark the flush as ready and return
        lv_display_flush_ready(display);
        return;
    }

    int32_t x, y;
    for (y = area->y1; y <= area->y2; y++) {
        for (x = area->x1; x <= area->x2; x += 8) {
            uint8_t pixel_data = 0;
            // pack the 8 pixels (corresponding to the 8-bit font width) into a single byte
            for (int i = 0; i < 8 && (x + i) <= area->x2; i++) {
                pixel_data |= ((px_map[i]) ? 0xFF : 0x00) << i;
            }
            // store the packed pixel data in the DMA buffer
            put_px(x, y, pixel_data);
            px_map += 8;
        }
    }

    // check if a DMA transfer is not in progress
    if (!dma_transfer_in_progress) {
        // if not, start a new DMA transfer to refresh the display
        dma_transfer_in_progress = true;
        spi_transaction_t transaction = {
            .length = sizeof(dma_buffer) * 8,
            .tx_buffer = dma_buffer,
        };
        ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &transaction, portMAX_DELAY));
        spi_transaction_t* out_trans;
        ESP_ERROR_CHECK(spi_device_get_trans_result(spi_handle, &out_trans, portMAX_DELAY));
        dma_transfer_in_progress = false;
    }

    lv_display_flush_ready(display);
}

static void dmd_refresh_task(void *arg) 
{
    while (1) {
        // trigger a display refresh by checking if a DMA transfer is not in progress
        if (!dma_transfer_in_progress) {
            dma_transfer_in_progress = true;
            spi_transaction_t transaction = {
                .length = sizeof(dma_buffer) * 8,
                .tx_buffer = dma_buffer,
            };
            ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &transaction, portMAX_DELAY));
            spi_transaction_t* out_trans;
            ESP_ERROR_CHECK(spi_device_get_trans_result(spi_handle, &out_trans, portMAX_DELAY));
            dma_transfer_in_progress = false;
        }

        // wait for the next refresh cycle
        vTaskDelay(pdMS_TO_TICKS(1000 / DMD_REFRESH_RATE));
    }
}
