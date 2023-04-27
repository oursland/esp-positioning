#include "ism330dhcx-task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "rom/ets_sys.h"

#include "ism330dhcx_reg.h"

#define TAG         "ism330dhcx-task"

#define IMU_HOST    SPI2_HOST
#define IMU_CS      GPIO_NUM_48
#define IMU_SCLK    GPIO_NUM_47
#define IMU_MISO    GPIO_NUM_14
#define IMU_MOSI    GPIO_NUM_21
#define IMU_INT1    GPIO_NUM_13
#define IMU_INT2    GPIO_NUM_45

static int32_t stmdev_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    spi_device_handle_t spi = (spi_device_handle_t)handle;
    spi_transaction_t t = {
        .addr = reg,
        .length = len*8,
        .tx_buffer = (const void *)data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, &t);
    if(err != ESP_OK) {
        return -1;
    }
    return 0;
}

static int32_t stmdev_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    spi_device_handle_t spi = (spi_device_handle_t)handle;
    spi_transaction_t t = {
        .addr = reg | 0x80,
        .length = len*8,
        .rxlength = len*8,
        .rx_buffer = (void *)data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, &t);
    if(err != ESP_OK) {
        return -1;
    }
    return 0;
}

static void stmdev_mdelay(uint32_t millisec) {
    vTaskDelay(pdMS_TO_TICKS(millisec));
}

void ism330dhcx_task_func(void *args) {
    ESP_LOGI(TAG, "Starting ISM330DHCX Task");

    // ISM330DHCX
    esp_err_t ret;
    spi_device_handle_t imu_spi_handle;
    spi_bus_config_t imu_bus_config = {
        .mosi_io_num = IMU_MOSI,
        .miso_io_num = IMU_MISO,
        .sclk_io_num = IMU_SCLK,
        .quadwp_io_num = -1,
    };
    spi_device_interface_config_t imu_device_config = {
        .address_bits = 8,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = IMU_CS,
        .queue_size = 1,
    };
    ret = spi_bus_initialize(IMU_HOST, &imu_bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(IMU_HOST, &imu_device_config, &imu_spi_handle);
    ESP_ERROR_CHECK(ret);

    vTaskDelay(pdMS_TO_TICKS(10));

    stmdev_ctx_t dev_ctx;
    dev_ctx.read_reg = stmdev_read;
    dev_ctx.write_reg = stmdev_write;
    dev_ctx.handle = imu_spi_handle;

    uint8_t whoamI;
    ism330dhcx_device_id_get(&dev_ctx, &whoamI);
    ESP_LOGI(TAG, "Received ID %02X, expected %02X", whoamI, ISM330DHCX_ID);

    ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        ism330dhcx_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Start device configuration. */
    ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Accelerometer Self Test */
    /* Set Output Data Rate */
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_52Hz);
    /* Set full scale */
    ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_4g);

    while(true) {
        int16_t data_raw[3];
        ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw);

        ESP_LOGI(TAG, "[%6d] [%6d] [%6d]", data_raw[0], data_raw[1], data_raw[2]);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
