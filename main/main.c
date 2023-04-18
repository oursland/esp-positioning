/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "rom/ets_sys.h"

#include "dw1000-base.h"
#include "dw1000-hal.h"
#include "ism330dhcx_reg.h"

// LEDs are active low
#define LED1        GPIO_NUM_17
#define LED2        GPIO_NUM_8

#define IMU_HOST    SPI2_HOST
#define IMU_CS      GPIO_NUM_48
#define IMU_SCLK    GPIO_NUM_47
#define IMU_MISO    GPIO_NUM_14
#define IMU_MOSI    GPIO_NUM_21
#define IMU_INT1    GPIO_NUM_13
#define IMU_INT2    GPIO_NUM_45

#define DW1000_HOST SPI3_HOST
#define DW1000_CS   GPIO_NUM_12
#define DW1000_SCLK GPIO_NUM_9
#define DW1000_MOSI GPIO_NUM_11
#define DW1000_MISO GPIO_NUM_10
#define DW1000_RSTn GPIO_NUM_3
#define DW1000_IRQ  GPIO_NUM_46

int32_t stmdev_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
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

int32_t stmdev_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
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

void stmdev_mdelay(uint32_t millisec) {
    vTaskDelay(pdMS_TO_TICKS(millisec));
}

static void IRAM_ATTR dw_hal_isr(void *arg) {

}

void dw_hal_init(void) {
    ESP_ERROR_CHECK(gpio_reset_pin(DWM1000_IRQ));
    ESP_ERROR_CHECK(gpio_set_direction(DWM1000_IRQ, GPIO_MODE_INPUT));
    // ESP_ERROR_CHECK(gpio_set_pull_mode(DWM1000_IRQ, GPIO_PULLDOWN_ONLY));  external pulldown is used
    ESP_ERROR_CHECK(gpio_set_intr_type(DWM1000_IRQ, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(DWM1000_IRQ, dw_hal_isr, NULL));
}

void dw_hal_enable_interrupt(void) {
    gpio_intr_enable(DWM1000_IRQ);
}

void dw_hal_disable_interrupt(void) {
    gpio_intr_disable(DWM1000_IRQ);
}

void dw_hal_clear_pending_interrupt(void) {

}

void dw_spi_init(void) {

}

int32_t dw_spi_write(void *handle, uint64_t reg, uint32_t reg_len, uint8_t *data, uint16_t len) {
    spi_device_handle_t spi = (spi_device_handle_t)handle;
    spi_transaction_ext_t t = {
        .base.flags = SPI_TRANS_VARIABLE_ADDR,
        .base.addr = reg,
        .address_bits = reg_len*8,
        .base.length = len*8,
        .base.tx_buffer = (const void *)data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    if(err != ESP_OK) {
        return -1;
    }
    return 0;
}

int32_t dw_spi_read(void *handle, uint64_t reg, uint32_t reg_len, uint8_t *data, uint16_t len) {
    spi_device_handle_t spi = (spi_device_handle_t)handle;
    spi_transaction_ext_t t = {
        .base.flags = SPI_TRANS_VARIABLE_ADDR,
        .base.addr = reg,
        .address_bits = reg_len*8,
        .base.length = len*8,
        .base.rxlength = len*8,
        .base.rx_buffer = (void *)data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    if(err != ESP_OK) {
        return -1;
    }
    return 0;
}

void dw_udelay(uint32_t microseconds) {
    ets_delay_us(microseconds);
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // LEDS
    bool led1 = 0;
    bool led2 = 0;

    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED1, led1);

    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_level(LED2, led2);

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
    printf("Received ID %02X, expected %02X\n", whoamI, ISM330DHCX_ID);

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

    // Reset DWM1000
    gpio_set_direction(DW1000_RSTn, GPIO_MODE_OUTPUT);
    gpio_set_level(DW1000_RSTn, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_direction(DW1000_RSTn, GPIO_MODE_INPUT);

    // DW1000
    spi_device_handle_t dw1000_spi_handle;
    spi_bus_config_t dw1000_bus_config = {
        .mosi_io_num = DW1000_MOSI,
        .miso_io_num = DW1000_MISO,
        .sclk_io_num = DW1000_SCLK,
        .quadwp_io_num = -1,
    };
    spi_device_interface_config_t dw1000_device_config = {
        .address_bits = 8,
        .clock_speed_hz = SPI_MASTER_FREQ_20M,
        .spics_io_num = DW1000_CS,
        .queue_size = 1,
    };
    ret = spi_bus_initialize(DW1000_HOST, &dw1000_bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(DW1000_HOST, &dw1000_device_config, &dw1000_spi_handle);
    ESP_ERROR_CHECK(ret);

    dw1000_base_driver dw1000_ctx;
    dw_init(&dw1000_ctx, dw1000_spi_handle);
    dw_conf_print(&dw1000_ctx);

    vTaskDelay(pdMS_TO_TICKS(10));

    /* Wait stable output */
    stmdev_mdelay(100);

    for(;;) {
        int16_t data_raw[3];
        ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw);
        printf("[%6d] [%6d] [%6d]\n", data_raw[0], data_raw[1], data_raw[2]);
        vTaskDelay(pdMS_TO_TICKS(100));

        led1 = !led1;
        gpio_set_level(LED1, led1);
    }
}
