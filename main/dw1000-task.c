#include "dw1000-task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "dw1000.h"
#include "dw1000-range.h"

#define TAG         "dw1000-task"

#define DW1000_HOST SPI3_HOST
#define DW1000_CS   GPIO_NUM_12
#define DW1000_SCLK GPIO_NUM_9
#define DW1000_MOSI GPIO_NUM_11
#define DW1000_MISO GPIO_NUM_10
#define DW1000_RSTn GPIO_NUM_3
#define DW1000_IRQ  GPIO_NUM_46

static QueueHandle_t dw1000_interrupt_queue;
static spi_device_handle_t dw1000_spi_handle;

static void IRAM_ATTR dw1000_irq_isr(void *arg) {
    int pin_number = (int)arg;
    xQueueSendFromISR(dw1000_interrupt_queue, &pin_number, NULL);
}

static void dwOpsInit(dwDevice_t *ctx) {
    spi_bus_config_t dw1000_bus_config = {
        .mosi_io_num = DW1000_MOSI,
        .miso_io_num = DW1000_MISO,
        .sclk_io_num = DW1000_SCLK,
        .quadwp_io_num = -1,
    };
    spi_device_interface_config_t dw1000_spi_config = {
        .address_bits = 8,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num = DW1000_CS,
        .queue_size = 1,
    };
    esp_err_t ret = spi_bus_initialize(DW1000_HOST, &dw1000_bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(DW1000_HOST, &dw1000_spi_config, &dw1000_spi_handle);
    ESP_ERROR_CHECK(ret);

    ctx->userdata = (void *)dw1000_spi_handle;
}

static void spiRead(dwDevice_t *ctx, const void *header, size_t headerLength, void* data, size_t dataLength) {
    uint8_t *hdr = (uint8_t *)header;
    uint32_t reg = hdr[0];
    if(headerLength > 1)
        reg = (reg << 8) | hdr[1];
    if(headerLength > 2)
        reg = (reg << 8) | hdr[2];
    ESP_LOGD(TAG, "spiRead(): reg = %06lX, reg_len = %d, len = %d", reg, headerLength, dataLength);
    spi_device_handle_t spi = (spi_device_handle_t)ctx->userdata;
    spi_transaction_ext_t t = {
        .base.flags = SPI_TRANS_VARIABLE_ADDR,
        .base.addr = reg,
        .address_bits = headerLength*8,
        .base.length = dataLength*8,
        .base.rxlength = dataLength*8,
        .base.rx_buffer = data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "spiRead() failed!");
    }
}

static void spiWrite(dwDevice_t *ctx, const void *header, size_t headerLength, const void* data, size_t dataLength) {
    uint8_t *hdr = (uint8_t *)header;
    uint32_t reg = hdr[0];
    if(headerLength > 1)
        reg = (reg << 8) | hdr[1];
    if(headerLength > 2)
        reg = (reg << 8) | hdr[2];
    ESP_LOGD(TAG, "spiWrite(): reg = %06lX, reg_len = %d, len = %d", reg, headerLength, dataLength);
    spi_device_handle_t spi = (spi_device_handle_t)ctx->userdata;
    spi_transaction_ext_t t = {
        .base.flags = SPI_TRANS_VARIABLE_ADDR,
        .base.addr = reg,
        .address_bits = headerLength*8,
        .base.length = dataLength*8,
        .base.tx_buffer = data,
    };
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "spiWrite() failed!");
    }
}

static void spiSetSpeed(dwDevice_t *ctx, dwSpiSpeed_t speed) {
    esp_err_t ret = spi_bus_remove_device(ctx->userdata);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dw1000_spi_config = {
        .address_bits = 8,
        .spics_io_num = DW1000_CS,
        .queue_size = 1,
    };

    switch(speed) {
        case dwSpiSpeedLow:
            ESP_LOGI(TAG, "Setting speed LOW");
            dw1000_spi_config.clock_speed_hz = 1 * 1000 * 1000;
            ret = spi_bus_add_device(DW1000_HOST, &dw1000_spi_config, &dw1000_spi_handle);
            ESP_ERROR_CHECK(ret);
            ctx->userdata = (void *)dw1000_spi_handle;
            break;

        case dwSpiSpeedHigh:
        default:
            ESP_LOGI(TAG, "Setting speed HIGH");
            dw1000_spi_config.clock_speed_hz = 20 * 1000 * 1000;
            ret = spi_bus_add_device(DW1000_HOST, &dw1000_spi_config, &dw1000_spi_handle);
            ESP_ERROR_CHECK(ret);
            ctx->userdata = (void *)dw1000_spi_handle;
            break;
    }
}

static void delayms(dwDevice_t *ctx, unsigned int delay) {
    vTaskDelay(pdMS_TO_TICKS(delay));
}

static void reset(dwDevice_t *dev) {
    ESP_LOGW(TAG, "Resetting DW1000");
    gpio_set_direction(DW1000_RSTn, GPIO_MODE_OUTPUT);
    gpio_set_level(DW1000_RSTn, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_direction(DW1000_RSTn, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void handleSent(dwDevice_t *ctx) {
    ESP_LOGI(TAG, "handleSent()");
}

void handleError(dwDevice_t *ctx) {
    ESP_LOGD(TAG, "handleError()");
}

void handleReceived(dwDevice_t *ctx) {
    ESP_LOGI(TAG, "handleReceived()");
    int dataLength = dwGetDataLength(ctx);
    uint8_t *data = (uint8_t *)malloc(dataLength);
    dwGetData(ctx, data, dataLength);
    ESP_LOGI(TAG, "Received %d bytes: [%02X] [%02X] [%02X] [%02X]", dataLength, data[0], data[1], data[2], data[3]);
    free(data);
}

void handleReceiveTimeout(dwDevice_t *ctx) {
    ESP_LOGI(TAG, "handleReceiveTimeout()");
}

void handleReceiveFailed(dwDevice_t *ctx) {
    ESP_LOGW(TAG, "handleReceiveFailed()");
}

void handleReceiveTimestampAvailable(dwDevice_t *ctx) {
    ESP_LOGI(TAG, "handleReceiveTimestampAvailable()");
}

void dw1000_task_func(void *args) {
    ESP_LOGI(TAG, "Starting DW1000 Task");

    dwDevice_t dwm_device;
    dwDevice_t *ctx = &dwm_device;
    dwOps_t dwOps = {
        .spiRead = spiRead,
        .spiWrite = spiWrite,
        .spiSetSpeed = spiSetSpeed,
        .delayms = delayms,
        .reset = reset,
    };

    dw1000_interrupt_queue = xQueueCreate(10, sizeof(int));

    ESP_ERROR_CHECK(gpio_intr_disable(DW1000_IRQ));
    ESP_ERROR_CHECK(gpio_reset_pin(DW1000_IRQ));
    ESP_ERROR_CHECK(gpio_set_direction(DW1000_IRQ, GPIO_MODE_INPUT));
    // ESP_ERROR_CHECK(gpio_set_pull_mode(DW1000_IRQ, GPIO_PULLDOWN_ONLY));  external pulldown is used
    ESP_ERROR_CHECK(gpio_set_intr_type(DW1000_IRQ, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(DW1000_IRQ, dw1000_irq_isr, (void *)DW1000_IRQ));

    dwInit(ctx, &dwOps);
    dwOpsInit(ctx);

    ctx->handleSent = handleSent;
    ctx->handleError = handleError;
    ctx->handleReceived = handleReceived;
    ctx->handleReceiveTimeout = handleReceiveTimeout;
    ctx->handleReceiveFailed = handleReceiveFailed;
    ctx->handleReceiveTimestampAvailable = handleReceiveTimestampAvailable;

    int result = dwConfigure(ctx);
    if (result == 0) {
        ESP_LOGI(TAG, "dwConfigure(): [OK]");
    } else {
        ESP_LOGI(TAG, "dwConfigure(): [ERROR]: %s", dwStrError(result));
    }

    ESP_LOGI(TAG, "dwGetDeviceId(): 0x%08lX", dwGetDeviceId(ctx));

    // dwTime_t delay = { .full = ANTENNA_DELAY/2 };
    // dwSetAntenaDelay(ctx, delay);

    // dwAttachSentHandler(ctx, txcallback);
    // dwAttachReceivedHandler(ctx, rxcallback);

    dwNewConfiguration(ctx);
    dwSetDefaults(ctx);
    dwSetSpiEdge(ctx, true);
    dwEnableMode(ctx, MODE_SHORTDATA_FAST_LOWPOWER);
    dwCommitConfiguration(ctx);

    dwEnableAllLeds(ctx);

    ESP_ERROR_CHECK(gpio_intr_enable(DW1000_IRQ));
    while(true) {
        // ESP_LOGI(TAG, "dwGetDeviceId(): 0x%08lX", dwGetDeviceId(ctx));

        // // transmit
        // dw_frame_blink_t txPacket;
        // dw_init_frame_blink(ctx, &txPacket);
        // dwNewTransmit(ctx);
        // dwSetData(ctx, (uint8_t*)&txPacket, sizeof(txPacket)+2);
        // dwStartTransmit(ctx);

        // receive
        dwNewReceive(ctx);
        dwStartReceive(ctx);

        int pin_number;
        if(xQueueReceive(dw1000_interrupt_queue, &pin_number, pdMS_TO_TICKS(100))) {
            dwHandleInterrupt(ctx);
        }
    }
}
