#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "common.h"
#include "dw1000-task.h"
#include "ftm-task.h"
#include "ism330dhcx-task.h"

#define TAG         "app_main"

// LEDs are active low
#define LED1        GPIO_NUM_17
#define LED2        GPIO_NUM_8

void app_main()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BASE));
    ESP_LOGI(TAG, "MAC address: [%02X:%02X:%02X:%02X:%02X:%02X]", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // LEDS
    bool led1 = 0;
    bool led2 = 0;

    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED1, led1);

    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_level(LED2, led2);

    task_t ism330dhcx_task;
    task_t ftm_task;
    task_t dw1000_task;

    QueueHandle_t main_queue = xQueueCreate(10, sizeof(event_type_t));

    ism330dhcx_task.task_queue = xQueueCreate(10, sizeof(event_type_t));
    ism330dhcx_task.main_queue = main_queue;
    ism330dhcx_task.task_handle = xTaskCreate(ism330dhcx_task_func, "ism330dhcx_task", 4096, &ism330dhcx_task, 1, NULL);

    ftm_task.task_queue = xQueueCreate(10, sizeof(event_type_t));
    ftm_task.main_queue = main_queue;
    ftm_task.task_handle = xTaskCreate(ftm_task_func, "ftm_task", 4096, &ftm_task, 1, NULL);

    dw1000_task.task_queue = xQueueCreate(10, sizeof(event_type_t));
    dw1000_task.main_queue = main_queue;
    dw1000_task.task_handle = xTaskCreate(dw1000_task_func, "dw1000_task", 4096, &dw1000_task, 1, NULL);

    for(;;) {
        led1 = !led1;
        gpio_set_level(LED1, led1);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
