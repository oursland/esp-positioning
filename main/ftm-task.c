#include "ftm-task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#define TAG         "ftm-task"

#define ETH_ALEN 6
#define MAX_CONNECT_RETRY_ATTEMPTS  5

static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;
static const int SCANNING_COMPLETE_BIT = BIT2;
static const int FTM_COMPLETE_BIT = BIT3;

static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;

static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est;
static uint32_t s_dist_est;

static bool s_ap_started;
static uint8_t s_ap_channel;

static uint8_t channel;

static bool s_reconnect = true;
static int s_retry_num = 0;

const int g_report_lvl =
// #ifdef CONFIG_ESP_FTM_REPORT_SHOW_DIAG
//     BIT0 |
// #endif
// #ifdef CONFIG_ESP_FTM_REPORT_SHOW_RTT
//    BIT1 |
// #endif
// #ifdef CONFIG_ESP_FTM_REPORT_SHOW_T1T2T3T4
//     BIT2 |
// #endif
// #ifdef CONFIG_ESP_FTM_REPORT_SHOW_RSSI
//     BIT3 |
// #endif
    0;

static uint16_t g_scan_ap_num;
static wifi_ap_record_t *g_ap_list_buffer;
static uint8_t s_ap_bssid[ETH_ALEN];

static void ftm_process_report(void) {
    if (!g_report_lvl)
        return;

    char *log = malloc(200);
    if (!log) {
        ESP_LOGE(TAG, "Failed to alloc buffer for FTM report");
        return;
    }

    bzero(log, 200);
    sprintf(log, "%s%s%s%s", g_report_lvl & BIT0 ? " Diag |":"", g_report_lvl & BIT1 ? "   RTT   |":"",
        g_report_lvl & BIT2 ? "       T1       |       T2       |       T3       |       T4       |":"",
        g_report_lvl & BIT3 ? "  RSSI  |":"");
    ESP_LOGI(TAG, "FTM Report:");
    ESP_LOGI(TAG, "|%s", log);
    for(int i = 0; i < s_ftm_report_num_entries; i++) {
        char *log_ptr = log;

        bzero(log, 200);
        if (g_report_lvl & BIT0) {
            log_ptr += sprintf(log_ptr, "%6d|", s_ftm_report[i].dlog_token);
        }
        if (g_report_lvl & BIT1) {
            log_ptr += sprintf(log_ptr, "%7" PRIu32 "  |", s_ftm_report[i].rtt);
        }
        if (g_report_lvl & BIT2) {
            log_ptr += sprintf(log_ptr, "%14llu  |%14llu  |%14llu  |%14llu  |", s_ftm_report[i].t1,
                                        s_ftm_report[i].t2, s_ftm_report[i].t3, s_ftm_report[i].t4);
        }
        if (g_report_lvl & BIT3) {
            log_ptr += sprintf(log_ptr, "%6d  |", s_ftm_report[i].rssi);
        }
        ESP_LOGI(TAG, "|%s", log);
    }
    free(log);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch(event_id) {
        case WIFI_EVENT_STA_CONNECTED:
            wifi_event_sta_connected_t *sta_event = (wifi_event_sta_connected_t *)event_data;

            ESP_LOGI(TAG, "Connected to %s (BSSID: "MACSTR", Channel: %d)", sta_event->ssid,
                MAC2STR(sta_event->bssid), sta_event->channel);

            memcpy(s_ap_bssid, sta_event->bssid, ETH_ALEN);
            s_ap_channel = sta_event->channel;
            xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_reconnect && ++s_retry_num < MAX_CONNECT_RETRY_ATTEMPTS) {
                ESP_LOGI(TAG, "sta disconnect, retry attempt %d...", s_retry_num);
                esp_wifi_connect();
            } else {
                ESP_LOGI(TAG, "sta disconnected");
            }
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
            break;

        case WIFI_EVENT_SCAN_DONE:
            xEventGroupSetBits(s_wifi_event_group, SCANNING_COMPLETE_BIT);
            break;

        case WIFI_EVENT_FTM_REPORT:
            wifi_event_ftm_report_t *ftm_event = (wifi_event_ftm_report_t *)event_data;

            if(ftm_event->status == FTM_STATUS_SUCCESS) {
                s_rtt_est = ftm_event->rtt_est;
                s_dist_est = ftm_event->dist_est;
                s_ftm_report = ftm_event->ftm_report_data;
                s_ftm_report_num_entries = ftm_event->ftm_report_num_entries;
                xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
            } else {
                ESP_LOGI(TAG, "FTM procedure with Peer("MACSTR") failed! (Status - %d)",
                    MAC2STR(ftm_event->peer_mac), ftm_event->status);
                xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
            }
            xEventGroupClearBits(s_wifi_event_group, FTM_COMPLETE_BIT);
            break;

        case WIFI_EVENT_AP_START:
            s_ap_started = true;
            break;

        case WIFI_EVENT_AP_STOP:
            s_ap_started = false;
            break;

        default:
    }
}

static bool wifi_perform_scan(const char *ssid, bool print_results) {
    wifi_scan_config_t scan_config = { 0 };
    scan_config.ssid = (uint8_t *)ssid;
    uint8_t i;

    if(ESP_OK != esp_wifi_scan_start(&scan_config, true)) {
        ESP_LOGI(TAG, "Failed to perform scan");
        return false;
    }

    esp_wifi_scan_get_ap_num(&g_scan_ap_num);
    if(g_scan_ap_num == 0) {
        ESP_LOGI(TAG, "No matching AP found");
        return false;
    }

    if(g_ap_list_buffer) {
        free(g_ap_list_buffer);
    }
    g_ap_list_buffer = (wifi_ap_record_t *)malloc(g_scan_ap_num * sizeof(wifi_ap_record_t));
    if(g_ap_list_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to malloc buffer to print scan results");
        return false;
    }

    if(esp_wifi_scan_get_ap_records(&g_scan_ap_num, (wifi_ap_record_t *)g_ap_list_buffer) == ESP_OK) {
        if(print_results) {
            for(i = 0; i < g_scan_ap_num; i++) {
                ESP_LOGI(TAG, "[%s][rssi=%d]""%s",
                    g_ap_list_buffer[i].ssid,
                    g_ap_list_buffer[i].rssi,
                    g_ap_list_buffer[i].ftm_responder ? "[FTM Responder]" : "");
            }
        }
    }

    ESP_LOGI(TAG, "sta scan done");

    return true;
}

void ftm_task_func(void *args) {
    ESP_LOGI(TAG, "Starting FTM Task");

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BASE));
    ESP_LOGI(TAG, "MAC address: [%02X:%02X:%02X:%02X:%02X:%02X]", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    s_ftm_event_group = xEventGroupCreate();

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t ap_config = {
        .ap.max_connection = 4,
        .ap.authmode = WIFI_AUTH_WPA2_PSK,
        .ap.ftm_responder = true,
    };
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "ftm-%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
    snprintf((char *)ap_config.ap.password, MAX_PASSPHRASE_LEN, "ftm-%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "Starting AP using SSID: [%s]", ap_config.ap.ssid);
    ESP_LOGI(TAG, "Starting AP using password: [%s]", ap_config.ap.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
    // ESP_ERROR_CHECK(esp_wifi_ftm_resp_set_offset(-100));

    wifi_second_chan_t second;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&channel, &second));

    QueueHandle_t event_queue = xQueueCreate(10, sizeof(uint32_t));

    // promiscuous mode?
    // esp_err_t esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb_t cb);
    // esp_err_t esp_wifi_set_promiscuous(bool en);
    // esp_err_t esp_wifi_set_promiscuous_filter(const wifi_promiscuous_filter_t *filter);
    // esp_err_t esp_wifi_set_promiscuous_ctrl_filter(const wifi_promiscuous_filter_t *filter);

    // wifi_perform_scan(NULL, true);

    wifi_scan_config_t scan_config = { 0 };
    scan_config.channel = channel;
    ESP_LOGI(TAG, "WiFi channel [%d]", channel);

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if(ESP_OK != esp_wifi_scan_start(&scan_config, true)) {
            ESP_LOGI(TAG, "Failed to perform scan");
        }
        esp_wifi_scan_get_ap_num(&g_scan_ap_num);
        if(g_scan_ap_num == 0) {
            ESP_LOGI(TAG, "No matching AP found");
            continue;
        }

        if(g_ap_list_buffer) {
            free(g_ap_list_buffer);
        }
        g_ap_list_buffer = (wifi_ap_record_t *)malloc(g_scan_ap_num * sizeof(wifi_ap_record_t));
        if(g_ap_list_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to malloc buffer to print scan results");
            continue;
        }

        if(esp_wifi_scan_get_ap_records(&g_scan_ap_num, (wifi_ap_record_t *)g_ap_list_buffer) == ESP_OK) {
            for(int i = 0; i < g_scan_ap_num; i++) {
                if(!strncmp("ftm-", (char *)g_ap_list_buffer[i].ssid, 4)) {
                    ESP_LOGI(TAG, "[%s][rssi=%d]""%s",
                        g_ap_list_buffer[i].ssid,
                        g_ap_list_buffer[i].rssi,
                        g_ap_list_buffer[i].ftm_responder ? "[FTM Responder]" : "");

                    wifi_ftm_initiator_cfg_t ftmi_cfg = {
                        .frm_count = 8,
                        .burst_period = 2,
                        .channel = channel,
                    };
                    memcpy(ftmi_cfg.resp_mac, g_ap_list_buffer[i].bssid, ETH_ALEN);

                    xEventGroupClearBits(s_wifi_event_group, FTM_COMPLETE_BIT);
                    if (ESP_OK != esp_wifi_ftm_initiate_session(&ftmi_cfg)) {
                        ESP_LOGE(TAG, "Failed to start FTM session");
                    }

                    EventBits_t bits = xEventGroupWaitBits(
                        s_ftm_event_group,
                        FTM_REPORT_BIT | FTM_FAILURE_BIT,
                        pdTRUE, pdFALSE, portMAX_DELAY);
                    if(bits & FTM_REPORT_BIT) {
                        ftm_process_report();
                        free(s_ftm_report);
                        s_ftm_report = NULL;
                        s_ftm_report_num_entries = 0;
                        ESP_LOGI(TAG, "Estimated RTT - %" PRId32 " nSec, Estimated Distance - %" PRId32 ".%02" PRId32 " meters",
                            s_rtt_est, s_dist_est / 100, s_dist_est % 100);
                    }
                }
            }
        }
    }
}
