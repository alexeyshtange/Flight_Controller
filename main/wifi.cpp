#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
//-------------------------------------------------------------
#define SERVER_IP "192.168.43.116"//"192.168.209.176"//
#define CLIENT_IP "192.168.209.100"
#define SERVER_PORT 4444
#define CLIENT_PORT 3333
#define ESP_WIFI_SSID "shtrundel"
#define ESP_WIFI_PASSWORD "12345678a"
#define ESP_MAXIMUM_RETRY 5

static const char *TAG = "wifi";

static int s_retry_num = 0;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	      esp_wifi_connect();
	      else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
	        if (s_retry_num < ESP_MAXIMUM_RETRY) {
	            esp_wifi_connect();
	            s_retry_num++;
	            ESP_LOGI(TAG, "retry to connect to the AP");
	      }
	      else {
	            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
	        ESP_LOGI(TAG,"connect to the AP fail");
	      }
	      }
	      else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
	        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
	        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
	        s_retry_num = 0;
	        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	        gpio_set_level(GPIO_NUM_2, 1);
	      }
}

void wifi_init_sta(void*)
{
	wifi_ap_record_t info;
	esp_err_t ret;
	  gpio_reset_pin(GPIO_NUM_2);
	  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	  gpio_set_level(GPIO_NUM_2, 0);
	  //Initialize NVS
	  ret = nvs_flash_init();
	  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ret = nvs_flash_erase();
	    ESP_LOGI(TAG, "nvs_flash_erase: 0x%04x", ret);
	    ret = nvs_flash_init();
	    ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);
	  }
	  ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);

  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD,
            .threshold = {
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 ESP_WIFI_SSID);

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s",
            ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
void wifi_check_task(void*){
    while (1) {
    	wifi_ap_record_t info;
    	esp_err_t ret;
        ret = esp_wifi_sta_get_ap_info(&info);
        if(ret==0);
        else{
          gpio_set_level(GPIO_NUM_2, 0);
          wifi_init_sta(NULL);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
