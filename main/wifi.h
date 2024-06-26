#ifndef MAIN_WIFI_H_
#define MAIN_WIFI_H_
//-------------------------------------------------------------
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
//-------------------------------------------------------------
#define ESP_MAXIMUM_RETRY 5
void wifi_init_sta(void);
//-------------------------------------------------------------
#endif /* MAIN_WIFI_H_ */
