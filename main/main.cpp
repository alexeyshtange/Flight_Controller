#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "structs.h"
#include "wifi.h"
static const char *TAG = "main";
extern "C" {
	void app_main(void);
}

SemaphoreHandle_t ActualAngleMutex;
SemaphoreHandle_t TargetAngleMutex;
SemaphoreHandle_t pidSemaphore;
AnglesDegree ActualAngles, TargetAngles;
ParamsUDP paramsUDP;
ledc_channel_config_t ledc_channel[4];

extern void InitI2C(void*);
extern void TaskReadMPU6050(void*);
extern void wifi_init_sta(void*);
extern void wifi_check_task(void*);
extern void InitUDP(ParamsUDP *paramsUDP);
extern void TaskSendUDP(void *pvParameters);
extern void TaskRecvUDP(void *pvParameters);
extern void InitPWM(ledc_channel_config_t *ledc_channel);
extern void InitESC(ledc_channel_config_t *ledc_channel);
extern void TaskDroneControl(void* pvParameters);
extern void InitTimerPID(void*);

void app_main(void)
{
	ActualAngleMutex = xSemaphoreCreateMutex();
	TargetAngleMutex = xSemaphoreCreateMutex();
	pidSemaphore = xSemaphoreCreateBinary();
	InitPWM(ledc_channel);
	InitESC(ledc_channel);
	wifi_init_sta(NULL);
	InitI2C(NULL);
	InitUDP(&paramsUDP);
	InitTimerPID(NULL);
	xTaskCreate(&wifi_check_task, "wifi_check_task", 2048, NULL, 0, NULL);
    xTaskCreate(&TaskReadMPU6050, "read_mpu_task", 8192, NULL, 1, NULL);
    xTaskCreate(&TaskSendUDP, "send_udp_task", 2048, &paramsUDP, 2, NULL);
    xTaskCreate(&TaskRecvUDP, "receive_udp_task", 2048, &paramsUDP, 2, NULL);
    xTaskCreate(&TaskDroneControl, "drone_control_task", 4096, ledc_channel, 1, NULL);
}
