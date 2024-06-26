#include "structs.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "udp";

#define SERVER_IP "192.168.43.116"//"192.168.209.176"//
#define CLIENT_IP "192.168.209.100"
#define SERVER_PORT 4444
#define CLIENT_PORT 3333
#define ESP_WIFI_SSID "shtrundel"
#define ESP_WIFI_PASSWORD "12345678a"
#define ESP_MAXIMUM_RETRY 5

extern SemaphoreHandle_t ActualAngleMutex;
extern SemaphoreHandle_t TargetAngleMutex;
extern AnglesDegree ActualAngles, TargetAngles;
void unpack(AnglesDegree *Angles, char *package)
{
    memcpy(&(Angles->X), &package[0], sizeof(float));
    memcpy(&(Angles->Y), &package[4], sizeof(float));
    memcpy(&(Angles->Z), &package[8], sizeof(float));
    memcpy(&(Angles->P), &package[12], sizeof(float));
    memcpy(&(Angles->I), &package[16], sizeof(float));
    memcpy(&(Angles->D), &package[20], sizeof(float));
}
void pack(AnglesDegree *Angles, char *package)
{
    memcpy(&package[0], &(Angles->X), sizeof(float));
    memcpy(&package[4], &(Angles->Y), sizeof(float));
    memcpy(&package[8], &(Angles->Z), sizeof(float));
}
void InitUDP(ParamsUDP *paramsUDP){
	  ESP_LOGI(TAG, "UDP: Create socket...\n");
	  if ( (paramsUDP->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0 ) {
		  ESP_LOGE(TAG, "UDP: socket not created\n");
		 	 vTaskDelete(NULL);
		 	 }
		 	 memset(&paramsUDP->servaddr, 0, sizeof(paramsUDP->servaddr));
		 	 memset(&paramsUDP->cliaddr, 0, sizeof(paramsUDP->cliaddr));
		 	 //Заполнение информации о клиенте
		 	paramsUDP->cliaddr.sin_family = AF_INET; // IPv4
		 	paramsUDP->cliaddr.sin_addr.s_addr = (INADDR_ANY);
		 	paramsUDP->cliaddr.sin_port = htons( CLIENT_PORT);
		 	 ESP_LOGI(TAG, "UDP: CLIENT IP: %lu", paramsUDP->cliaddr.sin_addr.s_addr);
		 	 //Свяжем сокет с адресом клиента
		 	 if (bind(paramsUDP->sockfd, (const struct sockaddr *)&paramsUDP->cliaddr, sizeof(struct sockaddr_in)) < 0 )
		 	 {
		 	 ESP_LOGE(TAG, "UDP: socket not binded\n");
		 	 vTaskDelete(NULL);
		 	 }
		 	 ESP_LOGI(TAG, "UDP: socket was binded\n");
		 	 //Заполнение информации о сервере
		 	paramsUDP->servaddr.sin_family = AF_INET; // IPv4
		 	paramsUDP->servaddr.sin_addr.s_addr = inet_addr( SERVER_IP);
		 	paramsUDP->servaddr.sin_port = htons( SERVER_PORT);
}
void TaskSendUDP(void *pvParameters) {
	ParamsUDP *paramsUDP = (ParamsUDP *)pvParameters;
    char buff[12] = {};
    for (;;) {
    		xSemaphoreTake(ActualAngleMutex, portMAX_DELAY);
        	pack(&ActualAngles, buff);
        	xSemaphoreGive(ActualAngleMutex);
    	    sendto(paramsUDP->sockfd, buff, sizeof(buff),  0, (struct sockaddr*) &paramsUDP->servaddr,  sizeof(paramsUDP->servaddr));
    	    vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
void TaskRecvUDP(void *pvParameters) {
	ParamsUDP *paramsUDP = (ParamsUDP *)pvParameters;
    char buff[24] = {};
    for (;;) {
	    if(recv(paramsUDP->sockfd, buff, sizeof(buff), MSG_WAITALL)<0);
	    else {
	    	xSemaphoreTake(TargetAngleMutex, portMAX_DELAY);
	    	unpack(&TargetAngles, buff);
	    	xSemaphoreGive(TargetAngleMutex);
    }
}
}
