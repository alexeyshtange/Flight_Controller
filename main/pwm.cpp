#include "structs.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define LEDC_IO_3 27
#define LEDC_IO_2 13
#define LEDC_IO_1 12
#define LEDC_IO_0 14

#define PWM_RESOLUTION LEDC_TIMER_16_BIT
#define PWM_MAX 7536//942//7536
#define PWM_MIN 2621//328//2621
#define PWM_3RPM 2760
static const char *TAG = "pwm";

extern SemaphoreHandle_t ActualAngleMutex;
extern SemaphoreHandle_t TargetAngleMutex;
extern SemaphoreHandle_t pidSemaphore;
extern AnglesDegree ActualAngles, TargetAngles;


float compute_pid(float setpoint, float process_variable, PID_struct *pid_struct) {
    float error = setpoint - process_variable;
    float proportional = pid_struct->kp * error;
    pid_struct->integral += error;
    float integral = 0;
    if(pid_struct->ki<=0)
    	pid_struct->integral = 0;
    else
    integral = pid_struct->ki*pid_struct->integral;
    float derivative = pid_struct->kd * (error - pid_struct->error_prev);
    pid_struct->error_prev = error;
    pid_struct->control_signal = proportional + integral + derivative;
    return pid_struct->control_signal;
}

int16_t float_to_esc(float value){
	int16_t val =  ((value + 1) * (PWM_MAX - PWM_3RPM) / 2) + PWM_3RPM;
	if(val>PWM_MAX)
		return PWM_MAX;
	else if (val<PWM_3RPM)
		return PWM_3RPM;
	else
		return val;
}
float scale(float value, float min, float max, float new_min, float new_max){
	return ((value + max) * (new_max - new_min) / (max - min)) + new_min;
}

void InitPWM(ledc_channel_config_t *ledc_channel){
	ledc_timer_config_t ledc_timer;
	ledc_timer.duty_resolution = PWM_RESOLUTION;
	ledc_timer.freq_hz = 50;
	ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_timer.timer_num = LEDC_TIMER_0;
	ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  	  ledc_timer_config(&ledc_timer);

  	      ledc_channel[0].gpio_num = LEDC_IO_0, ledc_channel[0].channel = LEDC_CHANNEL_0, ledc_channel[0].duty = 0, ledc_channel[0].intr_type = LEDC_INTR_DISABLE, ledc_channel[0].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[0].timer_sel = LEDC_TIMER_0, ledc_channel[0].hpoint =0;
  	      ledc_channel[1].gpio_num = LEDC_IO_1, ledc_channel[1].channel = LEDC_CHANNEL_1, ledc_channel[1].duty = 0, ledc_channel[1].intr_type = LEDC_INTR_DISABLE, ledc_channel[1].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[1].timer_sel = LEDC_TIMER_0, ledc_channel[1].hpoint =0;
  	      ledc_channel[2].gpio_num = LEDC_IO_2, ledc_channel[2].channel = LEDC_CHANNEL_2, ledc_channel[2].duty = 0, ledc_channel[2].intr_type = LEDC_INTR_DISABLE, ledc_channel[2].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[2].timer_sel = LEDC_TIMER_0, ledc_channel[2].hpoint =0;
  	      ledc_channel[3].gpio_num = LEDC_IO_3, ledc_channel[3].channel = LEDC_CHANNEL_3, ledc_channel[3].duty = 0, ledc_channel[3].intr_type = LEDC_INTR_DISABLE, ledc_channel[3].speed_mode = LEDC_LOW_SPEED_MODE, ledc_channel[3].timer_sel = LEDC_TIMER_0, ledc_channel[3].hpoint =0;

  	  for (int i = 0; i < 4; i++) {
  	      ledc_channel_config(&ledc_channel[i]);
  	  }
    }
void SetChannelPWM(ledc_channel_config_t *ledc_channel, uint16_t width)
{
    ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, (width));
    ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
}
void SetDronePWM(ledc_channel_config_t *ledc_channel, uint16_t *width){
	for(uint8_t i = 0; i<4; i++)
		SetChannelPWM(&ledc_channel[i], width[i]);
}
void InitESC(ledc_channel_config_t *ledc_channel){
    uint16_t width[4] = {PWM_MAX, PWM_MAX, PWM_MAX, PWM_MAX};
    SetDronePWM(ledc_channel, width);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    for(uint8_t i = 0; i<4; i++)
    	width[i]=PWM_MIN;
    SetDronePWM(ledc_channel, width);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}
static void TimerCallbackPID(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(pidSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void InitTimerPID(void*){
	esp_timer_handle_t TimerPID;
	  const esp_timer_create_args_t TimerPIDargs = {
	      .callback = &TimerCallbackPID,
	      .arg = NULL,
	      .dispatch_method = ESP_TIMER_TASK,
	      .name = "TimerCallbackPID",
	      .skip_unhandled_events = false
	  };
	  esp_timer_create(&TimerPIDargs, &TimerPID);
	  esp_timer_start_periodic(TimerPID, 100000);
}
void TaskDroneControl(void* pvParameters) {
	ledc_channel_config_t *ledc_channel = (ledc_channel_config_t *)pvParameters;
	PID_struct PidX = {0.0,0.0,2.0,0.0,1.0,0.0}, PidY = {0.0,0.0,2.0,0.0,1.0,0.0};
	uint16_t width[4];
    for (;;) {
    	xSemaphoreTake(pidSemaphore, portMAX_DELAY);
    	xSemaphoreTake(ActualAngleMutex, portMAX_DELAY);
    	xSemaphoreTake(TargetAngleMutex, portMAX_DELAY);
    	PidX.kp = TargetAngles.P; PidY.kp = TargetAngles.P;
    	PidX.ki = TargetAngles.I; PidY.ki = TargetAngles.I;
    	PidX.kd = TargetAngles.D; PidY.kd = TargetAngles.D;
    	compute_pid(TargetAngles.X, ActualAngles.X, &PidX);	 compute_pid(TargetAngles.Y, ActualAngles.Y, &PidY);
    	xSemaphoreGive(ActualAngleMutex);
    	xSemaphoreGive(TargetAngleMutex);
        	width[0] = float_to_esc(scale(TargetAngles.Z,-90,90,-1,1)+scale(PidX.control_signal/2,-90,90,-1,1)-scale(PidY.control_signal/2,-90,90,-1,1));
        	width[1] = PWM_MIN;// = float_to_esc(scale(TargetAngles.Z,-90,90,-1,1)+scale(PidX.control_signal/2,-90,90,-1,1)+scale(PidY.control_signal/2,-90,90,-1,1));
        	width[2] = PWM_MIN;// = float_to_esc(scale(TargetAngles.Z,-90,90,-1,1)-scale(PidX.control_signal/2,-90,90,-1,1)+scale(PidY.control_signal/2,-90,90,-1,1));
        	width[3] = float_to_esc(scale(TargetAngles.Z,-90,90,-1,1)-scale(PidX.control_signal/2,-90,90,-1,1)-scale(PidY.control_signal/2,-90,90,-1,1));
        	SetDronePWM(ledc_channel, width);
        	//ESP_LOGI(TAG, "%d	%d\n", width[0], width[3]);
        	//ESP_LOGI(TAG, "%d	%d\n", width[1], width[2]);
        	//ESP_LOGI(TAG, "------------\n");
       // }
    }
}
