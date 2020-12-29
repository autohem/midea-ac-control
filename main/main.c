/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"


#include <midea.h>


/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

//#define BLINK_GPIO CONFIG_BLINK_GPIO


TimerHandle_t tmr = NULL;
QueueHandle_t ir_tx_queue = NULL;
TaskHandle_t tx_task_handle = NULL;
TaskHandle_t heart_beat_task_handle = NULL;
TaskHandle_t button_handler_task_handle = NULL;


//tasks
void task_heart_beat(void *arg);
void task_tx(void *arg);

//callbacks
void timer_callback(void *arg);
void button_callback(void *arg);


void app_main(void)
{
   ir_tx_queue = xQueueCreate(1,sizeof(MideaFrameData));
   xTaskCreate(task_heart_beat,"heart-beat",512,NULL,5,&heart_beat_task_handle);
   xTaskCreate(task_tx,"TX-task",1024,NULL,10,&tx_task_handle);

   tmr = xTimerCreate("heart-beat-timer",pdMS_TO_TICKS(1000),true,NULL,timer_callback);
   xTimerStart(tmr,pdMS_TO_TICKS(100));

}


void timer_callback(void *arg){
   vTaskNotifyGiveFromISR(heart_beat_task_handle,NULL);
   portYIELD_FROM_ISR();
}
