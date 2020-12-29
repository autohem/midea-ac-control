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

//interrupt handlers
void IRAM_ATTR gpio_isr_handler(void *arg);

//config functions
void gpio_input_config(gpio_num_t pin);

void app_main(void)
{
   ir_tx_queue = xQueueCreate(1, sizeof(MideaFrameData));
   xTaskCreate(task_heart_beat, "heart-beat", 2048, NULL, 5, &heart_beat_task_handle);
   xTaskCreate(task_tx,"TX-task",1024,NULL,10,&tx_task_handle);

   tmr = xTimerCreate("heart-beat-timer", pdMS_TO_TICKS(1000), true, NULL, timer_callback);
   xTimerStart(tmr, pdMS_TO_TICKS(100));
   gpio_input_config(GPIO_NUM_14);
}

void timer_callback(void *arg)
{
   vTaskNotifyGiveFromISR(heart_beat_task_handle, NULL);
   portYIELD_FROM_ISR();
}

void task_heart_beat(void *arg)
{
   for (;;)
   {
      if (0 < ulTaskNotifyTake(pdFALSE, portMAX_DELAY))
      {
         ESP_LOGI("heart-beat", "alive");
      }
   }
}

void IRAM_ATTR gpio_isr_handler(void *arg)
{
   xQueueSendFromISR(ir_tx_queue, NULL, NULL);
}

void gpio_input_config(gpio_num_t pin)
{
   gpio_config_t io_conf;
   //interrupt of rising edge
   io_conf.intr_type = GPIO_INTR_POSEDGE;
   //bit mask of the pins, use GPIO4/5 here
   io_conf.pin_bit_mask = pin;
   //set as input mode
   io_conf.mode = GPIO_MODE_INPUT;
   //enable pull-up mode
   io_conf.pull_up_en = 1;
   gpio_config(&io_conf);

   //install gpio isr service
   gpio_install_isr_service(0);
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(pin, gpio_isr_handler, (void *)pin);
}

void task_tx(void *arg){
   for(;;){

      if(xQueueReceive(ir_tx_queue, NULL, portMAX_DELAY)) {
             ESP_LOGI("tx-task", "gpio-evt-received");
        }
   }
}