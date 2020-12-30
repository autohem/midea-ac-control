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
#include "driver/rmt.h"
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
static void IRAM_ATTR gpio_isr_handler(void *arg);

//config functions
void setup_gpio(void);
void setup_rtm_driver(gpio_num_t pin, rmt_channel_t channel, uint32_t carrier_frequency);

void app_main(void)
{

   setup_gpio();
   setup_rtm_driver(GPIO_NUM_2,0,midea_carrier_frequency);
   
   ir_tx_queue = xQueueCreate(1, sizeof(MideaFrameData));
   xTaskCreate(task_heart_beat, "heart-beat", 2048, NULL, 5, &heart_beat_task_handle);
   xTaskCreate(task_tx, "TX-task", 4096, NULL, 10, &tx_task_handle);

   tmr = xTimerCreate("heart-beat-timer", pdMS_TO_TICKS(1000), true, NULL, timer_callback);
   xTimerStart(tmr, pdMS_TO_TICKS(100));
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

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
   MideaFrameData data;
   xQueueSendFromISR(ir_tx_queue, &data, NULL);
}

void setup_gpio()
{
   gpio_config_t io_conf;
   //disable interrupt
   io_conf.intr_type = GPIO_INTR_DISABLE;
   //set as output mode
   io_conf.mode = GPIO_MODE_OUTPUT;
   //bit mask of the pins that you want to set,e.g.GPIO18/19
   io_conf.pin_bit_mask = (1ULL << GPIO_NUM_2);
   //disable pull-down mode
   io_conf.pull_down_en = 0;
   //disable pull-up mode
   io_conf.pull_up_en = 0;
   //configure GPIO with the given settings
   gpio_config(&io_conf);

   //interrupt of rising edge
   io_conf.intr_type = GPIO_INTR_POSEDGE;
   //bit mask of the pins, use GPIO4/5 here
   io_conf.pin_bit_mask = (1ULL << GPIO_NUM_14);
   //set as input mode
   io_conf.mode = GPIO_MODE_INPUT;
   //enable pull-up mode
   io_conf.pull_up_en = 1;
   gpio_config(&io_conf);

   //change gpio intrrupt type for one pin
   gpio_set_intr_type(GPIO_NUM_14, GPIO_INTR_ANYEDGE);

   //install gpio isr service
   gpio_install_isr_service(0);
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(GPIO_NUM_14, gpio_isr_handler, NULL);

   //remove isr handler for gpio number.
   gpio_isr_handler_remove(GPIO_NUM_2);
   //hook isr handler for specific gpio pin again
   gpio_isr_handler_add(GPIO_NUM_14, gpio_isr_handler, NULL);
}

void task_tx(void *arg)
{
   MideaFrameData data;
   for (;;)
   {
      if (xQueueReceive(ir_tx_queue, &data, portMAX_DELAY))
      {
         ESP_LOGI("tx-task", "gpio-evt-received");
      }
   }
}

void setup_rtm_driver(gpio_num_t pin, rmt_channel_t channel, uint32_t carrier_frequency)
{
   rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin, channel);
   config.tx_config.carrier_en = true;
   config.tx_config.carrier_freq_hz = carrier_frequency;
   config.clk_div = 80;
   config.tx_config.loop_en = false;
   config.tx_config.carrier_duty_percent = 75;
   config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
   config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
   config.tx_config.idle_output_en = true;

   ESP_ERROR_CHECK(rmt_config(&config));
   ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}
