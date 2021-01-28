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

#include "midea.h"

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
   setup_rtm_driver(GPIO_NUM_2, 0, midea_carrier_frequency);

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
    //for while some static data
   static MideaFrameData data;
   data.protocol_id = 0xB2;
   data.state = STATE_ON;
   data.fan = FAN_MED;
   data.mode = MODE_COOL;
   data.temperature = T23C;
   
   xQueueSendFromISR(ir_tx_queue, &data, NULL);
}

void setup_gpio()
{
   gpio_config_t io_conf;
   /*
      setup gpio 2 as output
   */
   io_conf.intr_type = GPIO_INTR_DISABLE;
   io_conf.mode = GPIO_MODE_OUTPUT;
   io_conf.pin_bit_mask = (1ULL << GPIO_NUM_2);
   io_conf.pull_down_en = 0;
   io_conf.pull_up_en = 0;
   gpio_config(&io_conf);

   /*
      setup gpio 5 as input
   */
   io_conf.intr_type = GPIO_INTR_POSEDGE;
   io_conf.pin_bit_mask = (1ULL << GPIO_NUM_5);
   io_conf.mode = GPIO_MODE_INPUT;
   io_conf.pull_up_en = 1;

   gpio_config(&io_conf);
   gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_ANYEDGE);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(GPIO_NUM_5, gpio_isr_handler, NULL);
}

void task_tx(void *arg)
{
   MideaFrame frame;
   midea_tx_buffer_t tx_buffer;

   initialize_midea_tx_buffer(&tx_buffer[0]);

   for (;;)
   {
      if (xQueueReceive(ir_tx_queue, &frame.data, portMAX_DELAY))
      {
         ESP_LOGI("tx-task", "gpio-evt-received");

         ESP_LOGI("tx-task", "encodig data");
         midea_encode(frame.raw, &tx_buffer[1]);
         ESP_LOGI("tx-task", "data encoded!");

         ESP_LOGI("tx-task", "Tx start");
         rmt_write_items(0, tx_buffer, midea_tx_buffer_size, true);
         rmt_write_items(0, tx_buffer, midea_tx_buffer_size, true);
         ESP_LOGI("tx-task", "Tx end");
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
   config.tx_config.carrier_duty_percent = 50;
   config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
   config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
   config.tx_config.idle_output_en = true;

   ESP_ERROR_CHECK(rmt_config(&config));
   ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}
