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
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include "midea.h"

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/*FreeRTOS event group to signal when is connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

void wifi_init(void);

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

   /*--------WIFI CONFI------------*/
   esp_err_t ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
   {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }

   ESP_ERROR_CHECK(ret);

   /*-------------------------------*/

   setup_gpio();
   setup_rtm_driver(GPIO_NUM_2, 0, midea_carrier_frequency);

   ir_tx_queue = xQueueCreate(1, sizeof(MideaFrameData));
   xTaskCreate(task_heart_beat, "heart-beat", 2048, NULL, 5, &heart_beat_task_handle);
   xTaskCreate(task_tx, "TX-task", 4096, NULL, 10, &tx_task_handle);

   tmr = xTimerCreate("heart-beat-timer", pdMS_TO_TICKS(1000), true, NULL, timer_callback);
   xTimerStart(tmr, pdMS_TO_TICKS(100));

   wifi_init();
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

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
   if (event_base == WIFI_EVENT)
   {
      if (event_id == WIFI_EVENT_STA_START)
      {
         esp_wifi_connect();
      }

      if (event_id == WIFI_EVENT_STA_DISCONNECTED)
      {
         if (s_retry_num < WIFI_MAX_RETRY)
         {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WIFI", "retry to connect to the AP");
         }
         else
         {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
         }

         ESP_LOGI("WIFI", "connect to the AP fail");
      }

      return;
   }

   if (event_base == IP_EVENT)
   {
      if (event_id == IP_EVENT_ETH_GOT_IP)
      {
         ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
         ESP_LOGI("WIFI", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
         s_retry_num = 0;
         xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      }
   }
}

void wifi_init(void)
{
   s_wifi_event_group = xEventGroupCreate();

   ESP_ERROR_CHECK(esp_netif_init());

   ESP_ERROR_CHECK(esp_event_loop_create_default());
   esp_netif_create_default_wifi_sta();

   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK(esp_wifi_init(&cfg));

   ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
   ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

   wifi_config_t wifi_config = {
       .sta = {
           .ssid = WIFI_SSID,
           .password = WIFI_PASS,
           /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
           .threshold.authmode = WIFI_AUTH_WPA2_PSK,

           .pmf_cfg = {
               .capable = true,
               .required = false},
       },
   };
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
   ESP_ERROR_CHECK(esp_wifi_start());

   ESP_LOGI("WIFI", "wifi_init_sta finished.");

   /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
   EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);

   /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
   if (bits & WIFI_CONNECTED_BIT)
   {
      ESP_LOGI("WIFI", "connected to ap SSID:%s password:%s",
               WIFI_SSID, WIFI_PASS);
   }
   else if (bits & WIFI_FAIL_BIT)
   {
      ESP_LOGI("WIFI", "Failed to connect to SSID:%s, password:%s",
               WIFI_SSID, WIFI_PASS);
   }
   else
   {
      ESP_LOGE("WIFI", "UNEXPECTED EVENT");
   }

   ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
   ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
   vEventGroupDelete(s_wifi_event_group);
}
