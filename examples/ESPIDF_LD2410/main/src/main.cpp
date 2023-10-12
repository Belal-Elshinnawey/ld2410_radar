

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "radar_app.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "ld2410_radar.h"
ld2410_radar_data_payload_t report;
ld2410_radar_parameter_payload_t sensor_settings;
ld2410_radar_fw_payload_t fw_version;
radar_status_change_callback_t status_change_callback;
bool is_occupied = false;
#define RADAR_BUFFER_SIZE 255
static QueueHandle_t radar_uart_queue;

static void uart_rx_task(void *arg)
{
   uint8_t data_byte = 0;
   uint8_t oc = 0;
   while (1)
   {
      if (uart_read_bytes(CONFIG_RADAR_UART_NUM, &data_byte, 1, pdMS_TO_TICKS(100)) > 0)
      {
         if (xQueueSend(radar_uart_queue, &data_byte, pdMS_TO_TICKS(100)) == errQUEUE_FULL)
         {
            xQueueReceive(radar_uart_queue, &oc, pdMS_TO_TICKS(100));
            xQueueSend(radar_uart_queue, &data_byte, pdMS_TO_TICKS(100));
         }
      }
   }
}
int poll_radar_uart_queue(uint8_t *inByte)
{
   if (xQueueReceive(radar_uart_queue, (void *)inByte, pdMS_TO_TICKS(100)) == pdTRUE)
   {
      return 0;
   }
   return -1;
}
void write_to_uart(const uint8_t *buf, size_t len)
{
   size_t written_bytes = 0;
   do
   {
      written_bytes += uart_write_bytes(CONFIG_RADAR_UART_NUM, (const char *)buf, len - written_bytes);
   } while (written_bytes < len);
}
unsigned long get_uptime_ms()
{
   return (unsigned long)(esp_timer_get_time() / 1000ULL);
}
ld2410_radar_ack_results_t change_ld2410_sensor_baudrate(unsigned long long bps)
{
   // Write baudrate command
   if (ld2410_radar_set_baudrate(bps) == ld2410_radar_ack_results_success)
   {
      ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, bps));
      return ld2410_radar_ack_results_success;
   }
   else
   {
      return ld2410_radar_ack_results_fail;
   }
}
unsigned long long find_ld2410_current_baudrate()
{
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 115200ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 115200ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 256000ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 256000ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 9600ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 9600ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 19200ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 19200ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 38400ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 38400ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 57600ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 57600ULL;
   }
   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 230400ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 230400ULL;
   }

   ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 460800ULL));
   ld2410_radar_enter_command_mode();
   ld2410_radar_exit_command_mode();
   if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
   {
      return 460800ULL;
   }
   return 0;
}
void radar_app_loop(void *params)
{
   unsigned long long current_br = find_ld2410_current_baudrate();
   while (current_br == 0)
   {
      current_br = find_ld2410_current_baudrate();
   }
   ESP_LOGW("RADAR", "current baudrate %llu\n", current_br);
   if (current_br != 9600ULL)
   {
      while (ld2410_radar_factory_reset() != ld2410_radar_ack_results_success)
      {
      }
      ESP_LOGW("RADAR", "Factory reset Success\n");
      ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_RADAR_UART_NUM, 256000ULL));
      while (change_ld2410_sensor_baudrate(9600ULL) == ld2410_radar_ack_results_fail)
      {
         vTaskDelay(pdMS_TO_TICKS(10));
      }
   }
   if (ld2410_radar_get_fw_version(&fw_version) == ld2410_radar_ack_results_success)
   {
      ESP_LOGW("RADAR", "FW Read Success %02x.%02x.%02x%02x%02x%02x\n",
               fw_version.major_version,
               fw_version.minor_version,
               fw_version.bugfix_version1,
               fw_version.bugfix_version2,
               fw_version.bugfix_version3,
               fw_version.bugfix_version4);
   }
   else
   {
      ESP_LOGW("RADAR", "FW Read Failed\n");
   }
   while (ld2410_radar_bluetooth_off() != ld2410_radar_ack_results_success)
   {
      ESP_LOGW("RADAR", "Bluetooth off failed\n");
   }
   ESP_LOGW("RADAR", "Bluetooth off success\n");
   // while (ld2410_radar_start_engineering_mode() == ld2410_radar_ack_results_fail)
   // {
   // }
   for (;;)
   {
      if (ld2410_radar_read_data_frame(&report) != ld2410_radar_ack_results_success)
      {
         continue;
      }

      if (report.measurement_value.basic_values.data_type == 0x02)
      {
         ESP_LOGW("RADAR", "Basic State: %d,", report.measurement_value.basic_values.state);
         ESP_LOGW("RADAR", " Moving energy: %d,", report.measurement_value.basic_values.moving_energy);
         ESP_LOGW("RADAR", " Moving distance: %d,", report.measurement_value.basic_values.moving_distance);
         ESP_LOGW("RADAR", " Stationary energy: %d,", report.measurement_value.basic_values.stationary_energy);
         ESP_LOGW("RADAR", " Stationary distance: %d,", report.measurement_value.basic_values.stationary_distance);
         ESP_LOGW("RADAR", " Detection distance: %d\n", report.measurement_value.basic_values.detection_distance);
      }
      else if (report.measurement_value.engineering_values.data_type == 0x01)
      {
         ESP_LOGW("RADAR", "Engineering State: %d,", report.measurement_value.engineering_values.state);
         ESP_LOGW("RADAR", " Moving energy: %d,", report.measurement_value.engineering_values.moving_energy);
         ESP_LOGW("RADAR", " Moving distance: %d,", report.measurement_value.engineering_values.moving_distance);
         ESP_LOGW("RADAR", " Stationary energy: %d,", report.measurement_value.engineering_values.stationary_energy);
         ESP_LOGW("RADAR", " Stationary distance: %d,", report.measurement_value.engineering_values.stationary_distance);
         ESP_LOGW("RADAR", " Detection distance: %d", report.measurement_value.engineering_values.detection_distance);
         ESP_LOGW("RADAR", " Photo Sensor: %d\n", report.measurement_value.engineering_values.photo_sensor);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
   }
}
void app_main(void)
{
   status_change_callback = status_change_cb;
   const uart_config_t uart_config = {
       .baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .source_clk = UART_SCLK_APB,
   };
   uart_driver_install(CONFIG_RADAR_UART_NUM, RADAR_BUFFER_SIZE * 2, 0, 0, NULL, 0);
   uart_param_config(CONFIG_RADAR_UART_NUM, &uart_config);
   uart_set_pin(CONFIG_RADAR_UART_NUM, CONFIG_RADAR_UART_TX, CONFIG_RADAR_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   radar_uart_queue = xQueueCreate(RADAR_BUFFER_SIZE, sizeof(uint8_t));
   xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, configMAX_PRIORITIES, NULL);
   ld2410_radar_init_callbacks(get_uptime_ms, poll_radar_uart_queue, write_to_uart);
   xTaskCreatePinnedToCore(radar_app_loop, "radar_app_loop", 2048, (void *)NULL, configMAX_PRIORITIES - 5, NULL, 0);
}
