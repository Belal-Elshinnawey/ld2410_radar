#include "ld2410_radar.h"
#include <cppQueue.h> //Queue Lib by SMFSW

#define LD2410_SERIAL_TX 17
#define LD2410_SERIAL_RX 16

#ifdef __AVR__
  #include <SoftwareSerial.h>
  SoftwareSerial Serial1 (LD2410_SERIAL_RX, LD2410_SERIAL_TX);
  #define LD2410_SERIAL Serial1
#elif ESP32
  #define LD2410_SERIAL Serial1
#endif

cppQueue  q(1, 255, FIFO, true);

ld2410_radar_data_payload_t report;
ld2410_radar_parameter_payload_t sensor_settings;
ld2410_radar_fw_payload_t fw_version;


void write_to_uart(const uint8_t *buf, size_t len){
  LD2410_SERIAL.write(buf, len);
}

int pull_radar_uart_queue(uint8_t *inByte)
{
  if(!q.isEmpty()){
        q.pop(inByte);
        return 0;
   }
  return -1;
}

void set_serial_baudrate(unsigned long long bps){
  #ifdef __AVR__
    LD2410_SERIAL.end();
    LD2410_SERIAL.begin(bps);
#elif ESP32
  LD2410_SERIAL.end();
  LD2410_SERIAL.begin(bps, SERIAL_8N1, LD2410_SERIAL_RX, LD2410_SERIAL_TX);
#endif
}

ld2410_radar_ack_results_t change_ld2410_sensor_baudrate(unsigned long long bps){
  if (ld2410_radar_set_baudrate(bps) == ld2410_radar_ack_results_success)
  {
    set_serial_baudrate(bps);
    return ld2410_radar_ack_results_success;
  }
  else
  {
    return ld2410_radar_ack_results_fail;
  }
}

unsigned long long find_ld2410_current_baudrate()
{
  set_serial_baudrate(115200ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 115200ULL;
  }
  set_serial_baudrate(256000ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 256000ULL;
  }
  set_serial_baudrate(9600ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 9600ULL;
  }
  set_serial_baudrate(19200ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 19200ULL;
  }
  set_serial_baudrate(38400ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 38400ULL;
  }
  set_serial_baudrate(57600ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 57600ULL;
  }
  set_serial_baudrate(230400ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 230400ULL;
  }
  set_serial_baudrate(460800ULL);
  ld2410_radar_enter_command_mode();
  ld2410_radar_exit_command_mode();
  if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
  {
    return 460800ULL;
  }
  return 0;
}

#ifdef __AVR__
  uint8_t rcv;
  uint8_t oc;
  void serialEvent() {
    while  (LD2410_SERIAL.available())          // Ensure Serial is available
    {
      rcv = (uint8_t) LD2410_SERIAL.read();  // Read char from Serial
      if(q.isFull()){
        q.pop(&oc);
      }
      if(rcv != -1){
        q.push(&rcv);
      }
    }
  }
#elif ESP32
  uint8_t rcv;
  uint8_t oc;
  void serialEvent(void * params) {
    while(true){
      while  (LD2410_SERIAL.available())          // Ensure Serial is available
      {
        rcv = (uint8_t) LD2410_SERIAL.read();  // Read char from Serial
        if(q.isFull()){
          q.pop(&oc);
        }
        if(rcv != -1){
          q.push(&rcv);
        }
      }
      delay(1);
    }
  }
#endif

void setup() {
  Serial.begin(115200);
  ld2410_radar_init_callbacks(millis, pull_radar_uart_queue, write_to_uart);
#ifdef __AVR__
    LD2410_SERIAL.begin(256000ULL);
#elif ESP32
   LD2410_SERIAL.begin(256000ULL, SERIAL_8N1, LD2410_SERIAL_RX, LD2410_SERIAL_TX);
   xTaskCreate(serialEvent, "serial_task", 2048, NULL, 5, NULL);
#endif
  Serial.print("Starting\n");
  // Check current baudrate
  unsigned long long current_br = find_ld2410_current_baudrate();
  while (current_br == 0)
  {
    current_br = find_ld2410_current_baudrate();
  }
  Serial.print("current baudrate ");
  Serial.println((int)current_br);
  if (current_br != 115200ULL)
  {
    while (ld2410_radar_factory_reset() != ld2410_radar_ack_results_success)
    {
    }
    Serial.print("Factory reset Success\n");
    set_serial_baudrate(256000ULL);
    while (change_ld2410_sensor_baudrate(115200ULL) == ld2410_radar_ack_results_fail)
    {
      delay(10);
    }
  }

  if (ld2410_radar_get_fw_version(&fw_version) == ld2410_radar_ack_results_success)
  {
    Serial.print("FW Read Success ");
    Serial.print(fw_version.major_version);
    Serial.print(".");
    Serial.print(fw_version.minor_version);
    Serial.print(".");
    Serial.print(fw_version.bugfix_version1);
    Serial.print(fw_version.bugfix_version2);
    Serial.print(fw_version.bugfix_version3);
    Serial.println(fw_version.bugfix_version4);
  }
  else
  {
    Serial.print("FW Read Failed\n");
  }

  while (ld2410_radar_bluetooth_off() != ld2410_radar_ack_results_success)
  {
    Serial.print("Bluetooth off failed\n");
  }
  Serial.print("Bluetooth off success\n");
  delay(1000);
// //Engineering Mode: use ld2410_radar_stop_engineering_mode to return to basic mode
//  while(ld2410_radar_start_engineering_mode != ld2410_radar_ack_results_success){
//    
//  }
}

void loop() {
    if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
    {
       if (report.measurement_value.basic_values.data_type == 0x02)
      {
        Serial.print("Basic_State: ");
        Serial.print(report.measurement_value.basic_values.state);
        Serial.print(", Moving_energy: ");
        Serial.print(report.measurement_value.basic_values.moving_energy);
        Serial.print(", Moving_distance: ");
        Serial.print(report.measurement_value.basic_values.moving_distance);
        Serial.print(", Stationary_energy: ");
        Serial.print(report.measurement_value.basic_values.stationary_energy);
        Serial.print(", Stationary_distance: ");
        Serial.print(report.measurement_value.basic_values.stationary_distance);
        Serial.print(", Detection_distance: ");
        Serial.println(report.measurement_value.basic_values.detection_distance);
      }
      else if (report.measurement_value.engineering_values.data_type == 0x01)
      {
        Serial.print("Engineering State: ");
        Serial.println(report.measurement_value.engineering_values.state);
        Serial.print(" Moving energy: ");
        Serial.println(report.measurement_value.engineering_values.moving_energy);
        Serial.print(" Moving distance: ");
        Serial.println(report.measurement_value.engineering_values.moving_distance);
        Serial.print(" Stationary energy: ");
        Serial.println(report.measurement_value.engineering_values.stationary_energy);
        Serial.print(" Stationary distance: ");
        Serial.println(report.measurement_value.engineering_values.stationary_distance);
        Serial.print(" Detection distance: ");
        Serial.println(report.measurement_value.engineering_values.detection_distance);
        Serial.print(" Photo Sensor: ");
        Serial.println(report.measurement_value.engineering_values.photo_sensor);
      }
    }
}
