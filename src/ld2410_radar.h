
#ifndef LD2410_RADAR_H__
#define LD2410_RADAR_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include <stddef.h>

#define LD2410_RADAR_RX_NULL 0
#define LD2410_RADAR_RX_HDR_1 1
#define LD2410_RADAR_RX_HDR_2 2
#define LD2410_RADAR_RX_HDR_3 3
#define LD2410_RADAR_RX_DATA 4
#define LD2410_RADAR_TIMEOUT_MSEC 1000

#define LD2410_RADAR_RESTART                                                   \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_RESTART_ACK                                                           \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA3, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_FACTORY_RESET                                             \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_FACTORY_RESET_ACK                                                     \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA2, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }

#define LD2410_RADAR_START_CONFIG_MODE_PAYLOAD                                             \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_START_CONFIG_MODE_ACK                                                                         \
    {                                                                                                              \
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_END_CONFIG_MODE_PAYLOAD                                   \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_END_CONFIG_MODE_ACK                                                   \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFE, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_SET_MAX_GATE_AND_DELAY                             \
    {                                                                   \
        0xFD, 0xFC, 0xFB, 0xFA, 0x14, 0x00, 0x60, 0x00, 0x00, 0x00,     \
            0x99, 0x00, 0x00, 0x00, 0x01, 0x00, 0x99, 0x00, 0x00, 0x00, \
            0x02, 0x00, 0x99, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01  \
    }
#define LD2410_RADAR_SET_MAX_GATE_AND_DELAY_ACK                                            \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x60, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_SET_BAUDRATE                                                          \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA1, 0x00, 0x07, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_SET_BAUDRATE_ACK                                                      \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA1, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_GET_PARAMETERS                                            \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x61, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_GET_FW_VERSION                                            \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_BLUETOOTH_ON                                                          \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA4, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_BLUETOOTH_OFF                                                         \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA4, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_BLUETOOTH_ACK                                                         \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA4, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_START_ENGINEERING_MODE                                    \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x62, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_START_ENGINEERING_MODE_ACK                                            \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x62, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_STOP_ENGINEERING_MODE                                     \
    {                                                                          \
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x63, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_STOP_ENGINEERING_MODE_ACK                                             \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x62, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_SET_GATE_SENSITIVITY                                                                                                                                                  \
    {                                                                                                                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x14, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0X04, 0x03, 0x02, 0x01 \
    }
#define LD2410_RADAR_SET_GATE_SENSITIVITY_ACK                                              \
    {                                                                                      \
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x64, 0x01, 0x00, 0x00, 0X04, 0x03, 0x02, 0x01 \
    }
    typedef struct __attribute__((packed)) ld2410_radar_fw_payload_s
    {
        uint8_t header1;       // fd
        uint8_t header2;       // fc
        uint8_t header3;       // fb
        uint8_t header4;       // fa
        uint16_t inner_length; // 0c 00
        uint8_t inner_header;  // a0
        uint8_t ack;           // 01
        uint8_t space1;        // 00
        uint8_t space2;        // 00
        uint8_t space3;        // 00
        uint8_t space4;        // 01
        uint8_t minor_version;
        uint8_t major_version;
        uint8_t bugfix_version4;
        uint8_t bugfix_version3;
        uint8_t bugfix_version2;
        uint8_t bugfix_version1;
    } ld2410_radar_fw_payload_t;

    typedef struct __attribute__((packed)) ld2410_radar_parameter_payload_s
    {
        uint8_t header1;       // FD
        uint8_t header2;       // FC
        uint8_t header3;       // FB
        uint8_t header4;       // FA
        uint16_t inner_length; // 1C 00
        uint8_t command;       // 61
        uint8_t ack;           // 01
        uint8_t space1;
        uint8_t space2;
        uint8_t inner_header;        // AA
        uint8_t max_gate_value;      // <= 08
        uint8_t max_moving_gate;     // <= 08
        uint8_t max_stationary_gate; // <= 08
        uint8_t moving_gate_0;       // <= 100
        uint8_t moving_gate_1;       // <= 100
        uint8_t moving_gate_2;       // <= 100
        uint8_t moving_gate_3;       // <= 100
        uint8_t moving_gate_4;       // <= 100
        uint8_t moving_gate_5;       // <= 100
        uint8_t moving_gate_6;       // <= 100
        uint8_t moving_gate_7;       // <= 100
        uint8_t moving_gate_8;       // <= 100
        uint8_t stationary_gate_0;   // <= 100
        uint8_t stationary_gate_1;   // <= 100
        uint8_t stationary_gate_2;   // <= 100
        uint8_t stationary_gate_3;   // <= 100
        uint8_t stationary_gate_4;   // <= 100
        uint8_t stationary_gate_5;   // <= 100
        uint8_t stationary_gate_6;   // <= 100
        uint8_t stationary_gate_7;   // <= 100
        uint8_t stationary_gate_8;   // <= 100
        uint8_t space3;
        uint8_t space4;
        uint8_t trailer1; // 04
        uint8_t trailer2; // 03
        uint8_t trailer3; // 02
        uint8_t trailer4; // 01
    } ld2410_radar_parameter_payload_t;

    typedef enum __attribute__((packed)) ld2410_radar_target_type_e
    {
        ld2410_radar_target_type_none = 0,
        ld2410_radar_target_type_moving,
        ld2410_radar_target_type_stationary,
        ld2410_radar_target_type_both
        /*
        Idk how you can be both moving and stationary,
        I used google translate to read Chinese instructions, gimme a break
        */
    } ld2410_radar_target_type_t;

    typedef enum __attribute__((packed)) ld2410_radar_frame_type_e
    {
        ld2410_radar_engineering_frame_type = 0x01,
        ld2410_radar_basic_frame_type = 0x02,
    } ld2410_radar_frame_type_t;

    typedef struct __attribute__((packed)) ld2410_radar_data_payload_s
    {
        uint8_t valid;
        union __attribute__((packed))
        {
            struct __attribute__((packed))
            {
                ld2410_radar_frame_type_t data_type;
                uint8_t header;
                ld2410_radar_target_type_t state;
                uint16_t moving_distance;
                uint8_t moving_energy;
                uint16_t stationary_distance;
                uint8_t stationary_energy;
                uint16_t detection_distance;
            } basic_values; // 4 header[0-3] + 2 length[4-5] + 11 bytes data[6-16] + 1 trailer[17] + 1 check[18] + 4 trailer[19-22]= 23
            struct __attribute__((packed))
            {
                ld2410_radar_frame_type_t data_type;
                uint8_t header;
                ld2410_radar_target_type_t state;
                uint16_t moving_distance;
                uint8_t moving_energy;
                uint16_t stationary_distance;
                uint8_t stationary_energy;
                uint16_t detection_distance;
                // Engineering Mode data:
                uint8_t max_motion_gate;
                uint8_t max_static_gate;
                struct __attribute__((packed))
                {
                    uint8_t moving_gate_0_energy;
                    uint8_t moving_gate_1_energy;
                    uint8_t moving_gate_2_energy;
                    uint8_t moving_gate_3_energy;
                    uint8_t moving_gate_4_energy;
                    uint8_t moving_gate_5_energy;
                    uint8_t moving_gate_6_energy;
                    uint8_t moving_gate_7_energy;
                    uint8_t moving_gate_8_energy;
                } moving_gates_energy;
                struct __attribute__((packed))
                {
                    uint8_t static_gate_0_energy;
                    uint8_t static_gate_1_energy;
                    uint8_t static_gate_2_energy;
                    uint8_t static_gate_3_energy;
                    uint8_t static_gate_4_energy;
                    uint8_t static_gate_5_energy;
                    uint8_t static_gate_6_energy;
                    uint8_t static_gate_7_energy;
                    uint8_t static_gate_8_energy;

                } static_gates_energy;
                uint8_t photo_sensor;
                uint8_t gpio_state;
            } engineering_values; // 4 header[0-3] + 2 length[4-5] + 33 bytes data[6-38] + 1 trailer[39] + 1 check[40] + 4 trailer[41-44]= 45
        } measurement_value;
    } ld2410_radar_data_payload_t;

    typedef enum __attribute__((packed)) ld2410_radar_ack_results_e
    {
        ld2410_radar_ack_results_success = 0,
        ld2410_radar_ack_results_need_restart,
        ld2410_radar_ack_results_fail
    } ld2410_radar_ack_results_t;

    typedef enum __attribute__((packed)) ld2410_radar_gate_e
    {
        ld2410_radar_gate_0 = 0,
        ld2410_radar_gate_1,
        ld2410_radar_gate_2,
        ld2410_radar_gate_3,
        ld2410_radar_gate_4,
        ld2410_radar_gate_5,
        ld2410_radar_gate_6,
        ld2410_radar_gate_7,
        ld2410_radar_gate_8,
    } ld2410_radar_gate_t;

    typedef enum __attribute__((packed)) ld2410_radar_baudrate_e
    {
        ld2410_radar_baudrate_9600 = 1,
        ld2410_radar_baudrate_19200,
        ld2410_radar_baudrate_38400,
        ld2410_radar_baudrate_57600,
        ld2410_radar_baudrate_115200,
        ld2410_radar_baudrate_230400,
        ld2410_radar_baudrate_256000,
        ld2410_radar_baudrate_460800,
    } ld2410_radar_baudrate_t;

    typedef unsigned long (*ld2410_radar_get_uptime_ms_callback_t)();
    typedef int (*ld2410_radar_poll_uart_byte_callback_t)(uint8_t *inByte);
    typedef void (*ld2410_radar_write_data_callback_t)(const uint8_t *buf, size_t len);
    typedef void (*ld2410_radar_log_hexdump_t)(const uint8_t *buf, size_t len);

    void ld2410_radar_init_callbacks(
        ld2410_radar_get_uptime_ms_callback_t gettime_cb,
        ld2410_radar_poll_uart_byte_callback_t poll_uart_cb,
        ld2410_radar_write_data_callback_t write_cb);
    ld2410_radar_ack_results_t ld2410_radar_read_data_frame(ld2410_radar_data_payload_t *report);
    ld2410_radar_ack_results_t ld2410_radar_enter_command_mode();
    ld2410_radar_ack_results_t ld2410_radar_exit_command_mode();
    ld2410_radar_ack_results_t ld2410_radar_factory_reset();
    ld2410_radar_ack_results_t ld2410_radar_restart();
    ld2410_radar_ack_results_t ld2410_radar_set_max_gate_and_delay(
        ld2410_radar_gate_t max_moving_gate, ld2410_radar_gate_t max_stationary_gate, uint8_t delay);
    ld2410_radar_ack_results_t ld2410_radar_get_parameters(ld2410_radar_parameter_payload_t *ld2410_radar_sensor_settings);
    ld2410_radar_baudrate_t ld2410_radar_get_baudrate_value(unsigned long long bps);
    ld2410_radar_ack_results_t ld2410_radar_set_baudrate(unsigned long long bps);
    ld2410_radar_ack_results_t ld2410_radar_get_fw_version(ld2410_radar_fw_payload_t *fw_payload);
    ld2410_radar_ack_results_t ld2410_radar_bluetooth_on();
    ld2410_radar_ack_results_t ld2410_radar_bluetooth_off();
    ld2410_radar_ack_results_t ld2410_radar_stop_engineering_mode();
    ld2410_radar_ack_results_t ld2410_radar_start_engineering_mode();
    ld2410_radar_ack_results_t ld2410_radar_set_gate_sensitivity(ld2410_radar_gate_t gate, uint8_t moving, uint8_t stationary);
#ifdef __cplusplus
}
#endif
#endif