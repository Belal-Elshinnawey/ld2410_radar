#include "ld2410_radar.h"
#include "stdbool.h"
#include "string.h"
ld2410_radar_get_uptime_ms_callback_t ld2410_radar_get_uptime_ms_callback;
ld2410_radar_poll_uart_byte_callback_t ld2410_radar_poll_uart_byte_callback;
ld2410_radar_write_data_callback_t ld2410_radar_write_data_callback;
uint8_t ___ld2410_raw_framedata[45];
const uint8_t __ld2410_radar_start_command_mode[] = LD2410_RADAR_START_CONFIG_MODE_PAYLOAD;
const uint8_t __ld2410_radar_start_command_mode_ack[] = LD2410_RADAR_START_CONFIG_MODE_ACK;
const uint8_t __ld2410_radar_end_command_mode[] = LD2410_RADAR_END_CONFIG_MODE_PAYLOAD;
const uint8_t __ld2410_radar_end_command_mode_ack[] = LD2410_RADAR_END_CONFIG_MODE_ACK;
const uint8_t __ld2410_radar_factory_reset[] = LD2410_RADAR_FACTORY_RESET;
const uint8_t __ld2410_radar_factory_reset_ack[] = LD2410_RADAR_FACTORY_RESET_ACK;
const uint8_t __ld2410_radar_restart[] = LD2410_RADAR_RESTART;
const uint8_t __ld2410_radar_restart_ack[] = LD2410_RADAR_RESTART_ACK;
const uint8_t __ld2410_radar_get_parameters[] = LD2410_RADAR_GET_PARAMETERS;
const uint8_t __ld2410_radar_get_fw_version[] = LD2410_RADAR_GET_FW_VERSION;
const uint8_t __ld2410_radar_bluetooth_on[] = LD2410_RADAR_BLUETOOTH_ON;
const uint8_t __ld2410_radar_bluetooth_off[] = LD2410_RADAR_BLUETOOTH_OFF;
const uint8_t __ld2410_radar_bluetooth_ack[] = LD2410_RADAR_BLUETOOTH_ACK;
uint8_t __ld2410_radar_set_max_gate_and_delay[] = LD2410_RADAR_SET_MAX_GATE_AND_DELAY;
const uint8_t __ld2410_radar_set_max_gate_and_delay_ack[] = LD2410_RADAR_SET_MAX_GATE_AND_DELAY_ACK;
const uint8_t __ld2410_radar_start_engineering_mode[] = LD2410_RADAR_START_ENGINEERING_MODE;
const uint8_t __ld2410_radar_start_engineering_mode_ack[] = LD2410_RADAR_START_ENGINEERING_MODE_ACK;
const uint8_t __ld2410_radar_stop_engineering_mode[] = LD2410_RADAR_STOP_ENGINEERING_MODE;
const uint8_t __ld2410_radar_stop_engineering_mode_ack[] = LD2410_RADAR_STOP_ENGINEERING_MODE_ACK;
uint8_t __ld2410_radar_set_baudrate[] = LD2410_RADAR_SET_BAUDRATE;
const uint8_t __ld2410_radar_set_baudrate_ack[] = LD2410_RADAR_SET_BAUDRATE_ACK;
uint8_t __ld2410_radar_set_gate_sensitivity[] = LD2410_RADAR_SET_GATE_SENSITIVITY;
uint8_t __ld2410_radar_set_gate_sensitivity_ack[] = LD2410_RADAR_SET_GATE_SENSITIVITY_ACK;
void ld2410_radar_init_callbacks(
    ld2410_radar_get_uptime_ms_callback_t gettime_cb,
    ld2410_radar_poll_uart_byte_callback_t poll_uart_cb,
    ld2410_radar_write_data_callback_t write_cb)
{
    ld2410_radar_get_uptime_ms_callback = gettime_cb;
    ld2410_radar_poll_uart_byte_callback = poll_uart_cb;
    ld2410_radar_write_data_callback = write_cb;
}
ld2410_radar_baudrate_t ld2410_radar_get_baudrate_value(unsigned long long bps)
{
    switch (bps)
    {
    case 9600ULL:
        return ld2410_radar_baudrate_9600;
    case 19200ULL:
        return ld2410_radar_baudrate_19200;
    case 38400ULL:
        return ld2410_radar_baudrate_38400;
    case 57600ULL:
        return ld2410_radar_baudrate_57600;
    case 115200ULL:
        return ld2410_radar_baudrate_115200;
    case 230400ULL:
        return ld2410_radar_baudrate_230400;
    case 256000ULL:
        return ld2410_radar_baudrate_256000;
    case 460800ULL:
        return ld2410_radar_baudrate_460800;
    default:
        return ld2410_radar_baudrate_256000;
    }
}
ld2410_radar_ack_results_t ld2410_radar_restart()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_restart, sizeof(__ld2410_radar_restart));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_restart_ack) && memcmp(__ld2410_radar_restart_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_restart_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_restart_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (write_results != ld2410_radar_ack_results_success)
    {
        if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
        {
            return ld2410_radar_ack_results_fail;
        }
        return ld2410_radar_ack_results_fail;
    }

    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_factory_reset()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_factory_reset, sizeof(__ld2410_radar_factory_reset));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        if (read_index == sizeof(__ld2410_radar_factory_reset_ack) && memcmp(__ld2410_radar_factory_reset_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_factory_reset_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_factory_reset_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    if (ld2410_radar_restart() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_read_data_frame(ld2410_radar_data_payload_t *report)
{
    uint8_t inByte;
    int data_valid = 0;
    int not_finished = 1;
    int rxstate = LD2410_RADAR_RX_NULL;
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (not_finished)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue; // No Data
        }
        switch (rxstate)
        {
        case LD2410_RADAR_RX_NULL:
            report->valid = 0;
            ___ld2410_raw_framedata[0] = 0;
            if (inByte == 0xf4)
            {
                rxstate = LD2410_RADAR_RX_HDR_1;
            }
            else
            {
                rxstate = LD2410_RADAR_RX_NULL;
            }
            break;
        case LD2410_RADAR_RX_HDR_1:
            if (inByte == 0xf3)
            {
                rxstate = LD2410_RADAR_RX_HDR_2;
            }
            else
            {
                rxstate = LD2410_RADAR_RX_NULL;
            }
            break;
        case LD2410_RADAR_RX_HDR_2:
            if (inByte == 0xf2)
            {
                rxstate = LD2410_RADAR_RX_HDR_3;
            }
            else
            {
                rxstate = LD2410_RADAR_RX_NULL;
            }
            break;
        case LD2410_RADAR_RX_HDR_3:
            if (inByte == 0xf1)
            {
                rxstate = LD2410_RADAR_RX_DATA;
                ___ld2410_raw_framedata[0] = 0xf4;
                ___ld2410_raw_framedata[1] = 0xf3;
                ___ld2410_raw_framedata[2] = 0xf2;
                ___ld2410_raw_framedata[3] = 0xf1;
            }
            else
            {
                rxstate = LD2410_RADAR_RX_NULL;
            }
            break;
        default:
            if ((rxstate < 7) ||                                         // If its a new frame
                (rxstate < 23 && ___ld2410_raw_framedata[06] == 0x02) || // or a basic data frame that didn't reach max
                (rxstate < 45 && ___ld2410_raw_framedata[06] == 0x01))   // or an engineering frame that didn't reach max
            {
                // Still getting data.
                ___ld2410_raw_framedata[rxstate] = inByte;
                rxstate++;
            }
            // Check if the frame is a valid basic data frame
            else if ((___ld2410_raw_framedata[06] == 0x02) && // Data type is basic
                     (___ld2410_raw_framedata[07] == 0xaa) && // Data frame header
                     (___ld2410_raw_framedata[17] == 0x55) && // Data frame trailer
                     (___ld2410_raw_framedata[19] == 0xf8) &&
                     (___ld2410_raw_framedata[20] == 0xf7) &&
                     (___ld2410_raw_framedata[21] == 0xf6) &&
                     (___ld2410_raw_framedata[22] == 0xf5)) // End of frame trailer
            {
                // Basic Data frame
                rxstate = LD2410_RADAR_RX_NULL;
                memcpy((void *)&(report->measurement_value.basic_values), &___ld2410_raw_framedata[6], sizeof(report->measurement_value.basic_values));
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
                /*
                radar is little endian becuase it stores the LSB in the lowest address.
                ex: frame length 23 00 translates to 0x0023
                so address 0 holdes the LSB and address 1 holds the MSB
                if system is big endian, reverse all the uint16_t values
                */
                report->measurement_value.basic_values.moving_distance = (report->measurement_value.basic_values.moving_distance << 8) | (report->measurement_value.basic_values.moving_distance >> 8);
                report->measurement_value.basic_values.stationary_distance = (report->measurement_value.basic_values.stationary_distance << 8) | (report->measurement_value.basic_values.stationary_distance >> 8);
                report->measurement_value.basic_values.detection_distance = (report->measurement_value.basic_values.detection_distance << 8) | (report->measurement_value.basic_values.detection_distance >> 8);
#endif
                //  sanity check
                if ((report->measurement_value.basic_values.moving_distance <= 800) &&
                    (report->measurement_value.basic_values.stationary_distance <= 800) &&
                    (report->measurement_value.basic_values.detection_distance <= 800) &&
                    (report->measurement_value.basic_values.moving_energy <= 100) &&
                    (report->measurement_value.basic_values.stationary_energy <= 100))
                {
                    report->valid = 1;
                    data_valid = 1;
                    not_finished = 0;
                }
                else
                {
                    report->valid = 0;
                    report->measurement_value.basic_values.data_type = 0;
                }
            }
            else if ((___ld2410_raw_framedata[06] == 0x01) && // Data type is engineering
                     (___ld2410_raw_framedata[07] == 0xaa) && // Data frame header
                     (___ld2410_raw_framedata[39] == 0x55) && // Data frame trailer
                     (___ld2410_raw_framedata[41] == 0xf8) &&
                     (___ld2410_raw_framedata[42] == 0xf7) &&
                     (___ld2410_raw_framedata[43] == 0xf6) &&
                     (___ld2410_raw_framedata[44] == 0xf5)) // End of frame trailer
            {
                // Engineering Data frame
                rxstate = LD2410_RADAR_RX_NULL;
                memcpy((void *)&(report->measurement_value.engineering_values), &___ld2410_raw_framedata[6], sizeof(report->measurement_value.engineering_values));
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
                /*
                radar is little endian becuase it stores the LSB in the lowest address.
                ex: frame length 23 00 translates to 0x0023
                so address 0 holdes the LSB and address 1 holds the MSB
                if system is big endian, reverse all the uint16_t values
                */
                report->measurement_value.engineering_values.moving_distance = (report->measurement_value.engineering_values.moving_distance << 8) | (report->measurement_value.engineering_values.moving_distance >> 8);
                report->measurement_value.engineering_values.stationary_distance = (report->measurement_value.engineering_values.stationary_distance << 8) | (report->measurement_value.engineering_values.stationary_distance >> 8);
                report->measurement_value.engineering_values.detection_distance = (report->measurement_value.engineering_values.detection_distance << 8) | (report->measurement_value.engineering_values.detection_distance >> 8);
#endif
                //  sanity check
                if ((report->measurement_value.engineering_values.max_motion_gate <= 8) &&
                    (report->measurement_value.engineering_values.max_static_gate <= 8) &&
                    (report->measurement_value.engineering_values.gpio_state <= 1) &&
                    (report->measurement_value.engineering_values.moving_distance <= 800) &&
                    (report->measurement_value.engineering_values.stationary_distance <= 800) &&
                    (report->measurement_value.engineering_values.detection_distance <= 800) &&
                    (report->measurement_value.engineering_values.moving_energy <= 100) &&
                    (report->measurement_value.engineering_values.stationary_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_0_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_1_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_2_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_3_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_4_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_5_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_6_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_7_energy <= 100) &&
                    (report->measurement_value.engineering_values.moving_gates_energy.moving_gate_8_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_0_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_1_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_2_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_3_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_4_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_5_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_6_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_7_energy <= 100) &&
                    (report->measurement_value.engineering_values.static_gates_energy.static_gate_8_energy <= 100))
                {
                    report->valid = 1;
                    data_valid = 1;
                    not_finished = 0;
                }
                else
                {
                    report->valid = 0;
                    report->measurement_value.engineering_values.data_type = 0;
                }
            }
            break;
        } // end switch
    }     // end while
    rxstate = LD2410_RADAR_RX_NULL;
    if (data_valid == 0)
    {
        return (ld2410_radar_ack_results_fail);
    }
    return (ld2410_radar_ack_results_success);
}
ld2410_radar_ack_results_t ld2410_radar_enter_command_mode()
{
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_write_data_callback(__ld2410_radar_start_command_mode, sizeof(__ld2410_radar_start_command_mode));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 18
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_start_command_mode_ack) && memcmp(__ld2410_radar_start_command_mode_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_start_command_mode_ack)) == 0)
        {
            return (ld2410_radar_ack_results_success);
        }
        else if (read_index >= sizeof(__ld2410_radar_start_command_mode_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    return (ld2410_radar_ack_results_fail);
}
ld2410_radar_ack_results_t ld2410_radar_exit_command_mode()
{
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_write_data_callback(__ld2410_radar_end_command_mode, sizeof(__ld2410_radar_end_command_mode));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_end_command_mode_ack) && memcmp(__ld2410_radar_end_command_mode_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_end_command_mode_ack)) == 0)
        {
            return (ld2410_radar_ack_results_success);
        }
        else if (read_index >= sizeof(__ld2410_radar_end_command_mode_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    return (ld2410_radar_ack_results_fail);
}
ld2410_radar_ack_results_t ld2410_radar_set_max_gate_and_delay(
    ld2410_radar_gate_t max_moving_gate, ld2410_radar_gate_t max_stationary_gate, uint8_t delay)
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    __ld2410_radar_set_max_gate_and_delay[10] = (uint8_t)max_moving_gate;
    __ld2410_radar_set_max_gate_and_delay[16] = (uint8_t)max_stationary_gate;
    __ld2410_radar_set_max_gate_and_delay[22] = (uint8_t)delay;

    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_set_max_gate_and_delay, sizeof(__ld2410_radar_set_max_gate_and_delay));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_set_max_gate_and_delay_ack) && memcmp(__ld2410_radar_set_max_gate_and_delay_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_set_max_gate_and_delay_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_set_max_gate_and_delay_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }

    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_set_baudrate(unsigned long long bps)
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    __ld2410_radar_set_baudrate[8] = (uint8_t)ld2410_radar_get_baudrate_value(bps);
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_set_baudrate, sizeof(__ld2410_radar_set_baudrate));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_set_baudrate_ack) && memcmp(__ld2410_radar_set_baudrate_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_set_baudrate_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_set_baudrate_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    // Restart Sensor
    if (ld2410_radar_restart() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_get_parameters(ld2410_radar_parameter_payload_t *ld2410_radar_sensor_settings)
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_get_parameters, sizeof(__ld2410_radar_get_parameters));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        //  Just being lazy:
        if (read_index == sizeof(ld2410_radar_parameter_payload_t) &&
            ___ld2410_raw_framedata[0] == 0xFD &&
            ___ld2410_raw_framedata[1] == 0xFC &&
            ___ld2410_raw_framedata[2] == 0xFB &&
            ___ld2410_raw_framedata[3] == 0xFA &&
            ___ld2410_raw_framedata[4] == 0x1C &&
            ___ld2410_raw_framedata[5] == 0x00 &&
            ___ld2410_raw_framedata[6] == 0x61 &&
            ___ld2410_raw_framedata[7] == 0x01 // Honestly if the rest has a mistake after all of this, then I'm okay with it
        )
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(ld2410_radar_parameter_payload_t))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (write_results == ld2410_radar_ack_results_success)
    {
        memcpy((void *)ld2410_radar_sensor_settings, ___ld2410_raw_framedata, sizeof(ld2410_radar_parameter_payload_t));
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
        ld2410_radar_sensor_settings->inner_length = (ld2410_radar_sensor_settings->inner_length << 8) | (ld2410_radar_sensor_settings->inner_length >> 8);
#endif
    }

    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_get_fw_version(ld2410_radar_fw_payload_t *fw_payload)
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_get_fw_version, sizeof(__ld2410_radar_get_fw_version));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }
        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        //  Just being lazy:
        if (read_index == sizeof(ld2410_radar_fw_payload_t) &&
            ___ld2410_raw_framedata[0] == 0xFD &&
            ___ld2410_raw_framedata[1] == 0xFC &&
            ___ld2410_raw_framedata[2] == 0xFB &&
            ___ld2410_raw_framedata[3] == 0xFA &&
            ___ld2410_raw_framedata[4] == 0x0C &&
            ___ld2410_raw_framedata[5] == 0x00 &&
            ___ld2410_raw_framedata[6] == 0xa0 &&
            ___ld2410_raw_framedata[7] == 0x01 // Honestly if the rest has a mistake after all of this, then I'm okay with it
        )
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(ld2410_radar_fw_payload_t))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (write_results == ld2410_radar_ack_results_success)
    {
        memcpy((void *)fw_payload, ___ld2410_raw_framedata, sizeof(ld2410_radar_fw_payload_t));
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
        fw_payload->inner_length = (fw_payload->inner_length << 8) | (fw_payload->inner_length >> 8);
#endif
    }

    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_bluetooth_on()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_bluetooth_on, sizeof(__ld2410_radar_bluetooth_on));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_bluetooth_ack) && memcmp(__ld2410_radar_bluetooth_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_bluetooth_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_bluetooth_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    // Restart Sensor
    if (ld2410_radar_restart() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_bluetooth_off()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_bluetooth_off, sizeof(__ld2410_radar_bluetooth_off));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_bluetooth_ack) && memcmp(__ld2410_radar_bluetooth_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_bluetooth_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_bluetooth_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    // Restart Sensor
    if (ld2410_radar_restart() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    if (write_results != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return ld2410_radar_ack_results_success;
}
ld2410_radar_ack_results_t ld2410_radar_start_engineering_mode()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_start_engineering_mode, sizeof(__ld2410_radar_start_engineering_mode));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_start_engineering_mode_ack) && memcmp(__ld2410_radar_start_engineering_mode_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_start_engineering_mode_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_start_engineering_mode_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }

    return write_results;
}
ld2410_radar_ack_results_t ld2410_radar_stop_engineering_mode()
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_stop_engineering_mode, sizeof(__ld2410_radar_stop_engineering_mode));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy:
        if (read_index == sizeof(__ld2410_radar_stop_engineering_mode_ack) && memcmp(__ld2410_radar_stop_engineering_mode_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_stop_engineering_mode_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_stop_engineering_mode_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return write_results;
}
ld2410_radar_ack_results_t ld2410_radar_set_gate_sensitivity(ld2410_radar_gate_t gate, uint8_t moving, uint8_t stationary)
{
    if (ld2410_radar_enter_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    __ld2410_radar_set_gate_sensitivity[10] = (uint8_t)gate;
    __ld2410_radar_set_gate_sensitivity[16] = moving > 100 ? 100 : moving;
    __ld2410_radar_set_gate_sensitivity[22] = stationary > 100 ? 100 : stationary;

    memset(___ld2410_raw_framedata, 0, sizeof(___ld2410_raw_framedata));
    uint8_t inByte, read_index;
    uint8_t frame_started = 0;
    ld2410_radar_ack_results_t write_results = ld2410_radar_ack_results_fail;
    ld2410_radar_write_data_callback(__ld2410_radar_set_gate_sensitivity, sizeof(__ld2410_radar_set_gate_sensitivity));
    unsigned long tstop = ld2410_radar_get_uptime_ms_callback() + 2 * LD2410_RADAR_TIMEOUT_MSEC;
    while (true)
    {
        if (ld2410_radar_get_uptime_ms_callback() > tstop)
        {
            break; // Timeout
        }
        inByte = 0;
        if (ld2410_radar_poll_uart_byte_callback(&inByte) != 0)
        {
            continue;
        }

        if ((frame_started == 0 && inByte != 0xFD))
        {
            // Frame did not start, and inbyte is not 0xFD
            continue; // No Data Yet
        }
        else if (frame_started != 1)
        {
            // Frame Did not start but inbyte is fd
            //  payload started
            frame_started = 1;
            read_index = 0;
        }
        ___ld2410_raw_framedata[read_index] = inByte;
        read_index++;
        // read_index + 1 should be 14
        //  Just being lazy: sizeof(__ld2410_radar_set_gate_sensitivity_ack)
        if (read_index == sizeof(__ld2410_radar_set_gate_sensitivity_ack) && memcmp(__ld2410_radar_set_gate_sensitivity_ack, ___ld2410_raw_framedata, sizeof(__ld2410_radar_set_gate_sensitivity_ack)) == 0)
        {
            write_results = (ld2410_radar_ack_results_success);
            break;
        }
        else if (read_index >= sizeof(__ld2410_radar_set_gate_sensitivity_ack))
        {
            // Try again until timeout
            frame_started = 0;
            read_index = 0;
        }
    }
    if (ld2410_radar_exit_command_mode() != ld2410_radar_ack_results_success)
    {
        return ld2410_radar_ack_results_fail;
    }
    return write_results;
}

/*
    Commands:
    Enter Command mode Done and tested
    Exit Command mode Done and tested
    Set max distance gate and delay mode Done and tested
    Read Config Command Done and tested
    Set serial baudrate command Done and tested
    Module Restart Command Done and tested
    Factory reset Command Done and tested
    Read Firmware version command Done and tested
    Bluetooth on/off Commands Done and tested
    Enable Engineering mode Done and tested
    Close engineering mode Done and tested
    Set Gate sensitivity Command Done and tested
*/
