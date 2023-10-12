#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <hal/nrf_uarte.h>
#include "string.h"
#include <zephyr/logging/log.h>
#include "ld2410_radar.h"
LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);
#define RADAR_NODE DT_NODELABEL(uart1)
K_MSGQ_DEFINE(uart_msgq, 1, 128, 0);

ld2410_radar_data_payload_t report;
ld2410_radar_parameter_payload_t sensor_settings;
ld2410_radar_fw_payload_t fw_version;
static const struct device *const radar_dev = DEVICE_DT_GET(RADAR_NODE);

void uart_irq_callback(const struct device *dev, void *user_data)
{
	uint8_t c;
	uint8_t oc;

	if (!uart_irq_update(dev))
	{
		return;
	}

	if (!uart_irq_rx_ready(dev))
	{
		return;
	}
	while (uart_fifo_read(dev, &c, 1) == 1)
	{
		if (k_msgq_num_free_get(&uart_msgq) == 0)
		{
			k_msgq_get(&uart_msgq, &oc, K_NO_WAIT);
		}
		k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
	}
}
void write_to_uart(const uint8_t *buf, size_t len)
{
	for (int i = 0; i < len; i++)
	{
		uart_poll_out(radar_dev, buf[i]);
	}
}
int pull_radar_uart_queue(uint8_t *inByte)
{
	return k_msgq_get(&uart_msgq, inByte, K_MSEC(100));
}
unsigned long get_uptime_ms()
{
	return (unsigned long)k_uptime_get();
}
static uint32_t get_uart_baudrate_value(unsigned long long bps)
{
	return ((bps) * (4294967296ULL) / 16000000ULL);
}
ld2410_radar_ack_results_t change_ld2410_sensor_baudrate(unsigned long long bps)
{
	// Write bauderate command
	if (ld2410_radar_set_baudrate(bps) == ld2410_radar_ack_results_success)
	{
		LOG_WRN("Baudrate changed Successfully\n");
		// If success change baudrate in nrfconnect sdk
		uint32_t baudrate_reg = get_uart_baudrate_value(bps); // Default values in nrfconnect sdk dont have 256k baudrate.
		// If using a different UART like uart2 use NRF_UARTE2
		NRF_UARTE1->BAUDRATE = baudrate_reg;
		return ld2410_radar_ack_results_success;
	}
	else
	{
		LOG_WRN("Baudrate change Failed\n");
		return ld2410_radar_ack_results_fail;
	}
}
unsigned long long find_ld2410_current_baudrate()
{
	uint32_t baudrate_reg;

	baudrate_reg = get_uart_baudrate_value(115200ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 115200ULL;
	}
	baudrate_reg = get_uart_baudrate_value(256000ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 256000ULL;
	}
	baudrate_reg = get_uart_baudrate_value(9600ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 9600ULL;
	}
	baudrate_reg = get_uart_baudrate_value(19200ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 19200ULL;
	}
	baudrate_reg = get_uart_baudrate_value(38400ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 38400ULL;
	}
	baudrate_reg = get_uart_baudrate_value(57600ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 57600ULL;
	}
	baudrate_reg = get_uart_baudrate_value(230400ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 230400ULL;
	}

	baudrate_reg = get_uart_baudrate_value(460800ULL);
	NRF_UARTE1->BAUDRATE = baudrate_reg;
	ld2410_radar_enter_command_mode();
	ld2410_radar_exit_command_mode();
	if (ld2410_radar_read_data_frame(&report) == ld2410_radar_ack_results_success)
	{
		return 460800ULL;
	}
	return 0;
}

void main(void)
{
	uart_irq_callback_user_data_set(radar_dev, uart_irq_callback, NULL);
	uart_irq_rx_enable(radar_dev);

	ld2410_radar_init_callbacks(get_uptime_ms, pull_radar_uart_queue, write_to_uart);
	printk("Starting\n");
	// Check current baudrate
	unsigned long long current_br = find_ld2410_current_baudrate();
	while (current_br == 0)
	{
		current_br = find_ld2410_current_baudrate();
	}
	printk("current baudrate %llu\n", current_br);
	if (current_br != 115200ULL)
	{
		while (ld2410_radar_factory_reset() != ld2410_radar_ack_results_success)
		{
		}
		printk("Factory reset Success\n");
		uint32_t baudrate_reg = get_uart_baudrate_value(256000ULL);
		NRF_UARTE1->BAUDRATE = baudrate_reg;
		while (change_ld2410_sensor_baudrate(115200ULL) == ld2410_radar_ack_results_fail)
		{
			k_msleep(10);
		}
	}

	if (ld2410_radar_get_fw_version(&fw_version) == ld2410_radar_ack_results_success)
	{
		printk("FW Read Success %02x.%02x.%02x%02x%02x%02x\n",
			   fw_version.major_version,
			   fw_version.minor_version,
			   fw_version.bugfix_version1,
			   fw_version.bugfix_version2,
			   fw_version.bugfix_version3,
			   fw_version.bugfix_version4);
	}
	else
	{
		printk("FW Read Failed\n");
	}

	while (ld2410_radar_bluetooth_off() != ld2410_radar_ack_results_success)
	{
		printk("Bluetooth off failed\n");
	}
	printk("Bluetooth off success\n");

	k_msleep(1000);

	while (ld2410_radar_set_gate_sensitivity(ld2410_radar_gate_0, 100, 100) != ld2410_radar_ack_results_success)
	{
		printk("Gate Sensitivity fail \n");
	}
	printk("Gate Sensitivity success \n");
	while (ld2410_radar_start_engineering_mode() == ld2410_radar_ack_results_fail)
	{
	}
	while (true)
	{
		if (ld2410_radar_read_data_frame(&report) != ld2410_radar_ack_results_success)
		{
			continue;
		}

		if (report.measurement_value.basic_values.data_type == 0x02)
		{
			LOG_WRN("Basic State: %d,", report.measurement_value.basic_values.state);
			LOG_WRN(" Moving energy: %d,", report.measurement_value.basic_values.moving_energy);
			LOG_WRN(" Moving distance: %d,", report.measurement_value.basic_values.moving_distance);
			LOG_WRN(" Stationary energy: %d,", report.measurement_value.basic_values.stationary_energy);
			LOG_WRN(" Stationary distance: %d,", report.measurement_value.basic_values.stationary_distance);
			LOG_WRN(" Detection distance: %d\n", report.measurement_value.basic_values.detection_distance);
		}
		else if (report.measurement_value.engineering_values.data_type == 0x01)
		{
			LOG_WRN("Engineering State: %d,", report.measurement_value.engineering_values.state);
			LOG_WRN(" Moving energy: %d,", report.measurement_value.engineering_values.moving_energy);
			LOG_WRN(" Moving distance: %d,", report.measurement_value.engineering_values.moving_distance);
			LOG_WRN(" Stationary energy: %d,", report.measurement_value.engineering_values.stationary_energy);
			LOG_WRN(" Stationary distance: %d,", report.measurement_value.engineering_values.stationary_distance);
			LOG_WRN(" Detection distance: %d", report.measurement_value.engineering_values.detection_distance);
			LOG_WRN(" Photo Sensor: %d\n", report.measurement_value.engineering_values.photo_sensor);
		}
	}
}
