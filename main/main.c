#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "esp_timer.h"

void wait_us_blocking(uint32_t micros_to_wait) {
    uint64_t micros_now_plus_delay = esp_timer_get_time() + micros_to_wait;
    while(micros_now_plus_delay > esp_timer_get_time()){}
}

void reset_state(void) {
	// TODO:
}

const uint8_t NUM_OF_CHANNELS = 2;
const uint8_t CHAN1 = 22;
const uint8_t CHAN2 = 23;

const int NUM_OF_SAMPLES = 1024;

const uint8_t TRIGGER = 21;
const int TRIGGER_MAX_TIMEOUT = 10000;


void arm_trigger() {
	ESP_LOGI("START CAPTURE", "NOW");

	int timeout = 0;
	while(gpio_get_level(TRIGGER) == 0) {
		timeout++;
		if(timeout > TRIGGER_MAX_TIMEOUT) {
			ESP_LOGI("TIMEOUT", "WAITING ON TRIGGER");
			break;
		}
		wait_us_blocking(1);
	}
	uint8_t *buffer = malloc(NUM_OF_SAMPLES);

	for (int i = 0; i < NUM_OF_SAMPLES; i++) {
		uint8_t chan1_current = gpio_get_level(CHAN1);
		uint8_t chan2_current = gpio_get_level(CHAN2) << 1;

		buffer[i] = chan1_current | chan2_current;
		wait_us_blocking(1);
	}
	uart_write_bytes(UART_NUM_2, buffer, NUM_OF_SAMPLES);
	uart_wait_tx_idle_polling(UART_NUM_2);
	free(buffer);

	gpio_set_level(TRIGGER, 0);
	ESP_LOGI("CAPTURE SENT", "NOW");
}

void send_device_id() {
	ESP_LOGI("SENDING DEVICE ID", "1ALS");
	uart_write_bytes(UART_NUM_2, "1ALS", 4);
}

void ignore(uint8_t cmd) {
	if(cmd != 0xFF) {
		ESP_LOGI("IGNORING MSG", "%x", cmd);
	}
}

void handle_command(uint8_t cmd) {
    switch (cmd) {
        case 0x00: reset_state(); break;
        case 0x01: arm_trigger(); break;
        case 0x02: send_device_id(); break;

	// Metadata
	case 0x04: {
		uint8_t metadata[] = {
			// Name
			0x01, 'H', 'E', 'L', 'L', 'O', 0x00,
			 // Firmware version
			0x02, '0', '.', '1', '7', 0x00,
			// sample memory (1024 bytes)
			0x21, 0x00, 0x00, 0x04, 0x00, 
			// Num channels
			0x40, NUM_OF_CHANNELS,
			// Protocol version
			0x41, 0x02,         
			// Terminate
			0x00                      
		};

		ESP_LOGI("SENDING METADATA", "0x04");
		uart_write_bytes(UART_NUM_2, (const char *)metadata, sizeof(metadata));
		uart_wait_tx_idle_polling(UART_NUM_2);
	    break;
	}

        default: ignore(cmd);
    }
}

void app_main(void)
{
	// QueueHandle_t uart_queue;

	uart_driver_install(UART_NUM_2, 256, 0, 0, NULL, 0);

	const uart_port_t uart_num = UART_NUM_2;
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	ESP_ERROR_CHECK(
		uart_set_pin(
			UART_NUM_2,
			GPIO_NUM_17,
			GPIO_NUM_16,
		       UART_PIN_NO_CHANGE,
		       UART_PIN_NO_CHANGE
	       )
	);

	gpio_set_direction(CHAN1, GPIO_MODE_INPUT);
	gpio_set_direction(CHAN2, GPIO_MODE_INPUT);

	gpio_set_direction(TRIGGER, GPIO_MODE_INPUT_OUTPUT);

	while (1) {
		uint8_t buf[1];
		buf[0] = 0xFF;

		uart_read_bytes(UART_NUM_2, buf, 1, 100);
		
		handle_command(buf[0]);
	}
}

