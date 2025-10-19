#include <stdint.h>
#include <stdlib.h>
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

const uint8_t NUM_OF_CHANNELS = 5;
const uint8_t CHAN0 = 18;
const uint8_t CHAN1 = 19;
const uint8_t CHAN2 = 21;
const uint8_t CHAN3 = 22;
const uint8_t CHAN4 = 23;

const uint8_t CHANNELS[] = {
	CHAN0,
	CHAN1,
	CHAN2,
	CHAN3,
	CHAN4,
};

// When changing this, also change in headers of metadata
const int NUM_OF_SAMPLES = 7168;

uint8_t trigger_mask = 0;
uint8_t triggers = 0;

void reset_trigger() {
	trigger_mask = 0;
	triggers = 0;
}

void sample_and_send() {
	uint8_t *buffer = malloc(NUM_OF_SAMPLES);

	for (int sample_num = 0; sample_num < NUM_OF_SAMPLES; sample_num++) {
		// RESET CURRENT VALUE IN BUFFER IF ANY
		buffer[sample_num] = 0;

		for(uint8_t channel_num = 0; channel_num < NUM_OF_CHANNELS; channel_num++) {
			uint8_t current_channel = gpio_get_level(CHANNELS[channel_num]) << channel_num;
			buffer[sample_num] |= current_channel;
		}

		wait_us_blocking(1);
	}

	uart_write_bytes(UART_NUM_2, buffer, NUM_OF_SAMPLES);
	uart_wait_tx_idle_polling(UART_NUM_2);

	free(buffer);

	ESP_LOGI("CAPTURE SENT", "NOW");
}

uint8_t get_current_pins_state() {
	uint8_t state = 0;
	for(uint8_t channel_num = 0; channel_num < NUM_OF_CHANNELS; channel_num++) {
		state |= (gpio_get_level(CHANNELS[channel_num]) << channel_num);
	}

	return state;
}

void arm_trigger() {
	if(trigger_mask) {
		ESP_LOGI("START CAPTURE", "TRIGGERED");
		// WAIT FOR TRIGGERS --> TODO: Find out how to make the watchdog not upset here.
		int timeout = 0;
		while((triggers ^ get_current_pins_state()) & trigger_mask){
			timeout++;
			if(timeout > 10000000) {
				ESP_LOGI("TIMEOUT", "TRIGGERED");
				break;
			}
		};

		// SEND
		sample_and_send();
	} else {
		ESP_LOGI("START CAPTURE", "IMMEDIATE");
		// GOOI IMMEDIATE
		sample_and_send();
	}
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

void send_metadata() {
	uint8_t metadata[] = {
		// Name
		0x01, 'H', 'E', 'L', 'L', 'O', 0x00,
		 // Firmware version
		0x02, '0', '.', '1', '7', 0x00,
		// sample memory (1024 bytes)
		0x21, 0x00, 0x00, 0x1C, 0x00, 
		// sample rate (TODO)
		0x23, 0x00, 0x3D, 0x09, 0x00, 
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
}

/*
 * Extended SUMP commands are 5 bytes.  A command byte followed by 4 bytes
 * of options. At this point normally we already read the command byte,
 * this gets the remaining 4 bytes of the command.
 */
void ignore_remaining_command_bytes(uint8_t num_bytes) {
	vTaskDelay(10);
	uint8_t buf[num_bytes];

	uart_read_bytes(UART_NUM_2, buf, 4, 100);

	ESP_LOGI("IGNORED COMMAND BYTES", "...");
}


void handle_command(uint8_t cmd) {
    switch (cmd) {
        case 0x01:
		arm_trigger();
		break;
        case 0x02:
		send_device_id();
		break;
	case 0x04:
		send_metadata();
		break;
	case 0xC0: 
		reset_trigger();
		uart_read_bytes(UART_NUM_2, &trigger_mask, 1, 100);
		ESP_LOGI("TRIGGER MASK", "%x", trigger_mask);
		break;
	
	case 0xC1:
		uart_read_bytes(UART_NUM_2, &triggers, 1, 100);
		ESP_LOGI("TRIGGER VALUES", "%x", triggers);
		break;
	
	case 0xC2:
	case 0xC3:
	case 0xC4:
	case 0xC5:
	case 0xC6:
	case 0xF3:
	case 0x80:
	case 0x81:
	case 0x82:
		ESP_LOGI("EXTENDED COMMAND", "%x", cmd);
		ignore_remaining_command_bytes(4);
		break;

        default: ignore(cmd);
    }
}

void app_main(void)
{
	uart_driver_install(UART_NUM_2, NUM_OF_SAMPLES, 0, 0, NULL, 0);

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

	for(uint8_t i = 0; i < NUM_OF_CHANNELS; i++) {
		gpio_set_direction(CHANNELS[i], GPIO_MODE_INPUT);
	}

	while (1) {
		uint8_t buf[1];
		buf[0] = 0xFF;

		uart_read_bytes(UART_NUM_2, buf, 1, 100);
		
		handle_command(buf[0]);
	}
}

