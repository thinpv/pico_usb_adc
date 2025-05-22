/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_11
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256 * 2

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
	BaseType_t mustYield = pdFALSE;
	// Notify that ADC continuous driver has done enough number of conversions
	vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

	return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
	adc_continuous_handle_t handle = NULL;

	adc_continuous_handle_cfg_t adc_config = {
			.max_store_buf_size = 1024,
			.conv_frame_size = EXAMPLE_READ_LEN,
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

	adc_continuous_config_t dig_cfg = {
			.sample_freq_hz = 20 * 1000,
			.conv_mode = EXAMPLE_ADC_CONV_MODE,
			.format = EXAMPLE_ADC_OUTPUT_TYPE,
	};

	adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
	dig_cfg.pattern_num = channel_num;
	for (int i = 0; i < channel_num; i++)
	{
		adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
		adc_pattern[i].channel = channel[i] & 0x7;
		adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
		adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

		ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
		ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
		ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
	}
	dig_cfg.adc_pattern = adc_pattern;
	ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

	*out_handle = handle;
}

typedef union
{
	struct
	{
		uint8_t d1 : 6;
		uint16_t d2 : 6;
		// uint8_t d3 : 4;
	} d8;
	uint16_t d16;
} sample_data_u;

// static void convert_data_16bit(uint16_t *data, int len)
// {
// 	sample_data_u sample_data;
// 	for (uint32_t i = 0; i < len / SUM_AGV_BYTE; i++)
// 	{
// 		uint16_t avg = 0;
// 		for (size_t j = 0; j < SUM_AGV_BYTE; j++)
// 		{
// 			avg += data[i * SUM_AGV_BYTE + j];
// 		}
// 		avg = avg / SUM_AGV_BYTE;
// 		sample_data.d16 = avg;
// 		data[i] = (sample_data.d8.d2 << 8) | sample_data.d8.d1 | 0x80;
// 	}
// }

static uint16_t convert_data_16bit_2(uint16_t data)
{
	sample_data_u sample_data;
	sample_data.d16 = data;
	return (sample_data.d8.d2 << 8) | sample_data.d8.d1 | 0x80;
}

static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

/**
 * @brief Application Queue
 */
static QueueHandle_t app_queue;
typedef struct
{
	uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; // Data buffer
	size_t buf_len;																	// Number of bytes received
	uint8_t itf;																		// Index of CDC device interface
} app_message_t;

/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
	/* initialization */
	size_t rx_size = 0;

	/* read */
	esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
	if (ret == ESP_OK)
	{

		app_message_t tx_msg = {
				.buf_len = rx_size,
				.itf = itf,
		};

		memcpy(tx_msg.buf, rx_buf, rx_size);
		xQueueSend(app_queue, &tx_msg, 0);
	}
	else
	{
		ESP_LOGE(TAG, "Read Error");
	}
}

/**
 * @brief CDC device line change callback
 *
 * CDC device signals, that the DTR, RTS states changed
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
	int dtr = event->line_state_changed_data.dtr;
	int rts = event->line_state_changed_data.rts;
	ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void usb_init(void)
{
	// Create FreeRTOS primitives
	app_queue = xQueueCreate(5, sizeof(app_message_t));
	assert(app_queue);
	app_message_t msg;

	ESP_LOGI(TAG, "USB initialization");
	const tinyusb_config_t tusb_cfg = {
			.device_descriptor = NULL,
			.string_descriptor = NULL,
			.external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
			.fs_configuration_descriptor = NULL,
			.hs_configuration_descriptor = NULL,
			.qualifier_descriptor = NULL,
#else
			.configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
	};

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t acm_cfg = {
			.usb_dev = TINYUSB_USBDEV_0,
			.cdc_port = TINYUSB_CDC_ACM_0,
			.rx_unread_buf_sz = 64,
			// .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
			.callback_rx_wanted_char = NULL,
			.callback_line_state_changed = NULL,
			.callback_line_coding_changed = NULL};

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
	/* the second way to register a callback */
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
			TINYUSB_CDC_ACM_0,
			CDC_EVENT_LINE_STATE_CHANGED,
			&tinyusb_cdc_line_state_changed_callback));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
	acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
			TINYUSB_CDC_ACM_1,
			CDC_EVENT_LINE_STATE_CHANGED,
			&tinyusb_cdc_line_state_changed_callback));
#endif

	ESP_LOGI(TAG, "USB initialization DONE");
	// while (1)
	// {
	// 	if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
	// 	{
	// 		if (msg.buf_len)
	// 		{

	// 			/* Print received data*/
	// 			ESP_LOGI(TAG, "Data from channel %d:", msg.itf);
	// 			ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

	// 			/* write back */
	// 			tinyusb_cdcacm_write_queue(msg.itf, msg.buf, msg.buf_len);
	// 			esp_err_t err = tinyusb_cdcacm_write_flush(msg.itf, 0);
	// 			if (err != ESP_OK)
	// 			{
	// 				ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
	// 			}
	// 		}
	// 	}
	// }
}

void app_main(void)
{
	esp_err_t ret;
	uint32_t ret_num = 0;
	uint8_t result[EXAMPLE_READ_LEN] = {0};
	memset(result, 0xcc, EXAMPLE_READ_LEN);

	s_task_handle = xTaskGetCurrentTaskHandle();

	adc_continuous_handle_t handle = NULL;
	continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

	adc_continuous_evt_cbs_t cbs = {
			.on_conv_done = s_conv_done_cb,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
	ESP_ERROR_CHECK(adc_continuous_start(handle));

	usb_init();

	while (1)
	{

		/**
		 * This is to show you the way to use the ADC continuous mode driver event callback.
		 * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
		 * However in this example, the data processing (print) is slow, so you barely block here.
		 *
		 * Without using this event callback (to notify this task), you can still just call
		 * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
		 */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

		uint16_t buff[EXAMPLE_READ_LEN];
		bool have_data = false;
		bool first_byte = false;
		int j = 0;

		while (1)
		{
			have_data = false;

			ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
			if (ret == ESP_OK)
			{
				have_data = true;
				// ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
				j = 0;
				uint32_t agv = 0;
				for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
				{
					adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
					uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
					uint32_t data = EXAMPLE_ADC_GET_DATA(p);
					/* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
					if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT))
					{
						// ESP_LOGI(TAG, "Unit: %s, Channel: %" PRIu32 ", Value: %" PRIx32, unit, chan_num, data);
						if (first_byte == false)
						{
							first_byte = true;
							agv = data;
						}
						else
						{
							first_byte = false;
							agv += data;
							buff[j++] = convert_data_16bit_2(agv / 2);
						}
						// ESP_LOGI(TAG, "Unit: %s, Channel: %" PRIu32 ", Value: %" PRIx32, unit, chan_num, data);
					}
					else
					{
						// ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", unit, chan_num, data);
					}
				}
				if (have_data)
					tinyusb_cdcacm_write_queue(0, (uint8_t *)buff, 2 * j);

				/**
				 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
				 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
				 * usually you don't need this delay (as this task will block for a while).
				 */
				// vTaskDelay(1);
			}
			else if (ret == ESP_ERR_TIMEOUT)
			{
				// We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
				break;
			}
		}
	}

	ESP_ERROR_CHECK(adc_continuous_stop(handle));
	ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
