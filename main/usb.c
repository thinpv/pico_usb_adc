#include <unistd.h>
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

static const char *TAG = "USB";

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

uint16_t convert_data_16bit_2(uint16_t data)
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
}

uint16_t buff[1024];
void usb_send_data(uint16_t *data, uint16_t len)
{
	if (len > 1024)
	{
		ESP_LOGE(TAG, "size to large");
	}
	for (int i = 0; i < len; i++)
	{
		buff[i] = convert_data_16bit_2(data[i]);
	}
	tinyusb_cdcacm_write_queue(0, (uint8_t *)buff, 2 * len);
}