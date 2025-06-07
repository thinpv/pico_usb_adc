#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "Main";

static QueueHandle_t myQueue;

extern void adc_init(void *data);

extern void usb_init(void);
extern void usb_send_data(uint16_t *data, uint16_t len);

static void handle_queue(void *data)
{
	ESP_LOGI(TAG, "handle_queue Start");
	uint16_t *buff = NULL;
	while (1)
	{
		if (xQueueReceive(myQueue, &buff, (TickType_t)5))
		{
			if (buff)
			{
				// printf("buff: \n");
				// for (int i = 0; i < 1024; i++)
				// 	printf("%d\n", buff[1023]);
				usb_send_data(buff, 1024);
				free(buff);
			}
		}
		else
			vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	myQueue = xQueueCreate(10, sizeof(uint16_t *));

	usb_init();

	if (xTaskCreate(handle_queue, "handle_queue", 10240, NULL, 5, NULL) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create task");
	}

	if (xTaskCreate(adc_init, "adc_init", 10240, &myQueue, 10, NULL) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create task");
	}
}