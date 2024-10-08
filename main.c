// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>
#include <math.h>
#include <time.h>

#include "bsp/board_api.h"

#include "usb_descriptors.h"

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN PICO_DEFAULT_LED_PIN

#define BUFFER_SIZE 2560

// set this to determine sample rate
// 96    = 500,000 Hz
// 240   = 200,000 Hz
// 480   = 100,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 240
// #define FSAMP 50000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 1000
#define N_SAMPLES 500

// globals
dma_channel_config cfg;
uint dma_chan;
typedef struct
{
  cdc_line_coding_t usb_lc;
  cdc_line_coding_t uart_lc;
  mutex_t lc_mtx;
  uint8_t uart_buffer[BUFFER_SIZE];
  uint32_t uart_pos;
  mutex_t uart_mtx;
  uint8_t usb_buffer[BUFFER_SIZE];
  uint32_t usb_pos;
  mutex_t usb_mtx;
} uart_data_t;

uart_data_t UART_DATA[CFG_TUD_CDC];

static char log_buff[512];
void mylog(const char *format, ...)
{
  if (tud_cdc_n_connected(1))
  {
    va_list list;
    va_start(list, format);
    sprintf(log_buff, format, list);
    va_end(list);
    tud_cdc_n_write(1, log_buff, strlen(log_buff));
    tud_cdc_n_write_flush(1);
  }
}

void usb_read_bytes(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];
  uint32_t len = tud_cdc_n_available(itf);

  if (len &&
      mutex_try_enter(&ud->usb_mtx, NULL))
  {
    len = MIN(len, BUFFER_SIZE - ud->usb_pos);
    if (len)
    {
      uint32_t count;

      count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
      // tud_cdc_n_write(itf, &ud->usb_buffer[ud->usb_pos], count);
      // tud_cdc_n_write_flush(itf);
      ud->usb_pos += count;
    }

    mutex_exit(&ud->usb_mtx);
  }
}

void usb_write_bytes(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];

  if (ud->uart_pos &&
      mutex_try_enter(&ud->uart_mtx, NULL))
  {
    uint32_t count;

    count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
    if (count < ud->uart_pos)
      memmove(ud->uart_buffer, &ud->uart_buffer[count],
              ud->uart_pos - count);
    ud->uart_pos -= count;

    mutex_exit(&ud->uart_mtx);

    if (count)
      tud_cdc_n_write_flush(itf);
  }
}

void usb_cdc_process(uint8_t itf)
{
  usb_read_bytes(itf);
  usb_write_bytes(itf);
}

void core1_entry(void)
{
  board_init();
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb)
  {
    board_init_after_tusb();
  }

  while (1)
  {
    int itf;
    int con = 0;

    tud_task();

    for (itf = 0; itf < CFG_TUD_CDC; itf++)
    {
      if (tud_cdc_n_connected(itf))
      {
        con = 1;
        usb_cdc_process(itf);
      }
    }

    gpio_put(LED_PIN, con);
  }
}

// void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count)
// {
//   adc_fifo_setup(true, false, 0, false, false);
//   adc_run(true);
//   for (size_t i = 0; i < count; i = i + 1)
//     buf[i] = adc_fifo_get_blocking();
//   adc_run(false);
//   adc_fifo_drain();
// }

void sample(uint8_t *capture_buf)
{
  adc_fifo_drain();
  adc_run(false);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,   // dst
                        &adc_hw->fifo, // src
                        N_SAMPLES,     // transfer count
                        true           // start immediately
  );

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  gpio_put(LED_PIN, 0);
}

void setup()
{
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
	adc_set_temp_sensor_enabled(false);
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      true   // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  // // calculate frequencies of each bin
  // float f_max = FSAMP;
  // float f_res = f_max / N_SAMPLES;
  // for (int i = 0; i < N_SAMPLES; i++) {freqs[i] = f_res*i;}
}

int main(void)
{
  int itf;

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  for (int i = 0; i < 10; i++)
  {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(100);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(100);
  }

  setup();

  usbd_serial_init();

  multicore_launch_core1(core1_entry);

  uint8_t sample_buf[N_SAMPLES];
  while (1)
  {
    if (tud_cdc_n_connected(0))
    {
      sample(sample_buf);
      tud_cdc_n_write(0, sample_buf, sizeof(sample_buf));
      // tud_cdc_n_write(0, "\r\nabc\r\n", 7);
      tud_cdc_n_write_flush(0);
      // sleep_ms(500);
    }
  }

  return 0;
}
