/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "st7789_lcd.h"
#include "st7789_lcd.pio.h"

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

#define PIN_DIN 0
#define PIN_CLK 1
#define PIN_CS 2  // Must be CLK + 1
#define PIN_DC 3  // Must be CLK + 2
#define PIN_RESET 4
#define PIN_BL 5

#define SERIAL_CLK_DIV 2.f

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
#if 1
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                         // Software reset
        1, 10, 0x11,                         // Exit sleep mode
        2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
        2, 0, 0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
        5, 0, 0x2a, 0x00, 0x00, 0x00, 0xf0, // CASET: column addresses from 0 to 240 (f0)
        5, 0, 0x2b, 0x00, 0x00, 0x00, 0xf0, // RASET: row addresses from 0 to 240 (f0)
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                     // Terminate list
};
#else
// Modified sequence from Pimoroni's python library
// This plays with some power settings which makes the image slightly brighter,
// but if anything seems to make flickering slightly worse.
static const uint8_t st7789_init_seq[] = {
        1, 30, 0x01,
        2, 0, 0x36, 0x00,
        2, 0, 0x3A, 0x55,
        2, 0, 0xB7, 0x14,
        2, 0, 0xBB, 0x37,
        3, 0, 0xC2, 0x01, 0xFF,
        2, 0, 0xC3, 0x12,
        2, 0, 0xC4, 0x20,
        3, 0, 0xD0, 0xA4, 0XA1,
        2, 0, 0xC6, 0x0F,
        5, 0, 0x2a, 0x00, 0x00, 0x00, 0xf0, // CASET: column addresses from 0 to 240 (f0)
        5, 0, 0x2b, 0x00, 0x00, 0x00, 0xf0, // RASET: row addresses from 0 to 240 (f0)
        1, 2, 0x21,
        1, 2, 0x13,
        1, 10, 0x11,
        1, 2, 0x29,
        0
};
#endif

static inline uint32_t st7789_encode_cmd(uint8_t cmd, uint32_t data_count)
{
  uint32_t instr = 0x100;
  if (data_count) instr = (data_count * 8) << 8;
  instr |= cmd;
  return instr;
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, int count) {
  st7789_lcd_wait_idle(pio, sm);

  uint32_t instr = st7789_encode_cmd(*cmd++, count - 1);
  pio_sm_put(pio, sm, instr);

  if (count >= 2) {
    count--;
    while (count > 0) {
      instr = 0;
      for (int i = 0; i < count && i < 4; ++i) {
        instr <<= 8;
        instr |= cmd[i];
      }
      if (count < 4)
        instr <<= 8 * (4 - count);
      pio_sm_put(pio, sm, instr);
      count -= 4;
    }
  }
}

static void st7789_create_dma_channels(PIO pio, uint sm, uint chan[2])
{
  chan[0] = dma_claim_unused_channel(true);
  chan[1] = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(chan[0]);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  dma_channel_configure(
        chan[0],       // Channel to be configured
        &c,            // The configuration we just created
        &pio->txf[sm], // The write address
        NULL,          // The initial read address - set later
        0,             // Number of transfers - set later
        false          // Don't start yet
    );

  c = dma_channel_get_default_config(chan[1]);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  dma_channel_configure(
        chan[1],       // Channel to be configured
        &c,            // The configuration we just created
        &pio->txf[sm], // The write address
        NULL,          // The initial read address - set later
        0,             // Number of transfers - set later
        false          // Don't start yet
    );
}

ST7789 st7789_init(PIO pio, uint sm) {
    ST7789 st;
    st.pio = pio;
    st.sm = sm;

    uint offset = pio_add_program(pio, &st7789_lcd_program);
    st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

    gpio_init(PIN_RESET);
    gpio_init(PIN_BL);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_RESET, 1);
    gpio_put(PIN_BL, 0);

    const uint8_t *cmd = st7789_init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }

    gpio_put(PIN_BL, 1);

    st7789_create_dma_channels(st.pio, st.sm, st.chan);
    st.chan_idx = 0;

    return st;
}

void st7789_start_pixels(ST7789* st, uint32_t num_pixels) {
  st7789_lcd_wait_idle(st->pio, st->sm);

  // RAMWR
  uint32_t cmd = st7789_encode_cmd(0x2c, num_pixels * 2);
  pio_sm_put(st->pio, st->sm, cmd);
}

void st7789_start_pixels_at(ST7789* st, uint8_t x, uint8_t y, uint32_t num_pixels) {
  uint8_t ca_cmd[] = {0x2a, 0x00, x, 0x00, 0xf0}; // CASET: column addresses from 0 to 240 (f0)
  uint8_t ra_cmd[] = {0x2b, 0x00, y, 0x00, 0xf0}; // RASET: row addresses from 0 to 240 (f0)

  lcd_write_cmd(st->pio, st->sm, ca_cmd, 5);
  lcd_write_cmd(st->pio, st->sm, ra_cmd, 5);

  // RAMWR
  uint32_t cmd = st7789_encode_cmd(0x2c, num_pixels * 2);
  pio_sm_put(st->pio, st->sm, cmd);
}

uint32_t st7789_add_pixels_at_cmd(uint32_t* buffer, uint8_t x, uint8_t y, uint32_t num_pixels) {
  uint8_t ca_cmd[] = {0x2a, 0x00, x, 0x00, 0xf0};
  uint8_t ra_cmd[] = {0x2b, 0x00, y, 0x00, 0xf0};

  *buffer++ = st7789_encode_cmd(0x2a, 4);
  *buffer++ = 0xf0 | ((uint32_t)x << 16);
  *buffer++ = st7789_encode_cmd(0x2b, 4);
  *buffer++ = 0xf0 | ((uint32_t)y << 16);
  *buffer++ = st7789_encode_cmd(0x2c, num_pixels * 2);

  return 5;
}

static inline void st7789_chain_or_trigger(uint this_chan, uint other_chan, uint ctrl)
{
  if (dma_channel_is_busy(other_chan)) {
    // Other channel is busy, chain this one to it
    dma_channel_hw_addr(this_chan)->al1_ctrl = ctrl;
    uint other_ctrl = dma_channel_hw_addr(other_chan)->ctrl_trig;
    other_ctrl &= ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS;
    other_ctrl |= this_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;
    dma_channel_hw_addr(other_chan)->al1_ctrl = other_ctrl;

    if (!dma_channel_is_busy(other_chan) && !dma_channel_is_busy(this_chan)) {
        // Manually start this channel
      dma_channel_hw_addr(this_chan)->ctrl_trig = ctrl;
    }
  } else {
    dma_channel_hw_addr(this_chan)->ctrl_trig = ctrl;
  }
}

void st7789_dma_buffer(ST7789* st, const uint32_t* data, uint len)
{
  uint this_chan = st->chan[st->chan_idx];
  uint other_chan = st->chan[st->chan_idx ^ 1];

  // Ensure any previous transfer is finished.
  dma_channel_wait_for_finish_blocking(this_chan);

  dma_channel_hw_addr(this_chan)->read_addr = (uintptr_t)data;
  dma_channel_hw_addr(this_chan)->transfer_count = len;
  uint ctrl = dma_channel_hw_addr(this_chan)->ctrl_trig;
  ctrl &= ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS;
  ctrl |= DMA_CH0_CTRL_TRIG_INCR_READ_BITS | (this_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);

  st7789_chain_or_trigger(this_chan, other_chan, ctrl);

  st->chan_idx ^= 1;
}

void st7789_dma_buffer_one_channel(ST7789* st, const uint32_t* data, uint len)
{
  uint chan = st->chan[st->chan_idx];

  dma_channel_wait_for_finish_blocking(chan);
  dma_channel_hw_addr(chan)->read_addr = (uintptr_t)data;
  dma_channel_hw_addr(chan)->transfer_count = len;
  uint ctrl = dma_channel_hw_addr(chan)->ctrl_trig;
  ctrl &= ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS;
  ctrl |= DMA_CH0_CTRL_TRIG_INCR_READ_BITS | (chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
  dma_channel_hw_addr(chan)->ctrl_trig = ctrl;
}

static uint32_t pixel_to_dma[3];

void st7789_dma_repeat_pixel(ST7789* st, uint16_t pixel, uint repeats)
{
  uint this_chan = st->chan[st->chan_idx];
  uint other_chan = st->chan[st->chan_idx ^ 1];

  dma_channel_wait_for_finish_blocking(this_chan);

  pixel_to_dma[st->chan_idx] = ((uint32_t)pixel << 16) | pixel;
  dma_channel_hw_addr(this_chan)->read_addr = (uintptr_t)&pixel_to_dma[st->chan_idx];
  dma_channel_hw_addr(this_chan)->transfer_count = (repeats+1)>>1;
  uint ctrl = dma_channel_hw_addr(this_chan)->ctrl_trig;
  ctrl &= ~(DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
  ctrl |= this_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;

  st7789_chain_or_trigger(this_chan, other_chan, ctrl);

  st->chan_idx ^= 1;
}

void st7789_dma_repeat_pixel_one_channel(ST7789* st, uint16_t pixel, uint repeats)
{
  uint chan = st->chan[st->chan_idx];

  dma_channel_wait_for_finish_blocking(chan);

  pixel_to_dma[2] = ((uint32_t)pixel << 16) | pixel;
  dma_channel_hw_addr(chan)->read_addr = (uintptr_t)&pixel_to_dma[2];
  dma_channel_hw_addr(chan)->transfer_count = (repeats+1)>>1;
  uint ctrl = dma_channel_hw_addr(chan)->ctrl_trig;
  ctrl &= ~(DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
  ctrl |= chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;
  dma_channel_hw_addr(chan)->ctrl_trig = ctrl;
}

