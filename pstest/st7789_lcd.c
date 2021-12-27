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
#include "hardware/pwm.h"

#include "st7789_lcd.h"
#include "st7789_lcd.pio.h"

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

#define PIN_DIN PICOSYSTEM_LCD_MOSI_PIN
#define PIN_CLK PICOSYSTEM_LCD_SCLK_PIN
#define PIN_CS PICOSYSTEM_LCD_CSN_PIN  // Must be CLK - 1
#define PIN_DC PICOSYSTEM_LCD_DC_PIN
#define PIN_RESET PICOSYSTEM_LCD_RESET_PIN
#define PIN_BL PICOSYSTEM_BACKLIGHT_PIN

#define SERIAL_CLK_DIV 1.f

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
#if 1
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                         // Software reset
        1, 10, 0x11,                         // Exit sleep mode
        2, 2, 0x3a, 0x55,                    // Set colour mode to 16 bit
        2, 0, 0x35, 0x00,                    // Enable VSYNC output
        3, 0, 0x44, 0x00, 0xf0,              // VSYNC after 240 lines
        2, 0, 0xC6, 0x15,                    // 50Hz display
        2, 0, 0x36, 0x00,                    // Set MADCTL: row then column, refresh is bottom to top ????
        1, 2, 0x21,                          // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                          // Normal display on, then 10 ms delay
        1, 2, 0x29,                          // Main screen turn on, then wait 10 ms
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

static void st7789_create_dma_channels(ST7789* st)
{
  st->data_chan = dma_claim_unused_channel(true);
  st->ctrl_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(st->data_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pio_get_dreq(st->pio, st->sm, true));
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, st->ctrl_chan);
  channel_config_set_irq_quiet(&c, true);

  dma_channel_configure(
        st->data_chan, // Channel to be configured
        &c,            // The configuration we just created
        &st->pio->txf[st->sm], // The write address
        st->data_buf,  // The initial read address
        0,             // Number of transfers - set later
        false          // Don't start yet
    );

  c = dma_channel_get_default_config(st->ctrl_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, true);
  channel_config_set_ring(&c, true, 4);

  dma_channel_configure(
        st->ctrl_chan, // Channel to be configured
        &c,            // The configuration we just created
        &dma_hw->ch[st->data_chan].read_addr,  // Control block for data DMA channel
        st->ctrl_buf,  // The initial read address
        4,             // Number of transfers
        false          // Don't start yet
    );

  st->ctrl_ctrl = dma_hw->ch[st->data_chan].ctrl_trig;
}

static void st7789_write_tcb(ST7789* st, const uint32_t* data_ptr, uint32_t word_count, bool repeat)
{
  uint ctrl = st->ctrl_ctrl;
  if (!repeat) ctrl |= DMA_CH0_CTRL_TRIG_INCR_READ_BITS;
  if (word_count == 0) ctrl = 0;

  *st->ctrl_ptr++ = (uintptr_t)data_ptr;
  *st->ctrl_ptr++ = (uintptr_t)&st->pio->txf[st->sm];
  *st->ctrl_ptr++ = word_count;
  *st->ctrl_ptr++ = ctrl;
}

void st7789_init(ST7789* st, PIO pio, uint sm, uint32_t* data_buf, uint32_t* ctrl_buf)
{
    st->pio = pio;
    st->sm = sm;
    st->data_buf = st->data_ptr = data_buf;
    st->ctrl_buf = st->ctrl_ptr = ctrl_buf;
    st->transfer_in_progress = false;

    uint offset = pio_add_program(pio, &st7789_lcd_program);
    st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CS, PIN_DC, SERIAL_CLK_DIV);

    pwm_config cfg = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(PIN_BL), 65535);
    pwm_init(pwm_gpio_to_slice_num(PIN_BL), &cfg, true);
    gpio_set_function(PIN_BL, GPIO_FUNC_PWM);
    pwm_set_gpio_level(PIN_BL, 0);

    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);

    gpio_put(PIN_RESET, 1);

    const uint8_t *cmd = st7789_init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }

    pwm_set_gpio_level(PIN_BL, 10000);

    st7789_create_dma_channels(st);
}

void st7789_start_pixels_at(ST7789* st, uint8_t x, uint8_t y, uint8_t maxx, uint8_t maxy) {
  uint8_t ca_cmd[] = {0x2a, 0x00, x, 0x00, maxx};
  uint8_t ra_cmd[] = {0x2b, 0x00, y, 0x00, maxy};

  uint32_t* data_ptr = st->data_ptr;
  *st->data_ptr++ = st7789_encode_cmd(0x2a, 4);
  *st->data_ptr++ = maxx | ((uint32_t)x << 16);
  *st->data_ptr++ = st7789_encode_cmd(0x2b, 4);
  *st->data_ptr++ = maxy | ((uint32_t)y << 16);
  *st->data_ptr++ = st7789_encode_cmd(0x2c, (uint32_t)(maxx - x + 1) * (uint32_t)(maxy - y + 1) * 2);

  st7789_write_tcb(st, data_ptr, 5, false);
}

void st7789_trigger_transfer(ST7789* st)
{
  if (st->data_ptr != st->data_buf)
  {
    st->transfer_in_progress = true;

    // Zero length TCB finishes the chain
    st7789_write_tcb(st, NULL, 0, false);

    // Begin the transfer
    dma_channel_hw_addr(st->ctrl_chan)->read_addr = (uintptr_t)st->ctrl_buf;
    dma_channel_start(st->ctrl_chan);
  }

  // Reset pointers for next frame
  st->data_ptr = st->data_buf;
  st->ctrl_ptr = st->ctrl_buf;
}

void st7789_wait_for_transfer_complete(ST7789* st)
{
  if (st->transfer_in_progress)
  {
    while (!(dma_hw->intr & (1u << st->data_chan))) {}
    dma_hw->ints0 = 1u << st->data_chan;
    st->transfer_in_progress = false;
  }
}

void st7789_repeat_pixel(ST7789* st, uint16_t pixel, uint repeats)
{
  if (repeats & 1)
  {
    // Odd number of repeats, fix up the count to output one more pixel
    st->data_ptr[-1] += 0x1000;
  }

  *st->data_ptr = ((uint32_t)pixel << 16) | pixel;
  st7789_write_tcb(st, st->data_ptr++, (repeats+1) >> 1, true);
}

void st7789_dma_pixel_data(ST7789* st, const uint32_t* pixels, uint len)
{
  st7789_write_tcb(st, pixels, len, false);
}
