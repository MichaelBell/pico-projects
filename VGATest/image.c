#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "vga.h"
#include "flash.h"

//#include "feeding_duck320.h"
#include "perseverance.h"

static uint32_t* data_pos;

#define IMAGE_ROWS 240

#if 1
#define DISPLAY_COLS 640
#define DISPLAY_ROWS 480
#else
#if 0
#define DISPLAY_COLS 1280
#define DISPLAY_ROWS 720
#else
#define DISPLAY_COLS 1920
#define DISPLAY_ROWS 1080
#endif
#endif

#define MAX_CMD_LINE_LEN (((DISPLAY_COLS + 5) / 6) + 1)

typedef struct {
  uint32_t buffer[2][MAX_CMD_LINE_LEN];
  uint32_t* ptr;
  uint32_t len;
} channel_t;

channel_t channel[3];

#define red_chan channel[0]
#define green_chan channel[1]
#define blue_chan channel[2]

static uint16_t bufnum = 0;
static uint16_t display_row = 0;
static uint16_t image_row = 0;

static uint32_t zero = 0;

// Commands to core 0
#define CORE_CMD_INIT_FRAME      1

#define GRAYSCALE_BIT (1u << 31)

#define BLACK99 0b01000001111100000111110000011111  // 99 pixels of black
#define BLACK61 0b01000001111100000110000000000000  // 61 pixels of black

#ifdef IMAGE_IN_RAM
// This is the from RAM version
static void setup_next_line_ptr_and_len()
{
  if (display_row >= 120 && data_pos < image_dat + image_dat_len) {
    bufnum ^= 1;

    uint32_t lens = *data_pos++;
    if (lens & GRAYSCALE_BIT) {
      red_chan.len = lens & 0x3ff;
      red_chan.ptr = red_chan.buffer[bufnum];
      memcpy(red_chan.ptr + 2, data_pos, red_chan.len*sizeof(uint32_t));
      data_pos += red_chan.len;
      red_chan.ptr[red_chan.len + 2] = 0;
      red_chan.len += 3;

      green_chan.ptr = blue_chan.ptr = red_chan.ptr;
      green_chan.len = blue_chan.len = red_chan.len;
    }
    else
    {
      red_chan.len = (lens & 0x3ff);
      green_chan.len = ((lens >> 10) & 0x3ff);
      blue_chan.len = (lens >> 20);
      for (int i = 0; i < 3; ++i)
      {
        channel[i].ptr = channel[i].buffer[bufnum];
        memcpy(channel[i].ptr + 2, data_pos, channel[i].len*sizeof(uint32_t));
        data_pos += channel[i].len;
        channel[i].ptr[channel[i].len + 2] = 0;
        channel[i].len += 3;
      }
    }
    assert(data_pos <= image_dat + image_dat_len);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
  }
}
#else
// This streams from flash
static void __time_critical_func(setup_next_line_ptr_and_len)()
{
  static uint16_t scroll = 0;

  if (display_row == 0) scroll = (scroll + 1) & 0xfff;

  //if ((display_row & 1) == 0 && image_row < IMAGE_ROWS)
  if (display_row > (scroll >> 4) && image_row < IMAGE_ROWS)
  {
    bufnum ^= 1;
    ++image_row;

    // Extract lengths (1 word)
    uint32_t* lens;
    while (!flash_get_data(1, &lens));

    if (*lens & GRAYSCALE_BIT) {
      red_chan.len = *lens & 0x3ff;
      red_chan.ptr = red_chan.buffer[bufnum];

      flash_copy_data_blocking(red_chan.ptr + 2, red_chan.len);

      red_chan.ptr[red_chan.len + 2] = 0;
      red_chan.len += 3;

      green_chan.ptr = blue_chan.ptr = red_chan.ptr;
      green_chan.len = blue_chan.len = red_chan.len;
    }
    else
    {
      red_chan.len = (*lens & 0x3ff);
      green_chan.len = ((*lens >> 10) & 0x3ff);
      blue_chan.len = (*lens >> 20);
      for (int i = 0; i < 3; ++i)
      {
        channel[i].ptr = channel[i].buffer[bufnum];
        flash_copy_data_blocking(channel[i].ptr + 2, channel[i].len);
        channel[i].ptr[channel[i].len + 2] = 0;
        channel[i].len += 3;
      }
    }
    assert(data_pos <= image_dat + image_dat_len);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
  }
}
#endif

#define vga_red_dma   5
#define vga_green_dma 6
#define vga_blue_dma  7
#define vga_dma_channel_mask (0b111 << 5)
static uint32_t complete_dma_channel_bits = 0;

static void __time_critical_func(transfer_next_line)()
{
  dma_channel_transfer_from_buffer_now(vga_red_dma, red_chan.ptr, red_chan.len);
  dma_channel_transfer_from_buffer_now(vga_green_dma, green_chan.ptr, green_chan.len);
  dma_channel_transfer_from_buffer_now(vga_blue_dma, blue_chan.ptr, blue_chan.len);
  if (++display_row < DISPLAY_ROWS) 
    setup_next_line_ptr_and_len();
}

void __time_critical_func(dma_complete_handler)() 
{
  uint32_t ints = dma_hw->ints0 & vga_dma_channel_mask;
  dma_hw->ints0 = ints;
  complete_dma_channel_bits |= ints;

  if (complete_dma_channel_bits == vga_dma_channel_mask && display_row < DISPLAY_ROWS)
  {
    transfer_next_line();
  }
}

static void setup_dma_channels()
{
    // Setup DMAs
    dma_channel_config c = dma_channel_get_default_config(vga_red_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_red_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_red_dma,               // Channel to be configured
        &c,                        // The configuration we just created
        &vga_pio->txf[vga_red_sm], // The write address
        NULL,                      // The initial read address - set later
        0,                         // Number of transfers - set later
        false                      // Don't start yet
    );

    c = dma_channel_get_default_config(vga_green_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_green_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_green_dma,               // Channel to be configured
        &c,                          // The configuration we just created
        &vga_pio->txf[vga_green_sm], // The write address
        NULL,                        // The initial read address - set later
        0,                           // Number of transfers - set later
        false                        // Don't start yet
    );

    c = dma_channel_get_default_config(vga_blue_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_blue_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_blue_dma,               // Channel to be configured
        &c,                         // The configuration we just created
        &vga_pio->txf[vga_blue_sm], // The write address
        NULL,                       // The initial read address - set later
        0,                          // Number of transfers - set later
        false                       // Don't start yet
    );

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);

    dma_hw->inte0 = vga_dma_channel_mask;
}

void __time_critical_func(display_loop)() 
{
  setup_dma_channels();

#ifndef IMAGE_IN_RAM
  flash_set_stream(image_dat, image_dat_len);
#endif

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      channel[i].buffer[j][0] = BLACK99;
      channel[i].buffer[j][1] = BLACK61;
    }
  }

  while(1) 
  {
      uint instr = multicore_fifo_pop_blocking();
      uint cmd = instr & 0xff;
      uint data = instr >> 8;
      switch (cmd)
      {
        case CORE_CMD_INIT_FRAME:
        {
          data_pos = image_dat;
          display_row = 0;
          image_row = 0;
          flash_reset_stream();
          setup_next_line_ptr_and_len();
          
          irq_set_enabled(DMA_IRQ_0, false);
          dma_hw->ints0 = vga_dma_channel_mask;
          transfer_next_line();
          irq_set_enabled(DMA_IRQ_0, true);

          break;
        }
        default:
        {
          //!!
          __breakpoint();
        }
      }
  }
}

void __no_inline_not_in_flash_func(display_start_new_frame)() 
{
  multicore_fifo_push_blocking(CORE_CMD_INIT_FRAME);
}

void __no_inline_not_in_flash_func(display_end_frame)() 
{
  irq_set_enabled(DMA_IRQ_0, false);
  dma_channel_abort(vga_red_dma);
  dma_channel_abort(vga_green_dma);
  dma_channel_abort(vga_blue_dma);
  dma_hw->ints0 = vga_dma_channel_mask;
}