#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

#include "vga.h"

#include "feeding_duck320.h"

static uint32_t* data_pos;

#define DISPLAY_COLS 640
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

static uint32_t zero = 0;
static uint32_t row_ready = 0xffff;

#define CORE_CMD_INIT_FRAME      1
#define CORE_CMD_SETUP_NEXT_LINE 2


#define BLACK99 0b01000001111100000111110000011111  // 99 pixels of black
#define BLACK61 0b01000001111100000110000000000000  // 61 pixels of black

void dma_complete_handler() {

}

void setup_next_line_ptr_and_len(uint row)
{
  if (row >= 120 && data_pos < feeding_duck_dat + feeding_duck_dat_len) {
    bufnum ^= 1;

    uint32_t lens = *data_pos++;
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
    assert(data_pos <= feeding_duck_dat + feeding_duck_dat_len);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
  }
  row_ready = row;
  __mem_fence_release();
}

void display_loop() 
{
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
          data_pos = feeding_duck_dat;
          setup_next_line_ptr_and_len(0);
          break;
        }
        case CORE_CMD_SETUP_NEXT_LINE:
        {
          setup_next_line_ptr_and_len(data);
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

void __no_inline_not_in_flash_func(display_next_row)(uint16_t row)
{
  __mem_fence_acquire();
  if (row_ready == row - 1)
  {
    dma_channel_transfer_from_buffer_now(vga_red_dma, red_chan.ptr, red_chan.len);
    dma_channel_transfer_from_buffer_now(vga_green_dma, green_chan.ptr, green_chan.len);
    dma_channel_transfer_from_buffer_now(vga_blue_dma, blue_chan.ptr, blue_chan.len);
  }
  else
  {
    dma_channel_transfer_from_buffer_now(vga_red_dma, &zero, 1);
    dma_channel_transfer_from_buffer_now(vga_green_dma, &zero, 1);
    dma_channel_transfer_from_buffer_now(vga_blue_dma, &zero, 1);
  }
  multicore_fifo_push_blocking(CORE_CMD_SETUP_NEXT_LINE | ((uint32_t)row << 8));
}

void __no_inline_not_in_flash_func(display_end_frame)() 
{
  dma_channel_abort(vga_red_dma);
  dma_channel_abort(vga_green_dma);
  dma_channel_abort(vga_blue_dma);
}