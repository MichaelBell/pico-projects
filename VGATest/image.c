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

static uint32_t red_buffer[2][MAX_CMD_LINE_LEN];
static uint32_t green_buffer[2][MAX_CMD_LINE_LEN];
static uint32_t blue_buffer[2][MAX_CMD_LINE_LEN];

static uint32_t* red_ptr = NULL;
static uint32_t red_len = 0;
static uint32_t* green_ptr = NULL;
static uint32_t green_len = 0;
static uint32_t* blue_ptr = NULL;
static uint32_t blue_len = 0;
static uint16_t bufnum = 0;

static uint32_t zero = 0;
static uint32_t row_ready = 0xffff;

#define CORE_CMD_INIT_FRAME      1
#define CORE_CMD_SETUP_NEXT_LINE 2


#define BLACK99 0b01000001111100000111110000011111  // 99 pixels of black
#define BLACK61 0b01000001111100000110000000000000  // 61 pixels of black

void setup_next_line_ptr_and_len(uint row)
{
  if (row >= 120 && data_pos < feeding_duck_dat + feeding_duck_dat_len) {
    bufnum ^= 1;

    uint32_t lens = *data_pos++;
    red_len = (lens & 0x3ff);
    green_len = ((lens >> 10) & 0x3ff);
    blue_len = (lens >> 20);
    red_ptr = red_buffer[bufnum];
    green_ptr = green_buffer[bufnum];
    blue_ptr = blue_buffer[bufnum];
    memcpy(red_ptr + 2, data_pos, red_len*sizeof(uint32_t));
    data_pos += red_len;
    memcpy(green_ptr + 2, data_pos, green_len*sizeof(uint32_t));
    data_pos += green_len;
    memcpy(blue_ptr + 2, data_pos, blue_len*sizeof(uint32_t));
    data_pos += blue_len;
    assert(data_pos <= feeding_duck_dat + feeding_duck_dat_len);

    red_ptr[red_len + 2] = 0;
    green_ptr[green_len + 2] = 0;
    blue_ptr[blue_len + 2] = 0;
    //green_ptr[2] = 0;
    //blue_ptr[2] = 0;
    red_len += 3;
    green_len += 3;
    blue_len += 3;
  }
  else
  {
    red_len = green_len = blue_len = 1;
    red_ptr = green_ptr = blue_ptr = &zero;
  }
  row_ready = row;
  __mem_fence_release();
}

void display_loop() 
{
  for (int i = 0; i < 2; ++i) {
    red_buffer[i][0] = BLACK99;
    red_buffer[i][1] = BLACK61;
    green_buffer[i][0] = BLACK99;
    green_buffer[i][1] = BLACK61;
    blue_buffer[i][0] = BLACK99;
    blue_buffer[i][1] = BLACK61;
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
    dma_channel_transfer_from_buffer_now(vga_red_dma, red_ptr, red_len);
    dma_channel_transfer_from_buffer_now(vga_green_dma, green_ptr, green_len);
    dma_channel_transfer_from_buffer_now(vga_blue_dma, blue_ptr, blue_len);
  }
  else
  {
    dma_channel_transfer_from_buffer_now(vga_red_dma, &zero, 1);
    dma_channel_transfer_from_buffer_now(vga_green_dma, &zero, 1);
    dma_channel_transfer_from_buffer_now(vga_blue_dma, &zero, 1);
  }
  multicore_fifo_push_blocking(CORE_CMD_SETUP_NEXT_LINE | ((uint32_t)row << 8));
}
