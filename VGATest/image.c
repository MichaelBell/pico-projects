#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

#include "vga.h"

#include "feeding_duck320.h"

uint32_t* data_pos;

#define DISPLAY_COLS 640
#define MAX_CMD_LINE_LEN (((DISPLAY_COLS + 5) / 6) + 1)

uint32_t red_buffer[2][MAX_CMD_LINE_LEN];

void display_loop() 
{
  while(1) 
  {
      __wfi();
  }
}

void __no_inline_not_in_flash_func(display_start_new_frame)() 
{
  data_pos = feeding_duck_dat;
}

void __no_inline_not_in_flash_func(display_next_row)(uint16_t row)
{
  if (data_pos < feeding_duck_dat + feeding_duck_dat_len) {
    uint32_t lens = *data_pos++;
    uint32_t red_len = (lens & 0x3ff) + 1;
    uint32_t green_len = ((lens >> 10) & 0x3ff) + 1;
    uint32_t blue_len = (lens >> 20) + 1;
    dma_channel_transfer_from_buffer_now(vga_red_dma, data_pos, red_len);
    data_pos += red_len + green_len + blue_len;
    assert(data_pos <= feeding_duck_dat + feeding_duck_dat_len);
  }
}
