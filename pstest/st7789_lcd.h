#include "hardware/pio.h"

typedef struct {
  PIO pio;
  uint sm;
  uint data_chan;
  uint ctrl_chan;
  uint ctrl_ctrl;
  uint32_t* data_buf;
  uint32_t* ctrl_buf;
  uint32_t* data_ptr;
  uint32_t* ctrl_ptr;
  volatile bool transfer_in_progress;
} ST7789;

// Setup the display and return CB
void st7789_init(ST7789* st, PIO pio, uint sm, uint32_t* data_buf, uint32_t* ctrl_buf);

// Queue data transfer into a given box
void st7789_start_pixels_at(ST7789* st, uint8_t x, uint8_t y, uint8_t maxx, uint8_t maxy);

// Write all queued data
void st7789_trigger_transfer(ST7789* st);

// Wait for previous transfer to complete, this must be called before queueing any further commands
void st7789_wait_for_transfer_complete(ST7789* st);

// Queue a single pixel multiple times.  Repeats is the number of times the pixel is repeated.
void st7789_repeat_pixel(ST7789* st, uint16_t pixel, uint repeats);

// Queue pixel data from memory, note len is data length in 32-bit words, not pixels.
void st7789_dma_pixel_data(ST7789* st, const uint32_t* pixels, uint len);