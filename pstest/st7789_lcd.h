typedef struct {
  PIO pio;
  uint sm;
  uint chan[2];
  uint chan_idx;
} ST7789;

// Setup the display and return CB
ST7789 st7789_init(PIO pio, uint sm);

// Restart pixel output synchronously, pixels drawn at last location
// (or full screen if no location ever given) 
void st7789_start_pixels(ST7789* st, uint32_t num_pixels);

// Restart pixel output synchronously, at a given location
void st7789_start_pixels_at(ST7789* st, uint8_t x, uint8_t y, uint8_t width, uint8_t height);

// Write a command to restart pixel output into a buffer for DMA
uint32_t st7789_add_pixels_at_cmd(uint32_t* buffer, uint8_t x, uint8_t y, uint8_t width, uint8_t height);

// Send data to the display via DMA, chaining to the previous DMA
void st7789_dma_buffer(ST7789* st, const uint32_t* data, uint len);

// Send a single pixel multiple times, chaining to the previous DMA
void st7789_dma_repeat_pixel(ST7789* st, uint16_t pixel, uint repeats);

// Wait for next DMA channel to finish previous transfer, i.e. wait for last but one
// transfer to finish.  After this point you can reuse the buffer with index chan_idx.
static inline void st7789_wait_for_next_dma_chan(ST7789* st) {
    dma_channel_wait_for_finish_blocking(st->chan[st->chan_idx]);
}

// Wait for previous DMA to finish, then send pixels
void st7789_dma_buffer_one_channel(ST7789* st, const uint32_t* data, uint len);
void st7789_dma_repeat_pixel_one_channel(ST7789* st, uint16_t pixel, uint repeats);
