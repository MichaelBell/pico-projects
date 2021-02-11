void st7789_init(PIO pio, uint sm);
void st7789_start_pixels(PIO pio, uint sm, uint32_t num_pixels);
void st7789_start_pixels_at(PIO pio, uint sm, uint8_t x, uint8_t y, uint32_t num_pixels);
uint32_t st7789_add_pixels_at_cmd(uint32_t* buffer, uint8_t x, uint8_t y, uint32_t num_pixels);
void st7789_create_dma_channels(PIO pio, uint sm, uint chan[2]);
void st7789_dma_buffer(uint chan[2], uint chan_idx, const uint32_t* data, uint len);
void st7789_dma_repeat_pixel(uint chan[2], uint chan_idx, uint16_t pixel, uint repeats);
void st7789_dma_buffer_one_channel(uint chan, const uint32_t* data, uint len);
void st7789_dma_repeat_pixel_one_channel(uint chan, uint16_t pixel, uint repeats);

#include "st7789_lcd.pio.h"
