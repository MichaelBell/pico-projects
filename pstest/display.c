#include "pico/stdlib.h"

#include "display.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

const psd_colour PSD_BLACK = {.c = 0};
const psd_colour PSD_RED = {.r = 0x1f};
const psd_colour PSD_GREEN = {.g = 0x3f};
const psd_colour PSD_BLUE = {.b = 0x1f};

void ps_display_init(PS_DISPLAY* disp, psd_colour bg)
{
  gpio_init(PICOSYSTEM_LCD_VSYNC_PIN);
  gpio_set_dir(PICOSYSTEM_LCD_VSYNC_PIN, GPIO_IN);

  disp->background = bg;

  st7789_init(&disp->st, pio0, 0, disp->buffer[0], disp->buffer[1]);

  st7789_start_pixels_at(&disp->st, 0, 0, DISPLAY_ROWS - 1, DISPLAY_COLS - 1);
  st7789_repeat_pixel(&disp->st, disp->background.c, DISPLAY_ROWS * DISPLAY_COLS);
  st7789_trigger_transfer(&disp->st);
  st7789_wait_for_transfer_complete(&disp->st);
}

void ps_display_draw_rect(PS_DISPLAY* disp, psd_vec pos, psd_vec size, psd_colour colour)
{
  // TODO
}

void ps_display_draw_frect(PS_DISPLAY* disp, psd_vec pos, psd_vec size, psd_colour colour)
{
  st7789_start_pixels_at(&disp->st, pos.x, pos.y, pos.x + size.x - 1, pos.y + size.y - 1);
  st7789_repeat_pixel(&disp->st, colour.c, size.x * size.y);
}

psd_sprite* ps_display_add_sprite(PS_DISPLAY* disp, psd_vec pos, psd_vec size, const uint32_t* data, uint32_t data_len)
{
  psd_sprite* sprite = &disp->sprites[disp->max_sprite++];
  sprite->pos = pos;
  sprite->size = size;
  sprite->data = data;
  sprite->data_len = data_len;
  sprite->depth = 0;
  sprite->enabled = 1;
  sprite->draw = 1;

  return sprite;
}

void ps_display_move_sprite(PS_DISPLAY* disp, psd_sprite* sprite, psd_vec pos)
{
  if (sprite->pos.x != pos.x || sprite->pos.y != pos.y)
  {
    ps_display_draw_frect(disp, sprite->pos, sprite->size, disp->background);
    
    sprite->pos = pos;
    sprite->draw = 1;

    // TODO Overlapping sprites.
  }
}

void ps_display_render(PS_DISPLAY* disp)
{
  // Queue draw for sprites
  for (uint16_t i = 0; i < disp->max_sprite; ++i)
  {
    psd_sprite* sprite = &disp->sprites[i];
    if (sprite->draw)
    {
      st7789_start_pixels_at(&disp->st, sprite->pos.x, sprite->pos.y, sprite->pos.x + sprite->size.x - 1, sprite->pos.y + sprite->size.y - 1);
      st7789_dma_pixel_data(&disp->st, sprite->data, sprite->data_len);
    }
  }

  // Wait for a new VSYNC - this should be changed to a rising edge GPIO interrupt
  while(gpio_get(PICOSYSTEM_LCD_VSYNC_PIN)) {}
  while (!gpio_get(PICOSYSTEM_LCD_VSYNC_PIN)) {}

  st7789_trigger_transfer(&disp->st);
}

void ps_display_finish_render(PS_DISPLAY* disp)
{
  st7789_wait_for_transfer_complete(&disp->st);
}
