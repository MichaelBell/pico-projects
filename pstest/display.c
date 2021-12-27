#include "pico/stdlib.h"

#include "display.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

void ps_display_init(PS_DISPLAY* disp)
{
  gpio_init(PICOSYSTEM_LCD_VSYNC_PIN);
  gpio_set_dir(PICOSYSTEM_LCD_VSYNC_PIN, GPIO_IN);

  st7789_init(&disp->st, pio0, 0, disp->buffer[0], disp->buffer[1]);

  st7789_start_pixels_at(&disp->st, 0, 0, DISPLAY_ROWS - 1, DISPLAY_COLS - 1);
  st7789_repeat_pixel(&disp->st, 0, DISPLAY_ROWS * DISPLAY_COLS);
  st7789_trigger_transfer(&disp->st);
  st7789_wait_for_transfer_complete(&disp->st);
}

void ps_display_draw_rect(PS_DISPLAY* disp, psd_box bounds, psd_colour colour)
{
  // TODO
}

void ps_display_draw_frect(PS_DISPLAY* disp, psd_box bounds, psd_colour colour)
{
  st7789_start_pixels_at(&disp->st, bounds.tl.x, bounds.tl.y, bounds.br.x, bounds.br.y);
  st7789_repeat_pixel(&disp->st, colour.c, (bounds.br.x - bounds.tl.x + 1) * (bounds.br.y - bounds.tl.y + 1));
}

void ps_display_render(PS_DISPLAY* disp)
{
  // Wait for a new VSYNC - this should be changed to a rising edge GPIO interrupt
  while(gpio_get(PICOSYSTEM_LCD_VSYNC_PIN)) {}
  while (!gpio_get(PICOSYSTEM_LCD_VSYNC_PIN)) {}

  st7789_trigger_transfer(&disp->st);

  // TODO
  st7789_wait_for_transfer_complete(&disp->st);
}
