#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "st7789_lcd.h"

ST7789 st;

#define DISPLAY_ROWS 240
#define DISPLAY_COLS 240

int main()
{
  stdio_init_all();

  gpio_init_mask(0xffe100);
  gpio_set_dir_in_masked(0xff0100);
  for (uint32_t i = 0; i < 24; i++)
  {
    if ((1 << i) & 0xff0100) gpio_pull_up(i);
  }

  st = st7789_init(pio0, 0);

  st7789_start_pixels(&st, DISPLAY_ROWS * DISPLAY_COLS);
  st7789_dma_repeat_pixel_one_channel(&st, 0, DISPLAY_ROWS * DISPLAY_COLS);

  uint8_t posx = 117;
  uint8_t posy = 117;
  const uint8_t width = 4;
  const uint16_t colour = 0xf800;
  st7789_start_pixels_at(&st, posx, posy, width, width);
  st7789_dma_repeat_pixel_one_channel(&st, colour, width * width);

  while (1)
  {
    absolute_time_t start_time = get_absolute_time();

    uint8_t newposx = posx;
    uint8_t newposy = posy;

    if (gpio_get(PICOSYSTEM_SW_UP_PIN) == 0) newposy--;
    else if (gpio_get(PICOSYSTEM_SW_DOWN_PIN) == 0) newposy++;
    if (gpio_get(PICOSYSTEM_SW_LEFT_PIN) == 0) newposx--;
    else if (gpio_get(PICOSYSTEM_SW_RIGHT_PIN) == 0) newposx++;

    if (newposx != posx || newposy != posy)
    {
      st7789_start_pixels_at(&st, posx, posy, width, width);
      st7789_dma_repeat_pixel_one_channel(&st, 0, width * width);
      st7789_start_pixels_at(&st, newposx, newposy, width, width);
      st7789_dma_repeat_pixel_one_channel(&st, 0xf800, width * width);

      posx = newposx;
      posy = newposy;
    }

    uint32_t frame_time = absolute_time_diff_us(start_time, get_absolute_time());

    int delay = 20000;
    delay -= absolute_time_diff_us(start_time, get_absolute_time());
    if (delay > 0)
      sleep_us(delay);

    start_time = get_absolute_time();

  }

  return 0;
}