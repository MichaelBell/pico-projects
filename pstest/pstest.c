#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "st7789_lcd.h"

ST7789 st;

#define DISPLAY_ROWS 240
#define DISPLAY_COLS 240

#define ST_DATA_LEN (32 + ((DISPLAY_COLS + 1) >> 1))

int main()
{
  stdio_init_all();

  gpio_init(PICOSYSTEM_LED_G_PIN);
  gpio_init(PICOSYSTEM_LED_R_PIN);
  gpio_init(PICOSYSTEM_LED_B_PIN);
  gpio_set_dir(PICOSYSTEM_LED_R_PIN, GPIO_OUT);
  gpio_set_dir(PICOSYSTEM_LED_G_PIN, GPIO_OUT);
  gpio_set_dir(PICOSYSTEM_LED_B_PIN, GPIO_OUT);
  gpio_put(PICOSYSTEM_LED_G_PIN, 1);
  gpio_put(PICOSYSTEM_LED_R_PIN, 0);
  gpio_put(PICOSYSTEM_LED_B_PIN, 0);

  st = st7789_init(pio0, 0);

  for (uint32_t i = 0; ; ++i)
  {
    st7789_start_pixels_at(&st, 60 * (i & 3), 30 * ((i & 0x1f) >> 2), 60, 30);
    st7789_dma_repeat_pixel_one_channel(&st, i, 60 * 30);
  }

  return 0;
}