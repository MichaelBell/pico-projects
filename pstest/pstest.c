#include <stdio.h>
#include "pico/stdlib.h"

#include "display.h"

#include "hardware/gpio.h"

PS_DISPLAY disp;

int main()
{
  stdio_init_all();

  gpio_init_mask(0xffe000);
  gpio_set_dir_in_masked(0xff0000);
  gpio_set_dir_out_masked(0x00e000);
  for (uint32_t i = 16; i < 24; i++)
  {
    gpio_pull_up(i);
  }

  ps_display_init(&disp);

  psd_vec pos;
  pos.x = 117;
  pos.y = 117;
  const psd_colour red = {.r = 0x1f};
  const psd_colour blue = {.b = 0x1f};
  const psd_colour black = {.c = 0};
  psd_box ballbox;
  ballbox.tl = pos;
  ballbox.br.x = pos.x + 4;
  ballbox.br.y = pos.y + 4;
  ps_display_draw_frect(&disp, ballbox, red);
  ps_display_render(&disp);

  while (1)
  {
    absolute_time_t start_time = get_absolute_time();

    psd_vec newpos = pos;

    if (gpio_get(PICOSYSTEM_SW_UP_PIN) == 0) newpos.y--;
    else if (gpio_get(PICOSYSTEM_SW_DOWN_PIN) == 0) newpos.y++;
    if (gpio_get(PICOSYSTEM_SW_LEFT_PIN) == 0) newpos.x--;
    else if (gpio_get(PICOSYSTEM_SW_RIGHT_PIN) == 0) newpos.x++;

    if (newpos.x != pos.x || newpos.y != pos.y)
    {
      ballbox.tl = pos;
      ballbox.br.x = pos.x + 4;
      ballbox.br.y = pos.y + 4;
      ps_display_draw_frect(&disp, ballbox, black);

      pos = newpos;
      ballbox.tl = pos;
      ballbox.br.x = pos.x + 4;
      ballbox.br.y = pos.y + 4;
      ps_display_draw_frect(&disp, ballbox, red);
    }

    ps_display_render(&disp);
  }

  return 0;
}