#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "display.h"

#include "hardware/gpio.h"

PS_DISPLAY disp;

const uint32_t ball_data[18] =
{
  0x0000f800, 0xf800f800, 0xf8000000,
  0xf800f800, 0xf800f800, 0xf800f800, 
  0xf800f800, 0xffffffff, 0xf800f800, 
  0xf800f800, 0xffffffff, 0xf800f800, 
  0xf800f800, 0xf800f800, 0xf800f800, 
  0x0000f800, 0xf800f800, 0xf8000000,
};

#define BOUNDS 234
const psd_vec ball_size = {6, 6};

typedef struct
{
  float x;
  float y;
} fvec;

typedef struct
{
  fvec pos;
  fvec speed;
  psd_sprite* sprite;
} ball;

#define NUM_BALLS 120
ball balls[NUM_BALLS];

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

  ps_display_init(&disp, PSD_BLACK);

  for (uint32_t i = 0; i < NUM_BALLS; ++i)
  {
    psd_vec start_pos;
    start_pos.x = rand() % BOUNDS;
    start_pos.y = rand() % BOUNDS;
    balls[i].pos.x = start_pos.x;
    balls[i].pos.y = start_pos.y;
    balls[i].speed.x = ((float)rand() / (RAND_MAX / 4)) - 2.f;
    balls[i].speed.y = ((float)rand() / (RAND_MAX / 4)) - 2.f;
    balls[i].sprite = ps_display_add_sprite(&disp, start_pos, ball_size, ball_data, sizeof(ball_data) / sizeof(uint32_t));
  }
  
  ps_display_render(&disp);

  while (1)
  {
    for (uint32_t i = 0; i < NUM_BALLS; ++i)
    {
      ball* ball_ptr = &balls[i];
      
      ball_ptr->pos.x += balls[i].speed.x;
      if (ball_ptr->pos.x < 0.f)
      {
        ball_ptr->pos.x = 0.f;
        ball_ptr->speed.x = -ball_ptr->speed.x;
      }
      else if (ball_ptr->pos.x > BOUNDS)
      {
        ball_ptr->pos.x = BOUNDS;
        ball_ptr->speed.x = -ball_ptr->speed.x;
      }

      balls[i].pos.y += balls[i].speed.y;
      if (ball_ptr->pos.y < 0.f)
      {
        ball_ptr->pos.y = 0.f;
        ball_ptr->speed.y = -ball_ptr->speed.y;
      }
      else if (ball_ptr->pos.y > BOUNDS)
      {
        ball_ptr->pos.y = BOUNDS;
        ball_ptr->speed.y = -ball_ptr->speed.y;
      }
    }

    ps_display_finish_render(&disp);

    for (uint32_t i = 0; i < NUM_BALLS; ++i)
    {
      psd_vec newpos;
      newpos.x = balls[i].pos.x;
      newpos.y = balls[i].pos.y;
      ps_display_move_sprite(&disp, balls[i].sprite, newpos);
    }

    ps_display_render(&disp);
  }

  return 0;
}