#include <stdint.h>
#include "st7789_lcd.h"

#define PSD_CMD_BUFFER_SIZE 4096
#define PSD_CMD_BUFFER_SIZE_WORDS (PSD_CMD_BUFFER_SIZE / 4)

#define DISPLAY_ROWS 240
#define DISPLAY_COLS 240

typedef struct
{
  uint8_t x;
  uint8_t y;
} psd_vec;

typedef struct
{
  psd_vec tl;
  psd_vec br;
} psd_box;

typedef union
{
  struct {
    uint16_t b : 5;
    uint16_t g : 6;
    uint16_t r : 5;
  };
  uint16_t c;
} psd_colour;

typedef struct
{
  ST7789 st;
  uint32_t buffer[2][PSD_CMD_BUFFER_SIZE_WORDS];
} PS_DISPLAY;

void ps_display_init(PS_DISPLAY* disp);

void ps_display_draw_rect(PS_DISPLAY* disp, psd_box bounds, psd_colour colour);
void ps_display_draw_frect(PS_DISPLAY* disp, psd_box bounds, psd_colour colour);

void ps_display_render(PS_DISPLAY* disp);
