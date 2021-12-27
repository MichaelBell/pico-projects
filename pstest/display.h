#include <stdint.h>
#include "st7789_lcd.h"

#define PSD_CMD_BUFFER_SIZE 4096
#define PSD_MAX_SPRITES 256

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

extern const psd_colour PSD_BLACK;
extern const psd_colour PSD_RED;
extern const psd_colour PSD_GREEN;
extern const psd_colour PSD_BLUE;

typedef struct
{
  psd_vec pos;
  psd_vec size;
  const uint32_t* data;
  uint32_t depth : 6;
  uint32_t enabled : 1;
  uint32_t draw : 1;
  uint32_t data_len : 24;
} psd_sprite;

typedef struct
{
  ST7789 st;
  uint32_t buffer[2][PSD_CMD_BUFFER_SIZE];

  psd_sprite sprites[PSD_MAX_SPRITES];
  uint16_t max_sprite;
  
  psd_colour background;
} PS_DISPLAY;

void ps_display_init(PS_DISPLAY* disp, psd_colour background);

psd_sprite* ps_display_add_sprite(PS_DISPLAY* disp, psd_vec pos, psd_vec size, const uint32_t* data, uint32_t data_len);
void ps_display_move_sprite(PS_DISPLAY* disp, psd_sprite* sprite, psd_vec pos);

void ps_display_draw_rect(PS_DISPLAY* disp, psd_vec pos, psd_vec size, psd_colour colour);
void ps_display_draw_frect(PS_DISPLAY* disp, psd_vec pos, psd_vec size, psd_colour colour);

void ps_display_render(PS_DISPLAY* disp);
void ps_display_finish_render(PS_DISPLAY* disp);