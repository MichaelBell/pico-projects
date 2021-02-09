#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "st7789_lcd.h"
#include "AnimatedGIF.h"
#include "duck.h"

GIFIMAGE gif;
PIO pio = pio0;
uint sm = 0;
uint chan[2];

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240

int minx = 0;
int miny = 0;
int maxx = DISPLAY_WIDTH - 1;
int maxy = DISPLAY_HEIGHT - 1;

void draw(GIFDRAW *pDraw) {
  int width = MIN(pDraw->iWidth - pDraw->iX, maxx - minx);
  int y = MIN(pDraw->iY + pDraw->y, maxy) + miny;

  uint16_t pixels[DISPLAY_WIDTH];
  uint8_t* dataPtr = pDraw->pPixels;

  if (pDraw->ucDisposalMethod == 2)
  {
    st7789_start_pixels_at(pio, sm, pDraw->iX + minx, y);
    uint16_t bgPixel = pDraw->pPalette[pDraw->ucBackground];
    for (int x = 0; x < pDraw->iWidth; ++x) {
      uint8_t data = *dataPtr++;
      if (data == pDraw->ucTransparent) 
        pixels[x] = bgPixel;
      else
        pixels[x] = pDraw->pPalette[data];
    }
    st7789_dma_pixels_one_channel(chan[0], pixels, width);
  }
  else
  {
    if (pDraw->ucHasTransparency) {
      uint8_t tData = pDraw->ucTransparent;
      int x = 0, count = 0;

      while (x < width) {
        while (*dataPtr++ == tData && x < width) ++x;
        if (x == width) break;
        st7789_start_pixels_at(pio, sm, pDraw->iX + minx + x, y);

        // Back up as we incremented past the first non-transparent pixel
        dataPtr--;

        int startx = x;
        for (; x < width && *dataPtr != tData; ++x)
          pixels[x] = pDraw->pPalette[*dataPtr++];

        st7789_dma_pixels_one_channel(chan[0], pixels + startx, x - startx);
      }
    } 
    else 
    {
      st7789_start_pixels_at(pio, sm, pDraw->iX + minx, y);
      for (int x = 0; x < pDraw->iWidth; ++x) {
        pixels[x] = pDraw->pPalette[*dataPtr++];
      }
      st7789_dma_pixels_one_channel(chan[0], pixels, width);
    }
  }
}

int main()
{
  stdio_init_all();

  st7789_init(pio, sm);
  st7789_create_dma_channels(pio, sm, chan);

  st7789_start_pixels(pio, sm);
  st7789_dma_repeat_pixel_one_channel(chan[0], 0, DISPLAY_WIDTH * DISPLAY_HEIGHT);

  GIF_begin(&gif, LITTLE_ENDIAN_PIXELS, GIF_PALETTE_RGB565);
  while (1) {
    if (!GIF_openRAM(&gif, (uint8_t *)duck_gif, sizeof(duck_gif), draw)) {
      printf("Open failed\n");
    } 
    else
    {
      minx = MAX(0, (DISPLAY_WIDTH - gif.iCanvasWidth) >> 1);
      maxx = MIN(DISPLAY_WIDTH - 1, minx + gif.iCanvasWidth);
      miny = MAX(0, (DISPLAY_HEIGHT - gif.iCanvasHeight) >> 1);
      maxy = MIN(DISPLAY_HEIGHT - 1, minx + gif.iCanvasHeight);

      int delay = 10;
      absolute_time_t start_time = get_absolute_time();
      while (GIF_playFrame(&gif, &delay)) {
        st7789_stop_pixels(pio, sm);

        delay -= absolute_time_diff_us(start_time, get_absolute_time()) / 1000;
        printf("Frame delay %dms\n", delay);
        delay -= 100;
        if (delay > 0)
          sleep_ms(delay);

        start_time = get_absolute_time();
      }
      GIF_close(&gif);
      printf("Loop done\n");
    }
  }

  return 0;
}
