#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "st7789_lcd.h"
#include "AnimatedGIF.h"
#include "duck.h"

GIFIMAGE gif;
PIO pio = pio0;
uint sm = 0;
uint chan[2];
uint chan_idx = 0;

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240

int minx = 0;
int miny = 0;
int maxx = DISPLAY_WIDTH - 1;
int maxy = DISPLAY_HEIGHT - 1;
uint32_t pixel_data[2][DISPLAY_WIDTH*3];

void draw(GIFDRAW *pDraw) {
  int width = MIN(pDraw->iWidth, maxx - minx - pDraw->iX);
  int y = MIN(pDraw->iY + pDraw->y, maxy - miny) + miny;

  dma_channel_wait_for_finish_blocking(chan[chan_idx]);
  uint8_t* dataPtr = pDraw->pPixels;
  uint32_t* pixelPtr = pixel_data[chan_idx];

  if (pDraw->ucDisposalMethod == 2)
  {
    pixelPtr += st7789_add_pixels_at_cmd(pixelPtr, pDraw->iX + minx, y, width);
    uint16_t bgPixel = pDraw->pPalette[pDraw->ucBackground];
    for (int x = 0; x < width; ++x) {
      uint8_t data = *dataPtr++;
      uint16_t pixel;
      if (data == pDraw->ucTransparent) 
        pixel = bgPixel;
      else
        pixel = pDraw->pPalette[data];
      if (x & 1) *pixelPtr++ |= pixel;
      else *pixelPtr = pixel << 16;
    }
    if (width & 1) pixelPtr++;
  }
  else
  {
    if (pDraw->ucHasTransparency) {
      uint8_t tData = pDraw->ucTransparent;
      int x = 0, count = 0;

      while (x < width) {
        while (*dataPtr++ == tData && x < width) ++x;
        if (x == width) break;

        // Back up as we incremented past the first non-transparent pixel
        dataPtr--;

        int startx = x;
        uint32_t* startPixelPtr = pixelPtr;
        pixelPtr += 5;
        for (; x < width && *dataPtr != tData; ++x) {
          uint16_t pixel = pDraw->pPalette[*dataPtr++];
          if ((x - startx) & 1) *pixelPtr++ |= pixel;
          else *pixelPtr = pixel << 16;
        }
        if ((x - startx) & 1) pixelPtr++;

        st7789_add_pixels_at_cmd(startPixelPtr, pDraw->iX + minx + startx, y, x - startx);
      }
    } 
    else 
    {
      pixelPtr += st7789_add_pixels_at_cmd(pixelPtr, pDraw->iX + minx, y, width);
      for (int x = 0; x < width; ++x) {
        uint16_t pixel = pDraw->pPalette[*dataPtr++];
        if (x & 1) *pixelPtr++ |= pixel;
        else *pixelPtr = pixel << 16;
      }
      if (width & 1) pixelPtr++;
    }
  }

  st7789_dma_buffer(chan, chan_idx, pixel_data[chan_idx], pixelPtr - pixel_data[chan_idx]);
  chan_idx ^= 1;
}

int main()
{
  stdio_init_all();

  st7789_init(pio, sm);
  st7789_create_dma_channels(pio, sm, chan);

  st7789_start_pixels(pio, sm, DISPLAY_WIDTH * DISPLAY_HEIGHT);
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
        uint32_t frame_time = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;
        delay -= frame_time;
        printf("Frame time %dms\n", frame_time);
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
