#include <stdio.h>
#include "pico/stdlib.h"

#include "pico/sd_card.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "AnimatedGIF.h"
#include "st7789_lcd.h"

GIFIMAGE gif;
ST7789 st;

#define DISPLAY_ROWS 240
#define DISPLAY_COLS 240

#define ST_DATA_LEN (32 + ((DISPLAY_COLS + 1) >> 1))

struct {
  int minx;
  int miny;
  int maxx;
  int maxy;
  uint32_t st_data[2][ST_DATA_LEN];
} frame;

void sdgif_draw(GIFDRAW* pDraw)
{
  int width = MIN(pDraw->iWidth, frame.maxx - frame.minx - pDraw->iX);
  int y = MIN(pDraw->iY + pDraw->y, frame.maxy - frame.miny) + frame.miny;

  st7789_wait_for_next_dma_chan(&st);
  uint8_t* dataPtr = pDraw->pPixels;
  uint32_t* stPtr = frame.st_data[st.chan_idx];

  if (pDraw->ucDisposalMethod == 2)
  {
    stPtr += st7789_add_pixels_at_cmd(stPtr, pDraw->iX + frame.minx, y, width);
    uint16_t bgPixel = pDraw->pPalette[pDraw->ucBackground];
    for (int x = 0; x < width; ++x) {
      uint8_t data = *dataPtr++;
      uint16_t pixel;
      if (data == pDraw->ucTransparent) 
        pixel = bgPixel;
      else
        pixel = pDraw->pPalette[data];
      if (x & 1) *stPtr++ |= pixel;
      else *stPtr = pixel << 16;
    }
    if (width & 1) stPtr++;
  }
  else
  {
    if (pDraw->ucHasTransparency) {
      uint8_t tData = pDraw->ucTransparent;
      int x = 0, count = 0;
      uint32_t* stEndPtr = frame.st_data[st.chan_idx] + ST_DATA_LEN - 6;

      while (x < width) {
        while (*dataPtr++ == tData && x < width) ++x;
        if (x == width) break;

        // Back up as we incremented past the first non-transparent pixel
        dataPtr--;

        int startx = x;
        uint32_t* stStartPtr = stPtr;
        stPtr += 5;
        for (; x < width && *dataPtr != tData; ++x) {
          uint16_t pixel = pDraw->pPalette[*dataPtr++];
          if ((x - startx) & 1) {
            *stPtr++ |= pixel;
            if (stPtr >= stEndPtr) {
              ++x;
              break;
            }
          }
          else *stPtr = pixel << 16;
        }
        if ((x - startx) & 1) stPtr++;

        st7789_add_pixels_at_cmd(stStartPtr, pDraw->iX + frame.minx + startx, y, x - startx);

        if (stPtr >= stEndPtr && x < width)
        {
          st7789_dma_buffer(&st, frame.st_data[st.chan_idx], stPtr - frame.st_data[st.chan_idx]);
          st7789_wait_for_next_dma_chan(&st);
          stPtr = frame.st_data[st.chan_idx];
          stEndPtr = frame.st_data[st.chan_idx] + ST_DATA_LEN - 6;
        }
      }
    } 
    else 
    {
      stPtr += st7789_add_pixels_at_cmd(stPtr, pDraw->iX + frame.minx, y, width);
      for (int x = 0; x < width; ++x) {
        uint16_t pixel = pDraw->pPalette[*dataPtr++];
        if (x & 1) *stPtr++ |= pixel;
        else *stPtr = pixel << 16;
      }
      if (width & 1) stPtr++;
    }
  }

  st7789_dma_buffer(&st, frame.st_data[st.chan_idx], stPtr - frame.st_data[st.chan_idx]);
}

#define SECTOR_START 65536
#define SD_BUFFER_SECTORS 4
#define FILE_LEN 4892075

struct {
  uint32_t buffer[128 * SD_BUFFER_SECTORS];
  uint32_t buffer_start;
} sd_data;

void sdgif_init()
{
  sd_readblocks_sync(sd_data.buffer, SECTOR_START, SD_BUFFER_SECTORS);
  sd_data.buffer_start = 0;
}

int32_t sdgif_read(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
  assert(iLen <= 2 * SD_SECTOR_SIZE);

  int32_t iBytesRead = MIN(iLen, FILE_LEN - pFile->iPos);

  uint32_t offset_into_buffer = pFile->iPos - (sd_data.buffer_start << 9);

  if (offset_into_buffer < 0 || offset_into_buffer + iBytesRead > SD_SECTOR_SIZE * SD_BUFFER_SECTORS) {
    uint32_t sector = pFile->iPos >> 9;
    if (sd_data.buffer_start != sector) {
      sd_readblocks_sync(sd_data.buffer, sector + SECTOR_START, SD_BUFFER_SECTORS);
      sd_data.buffer_start = sector;
    }

    offset_into_buffer = pFile->iPos - (sd_data.buffer_start << 9);
    assert(offset_into_buffer >= 0 && offset_into_buffer <= SD_SECTOR_SIZE * SD_BUFFER_SECTORS);
  }

  memcpy(pBuf, (uint8_t*)sd_data.buffer + offset_into_buffer, iBytesRead);

  pFile->iPos += iBytesRead;
  return iBytesRead;
}

int32_t sdgif_seek(GIFFILE *pFile, int32_t iPosition)
{
  if (iPosition < 0) iPosition = 0;
  else if (iPosition >= pFile->iSize) iPosition = pFile->iSize-1;
  
  pFile->iPos = iPosition;
  return pFile->iPos;
}

int main()
{
  stdio_init_all();

  sd_init_4pins();
  sd_set_clock_divider(5);

  st = st7789_init(pio0, 0);

  // Clear screen
  st7789_start_pixels(&st, DISPLAY_ROWS * DISPLAY_COLS);
  st7789_dma_repeat_pixel_one_channel(&st, 0x0000, DISPLAY_ROWS * DISPLAY_COLS);

  GIF_begin(&gif, LITTLE_ENDIAN_PIXELS, GIF_PALETTE_RGB565);
  while (1) {
    sdgif_init();
    gif.iError = GIF_SUCCESS;
    gif.pfnRead = sdgif_read;
    gif.pfnSeek = sdgif_seek;
    gif.pfnDraw = sdgif_draw;
    gif.pfnOpen = NULL;
    gif.pfnClose = NULL;
    gif.GIFFile.iSize = FILE_LEN;
    if (!GIFInit(&gif))
    {
      printf("Open failed\n");
    } 
    else
    {
      //gif.pfnRead = gifRead;
      frame.minx = MAX(0, (DISPLAY_COLS - gif.iCanvasWidth) >> 1);
      frame.maxx = MIN(DISPLAY_COLS - 1, frame.minx + gif.iCanvasWidth);
      frame.miny = MAX(0, (DISPLAY_ROWS - gif.iCanvasHeight) >> 1);
      frame.maxy = MIN(DISPLAY_ROWS - 1, frame.miny + gif.iCanvasHeight);

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
