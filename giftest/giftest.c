#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/ssi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/structs/ssi.h"

#include "st7789_lcd.h"
#include "AnimatedGIF.h"
//#include "duck.h"

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

static uint8_t* gif_addr = (uint8_t*)0x10010000;
static uint32_t gif_len = 5702746;

#define PIXEL_DATA_LEN (32 + ((DISPLAY_WIDTH + 1)>>1))
uint32_t pixel_data[2][PIXEL_DATA_LEN];

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
      uint32_t* pixelEndPtr = pixel_data[chan_idx] + PIXEL_DATA_LEN - 6;

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
          if ((x - startx) & 1) {
            *pixelPtr++ |= pixel;
            if (pixelPtr >= pixelEndPtr) {
              ++x;
              break;
            }
          }
          else *pixelPtr = pixel << 16;
        }
        if ((x - startx) & 1) pixelPtr++;

        st7789_add_pixels_at_cmd(startPixelPtr, pDraw->iX + minx + startx, y, x - startx);

        if (pixelPtr >= pixelEndPtr && x < width)
        {
          st7789_dma_buffer(chan, chan_idx, pixel_data[chan_idx], pixelPtr - pixel_data[chan_idx]);
          chan_idx ^= 1;
          dma_channel_wait_for_finish_blocking(chan[chan_idx]);
          pixelPtr = pixel_data[chan_idx];
          pixelEndPtr = pixel_data[chan_idx] + PIXEL_DATA_LEN - 6;
        }
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

#define FLASH_BUFFER_LEN 268
uint flash_dma_chan;
uint32_t flash_read_buffer[FLASH_BUFFER_LEN / 4];
uintptr_t flash_last_read_addr = 0;

void flash_dma_init()
{
  flash_dma_chan = dma_claim_unused_channel(true);

  dma_channel_config cfg = dma_channel_get_default_config(flash_dma_chan);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_XIP_STREAM);
  dma_channel_configure(
            flash_dma_chan,
            &cfg,
            (void *) flash_read_buffer,   // Write addr
            (const void *) XIP_AUX_BASE,  // Read addr
            FLASH_BUFFER_LEN / 4,         // Transfer count
            false                        
    );
}

void flash_start_dma(uint8_t* addr) 
{
  while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
    (void) xip_ctrl_hw->stream_fifo;
  xip_ctrl_hw->stream_addr = (uintptr_t)addr & (~3u);
  xip_ctrl_hw->stream_ctr = FLASH_BUFFER_LEN / 4;

  dma_channel_transfer_to_buffer_now(flash_dma_chan, (void*)flash_read_buffer, FLASH_BUFFER_LEN / 4);
  flash_last_read_addr = (uintptr_t)addr & (~3u);
}

void flash_stop_dma()
{
  if (dma_channel_is_busy(flash_dma_chan)) {
    xip_ctrl_hw->stream_ctr = 0;
    //while (ssi_hw->sr & SSI_SR_BUSY_BITS);
    (void)*(io_rw_32*)XIP_NOCACHE_NOALLOC_BASE;
    dma_channel_abort(flash_dma_chan);
    //sleep_us(1);  // My transfer becomes unreliable if I remove this line
  }
}

int32_t gifRead(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    int32_t iBytesRead;

    iBytesRead = iLen;
    if ((pFile->iSize - pFile->iPos) < iLen)
       iBytesRead = pFile->iSize - pFile->iPos;
    if (iBytesRead <= 0)
       return 0;

    int32_t buffer_offset = (pFile->pData + pFile->iPos) - (uint8_t*)flash_last_read_addr;
    if (buffer_offset >= 0 && 
        buffer_offset + iBytesRead <= FLASH_BUFFER_LEN - 4)
    {
      uint32_t read_end_addr = ((uintptr_t)flash_read_buffer + buffer_offset + iBytesRead - 1) | 3;
      while (dma_channel_is_busy(flash_dma_chan) && dma_hw->ch[flash_dma_chan].write_addr < read_end_addr);

      asm volatile ("" ::: "memory");
      memcpy(pBuf, (uint8_t*)flash_read_buffer + buffer_offset, iBytesRead);
      pFile->iPos += iBytesRead;
      if (buffer_offset + iBytesRead > FLASH_BUFFER_LEN - 259) {
        flash_stop_dma();
        flash_start_dma(&pFile->pData[pFile->iPos]);
      }
    }
    else
    {
      flash_stop_dma();
      memcpy(pBuf, &pFile->pData[pFile->iPos], iBytesRead);
      pFile->iPos += iBytesRead;

      flash_start_dma(&pFile->pData[pFile->iPos]);
    }

    return iBytesRead;
}

int main()
{
  stdio_init_all();

  st7789_init(pio, sm);
  st7789_create_dma_channels(pio, sm, chan);

  st7789_start_pixels(pio, sm, DISPLAY_WIDTH * DISPLAY_HEIGHT);
  st7789_dma_repeat_pixel_one_channel(chan[0], 0, DISPLAY_WIDTH * DISPLAY_HEIGHT);

  flash_dma_init();

  GIF_begin(&gif, LITTLE_ENDIAN_PIXELS, GIF_PALETTE_RGB565);
  while (1) {
    flash_stop_dma();
    flash_start_dma(gif_addr);
    if (!GIF_openRAM(&gif, (uint8_t *)gif_addr, gif_len, draw)) {
      printf("Open failed\n");
    } 
    else
    {
      //gif.pfnRead = gifRead;
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
