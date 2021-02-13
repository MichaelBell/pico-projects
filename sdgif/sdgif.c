#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sd_card.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "AnimatedGIF.h"
#include "st7789_lcd.h"

GIFIMAGE gif;
ST7789 st;

int main()
{
    stdio_init_all();

    st = st7789_init(pio0, 0);

    st7789_start_pixels(&st, 240*240);
    st7789_dma_repeat_pixel_one_channel(&st, 0xf000, 240*120);
    st7789_dma_repeat_pixel_one_channel(&st, 0x000f, 240*120);

    puts("Hello, world!");

    return 0;
}
