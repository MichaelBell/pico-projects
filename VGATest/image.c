// Copyright (C) 2021 Michael Bell

#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/interp.h"

#include "vga.h"
#include "flash.h"

//#include "feeding_duck320.h"
//#include "perseverance.h"
#include "pcrane720.h"
//#include "feeding_duck720.h"

//#include "pcrane1078.h"

static uint32_t* data_pos;

#define IMAGE_ROWS 720

#if 0
#define DISPLAY_COLS 640
#define DISPLAY_ROWS 480
#else
#if 1
#define DISPLAY_COLS 1280
#define DISPLAY_ROWS 720
#else
#define DISPLAY_COLS 1920
#define DISPLAY_ROWS 1080
#endif
#endif

#define MAX_CMD_LINE_LEN (((DISPLAY_COLS + 5) / 6) + 1)

#define SYMBOLS_IN_TABLE 64
#define PIXEL_SYMBOLS_IDX 0
#define RLE_SYMBOLS_IDX 1

typedef struct {
  uint32_t buffer[2][MAX_CMD_LINE_LEN];
  uint32_t* ptr;
  uint32_t len;
  uint16_t symbol_table[2][SYMBOLS_IN_TABLE];
} channel_t;

channel_t channel[3];

#define red_chan channel[0]
#define green_chan channel[1]
#define blue_chan channel[2]

static uint16_t bufnum = 0;
static uint16_t display_row = 0;
static uint16_t image_row = 0;

static uint32_t zero = 0;

// Commands to core 0
#define CORE_CMD_INIT_FRAME      1

#define GRAYSCALE_BIT (1u << 31)

#define BLACK99 0b01000001111100000111110000011111  // 99 pixels of black
#define BLACK61 0b01000001111100000110000000000000  // 61 pixels of black

#define WHITE99 0x7fffffff  // 99 pixels of white
#define WHITE92 0x7ffffff8  // 92 pixels of white

const uint32_t white_line[] = {
  WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, 
  WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE92, 0
};

static uint32_t compressed_bits;
static int32_t compressed_bit_len;

// Read bits from flash ringbuffer.
inline static uint32_t __attribute__((always_inline)) get_bits(int32_t bit_len)
{
  interp1->add_raw[0] = bit_len;

  if (compressed_bit_len < bit_len)
  {
    int32_t need_bits = bit_len - compressed_bit_len;
    uint32_t result = compressed_bits << need_bits;
    compressed_bits = *(uint32_t*)interp1->peek[0];
    compressed_bit_len = 32 - need_bits;
    result |= compressed_bits >> compressed_bit_len;
    compressed_bits &= ((~0u) >> need_bits);

    return result;
  }
  else
  {
    compressed_bit_len -= bit_len;
    uint32_t result = compressed_bits >> compressed_bit_len;
    compressed_bits &= (1u << compressed_bit_len) - 1;
    return result;
  }
}

inline static uint32_t __attribute__((always_inline)) get_32_bits()
{
  interp1->add_raw[0] = 32;

  int32_t need_bits = 32 - compressed_bit_len;
  uint32_t result = compressed_bits << need_bits;
  compressed_bits = *(uint32_t*)interp1->peek[0];
  result |= compressed_bits >> compressed_bit_len;
  
  // Could maybe get rid of this and put the burden on the next reader
  // Would save cycles on back to back 32-bit reads.
  compressed_bits &= ((~0u) >> need_bits);

  return result;
}

static void read_compression_tables()
{
  compressed_bits = 0;
  compressed_bit_len = 0;

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      interp1->accum[0] = flash_get_data_in_ringbuffer_blocking((SYMBOLS_IN_TABLE * 10) / 32) << 5;

      for (int k = 0; k < SYMBOLS_IN_TABLE; ++k)
        channel[i].symbol_table[j][k] = get_bits(10);
      
      flash_release_ringbuffer();
    }
  }
}

static void read_lines(uint32_t offset)
{
  interp1->accum[0] = flash_get_data_in_ringbuffer_blocking(1) << 5;
  compressed_bits = 0;
  compressed_bit_len = 0;

  for (int channel_idx = 0; channel_idx < 3; ++channel_idx)
  {
    uint32_t len = get_bits(13);

    uint32_t cur_bit_idx = interp1->accum[0];

    // Fetch enough data.  This is a bit of a mess but the compiler should sort it out.
    uint32_t bits_left_in_cur_word = 32 - (cur_bit_idx & 0x1f);
    if (bits_left_in_cur_word == 32) bits_left_in_cur_word = 0;
    flash_get_data_in_ringbuffer_blocking((len - bits_left_in_cur_word + 31) / 32);
    
    // End of channel when accum[0] == len.
    len += cur_bit_idx;

    uint32_t* cmd_ptr = channel[channel_idx].ptr + offset;
    uint16_t* pixel_table = channel[channel_idx].symbol_table[PIXEL_SYMBOLS_IDX];
    uint16_t* rle_table = channel[channel_idx].symbol_table[RLE_SYMBOLS_IDX];

    uint32_t bits = get_32_bits();
    while (true)
    {
      if (bits & (1u << 30))
      {
        *cmd_ptr++ = bits;
        if (interp1->accum[0] >= len) break;
        bits = get_32_bits();
      }
      else
      {
        uint32_t cmd;
        interp0->accum[1] = bits;
        if (bits & (1u << 31))
        {
          cmd = 0xC0000000u;
          interp0->base[0] = (uintptr_t)pixel_table;
        }
        else
        {
          cmd = 0x40000000u;
          interp0->base[0] = (uintptr_t)rle_table;
        }

#if 0
        // The compiler makes a right mess of this!
        cmd |= *(uint16_t*)interp0->pop[0];
        cmd |= (uint32_t)(*(uint16_t*)interp0->pop[0]) << 10;
        cmd |= (uint32_t)(*(uint16_t*)interp0->pop[0]) << 20;
#else
        asm ("ldr r6, [%[itp0], #20]\n\t"
             "ldrh r1, [r6, #0]\n\t"
             "orrs %[cmd], r1\n\t"
             "ldr r6, [%[itp0], #20]\n\t"
             "ldrh r1, [r6, #0]\n\t"
             "lsls r1, #10\n\t"
             "orrs %[cmd], r1\n\t"
             "ldr r6, [%[itp0], #20]\n\t"
             "ldrh r1, [r6, #0]\n\t"
             "lsls r1, #20\n\t"
             "orrs %[cmd], r1" :
              [cmd] "+r" (cmd) :
              [itp0] "r" (interp0) :
              "r1", "r6");
#endif

        *cmd_ptr++ = cmd;

        // A compressed entry is not allowed to be the end of the line.
        bits = (bits << 20) | get_bits(20);
      }
    }
    assert(interp1->accum[0] == len);

    *cmd_ptr++ = 0;
    channel[channel_idx].len = cmd_ptr - channel[channel_idx].ptr;
    assert(channel[channel_idx].len < MAX_CMD_LINE_LEN);

    if (channel_idx != 2)
    {
      // Ensure we have enough bits to read the length.
      uint32_t bits_left_in_cur_word = 32 - (interp1->accum[0] & 0x1f);
      if (bits_left_in_cur_word == 32) bits_left_in_cur_word = 0;
      if (bits_left_in_cur_word < 13) flash_get_data_in_ringbuffer_blocking(1);
    }
  }

  flash_release_ringbuffer();
}

#ifdef IMAGE_IN_RAM
// This is the from RAM version
static void setup_next_line_ptr_and_len()
{
  if (display_row >= 120 && data_pos < image_dat + image_dat_len) {
    bufnum ^= 1;

    uint32_t lens = *data_pos++;
    if (lens & GRAYSCALE_BIT) {
      red_chan.len = lens & 0x3ff;
      red_chan.ptr = red_chan.buffer[bufnum];
      memcpy(red_chan.ptr + 2, data_pos, red_chan.len*sizeof(uint32_t));
      data_pos += red_chan.len;
      red_chan.ptr[red_chan.len + 2] = 0;
      red_chan.len += 3;

      green_chan.ptr = blue_chan.ptr = red_chan.ptr;
      green_chan.len = blue_chan.len = red_chan.len;
    }
    else
    {
      red_chan.len = (lens & 0x3ff);
      green_chan.len = ((lens >> 10) & 0x3ff);
      blue_chan.len = (lens >> 20);
      for (int i = 0; i < 3; ++i)
      {
        channel[i].ptr = channel[i].buffer[bufnum];
        memcpy(channel[i].ptr + 2, data_pos, channel[i].len*sizeof(uint32_t));
        data_pos += channel[i].len;
        channel[i].ptr[channel[i].len + 2] = 0;
        channel[i].len += 3;
      }
    }
    assert(data_pos <= image_dat + image_dat_len);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
  }
}
#else
// This streams from flash
static void __time_critical_func(setup_next_line_ptr_and_len)()
{
  static uint16_t scroll = 0;

  if (display_row == 0) scroll = (scroll + 1) & 0x3ff;

  // This is useful for forcing the display to adjust to the picture if you've
  // confused it.
  if (false && (display_row == 0 || display_row == DISPLAY_ROWS - 1))
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = count_of(white_line);
      channel[i].ptr = (uint32_t*)white_line;
    }
  }

  else if (image_row < IMAGE_ROWS) // && (display_row & 1) == 0) //display_row < DISPLAY_ROWS - 100)
  {
    bufnum ^= 1;
    ++image_row;

    {
      const uint32_t offset = 2;

      for (int i = 0; i < 3; ++i)
      {
        channel[i].ptr = channel[i].buffer[bufnum];
      }
      read_lines(offset);
    }
    assert(data_pos <= image_dat + image_dat_len);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
  }
}
#endif

#define vga_red_dma   5
#define vga_green_dma 6
#define vga_blue_dma  7
#define vga_dma_channel_mask (0b111 << 5)
static uint32_t complete_dma_channel_bits = 0;

static void __time_critical_func(transfer_next_line)()
{
  dma_channel_transfer_from_buffer_now(vga_red_dma, red_chan.ptr, red_chan.len);
  dma_channel_transfer_from_buffer_now(vga_green_dma, green_chan.ptr, green_chan.len);
  dma_channel_transfer_from_buffer_now(vga_blue_dma, blue_chan.ptr, blue_chan.len);
  if (++display_row < DISPLAY_ROWS) 
    setup_next_line_ptr_and_len();
}

void __time_critical_func(dma_complete_handler)() 
{
  uint32_t ints = dma_hw->ints0 & vga_dma_channel_mask;
  dma_hw->ints0 = ints;
  complete_dma_channel_bits |= ints;

  if (complete_dma_channel_bits == vga_dma_channel_mask && display_row < DISPLAY_ROWS)
  {
    complete_dma_channel_bits = 0;
    transfer_next_line();
  }
}

static void setup_dma_channels()
{
    // Setup DMAs
    dma_channel_config c = dma_channel_get_default_config(vga_red_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_red_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_red_dma,               // Channel to be configured
        &c,                        // The configuration we just created
        &vga_pio->txf[vga_red_sm], // The write address
        NULL,                      // The initial read address - set later
        0,                         // Number of transfers - set later
        false                      // Don't start yet
    );

    c = dma_channel_get_default_config(vga_green_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_green_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_green_dma,               // Channel to be configured
        &c,                          // The configuration we just created
        &vga_pio->txf[vga_green_sm], // The write address
        NULL,                        // The initial read address - set later
        0,                           // Number of transfers - set later
        false                        // Don't start yet
    );

    c = dma_channel_get_default_config(vga_blue_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_blue_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_blue_dma,               // Channel to be configured
        &c,                         // The configuration we just created
        &vga_pio->txf[vga_blue_sm], // The write address
        NULL,                       // The initial read address - set later
        0,                          // Number of transfers - set later
        false                       // Don't start yet
    );

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);

    dma_hw->inte0 = vga_dma_channel_mask;
}

void setup_interpolators()
{
  // interp0 is for decoding compressed data.
  // At start there are 3 6-bit indices in bits 30-12.  
  // The table holds 16-bit symbols.
  // Lane 1 accumulator is set with the original bits.
  // Lane 0 base is set with the table address.
  // Lane 1 shifts the bits right 6 bits every pop and stores back into acc 1.
  // Lane 0 reads the acc 1, shifts right 11 bits and masks.
  // Result of lane 0 is the memory location for the symbol.
  interp_config cfg = interp_default_config();
  interp_config_set_shift(&cfg, 11);
  interp_config_set_mask(&cfg, 1, 6);
  interp_config_set_cross_input(&cfg, true);
  interp_set_config(interp0, 0, &cfg);

  cfg = interp_default_config();
  interp_config_set_shift(&cfg, 6);
  interp_set_config(interp0, 1, &cfg);
  interp0->base[1] = 0;

  // interp1 is for reading bit data from the flash ringbuffer.
  // base0 is set to the ringbuffer pointer.  When reading bits,
  // put the flash ring buffer index << 5 into accum0, then before
  // reading each bit string add the number of bits directly to the accumulator.
  // If you need to read another word to complete the bit field
  // peek0 will give the address to read from.
  cfg = interp_default_config();
  interp_config_set_shift(&cfg, 3);
  interp_config_set_mask(&cfg, 2, FLASH_BUF_LOG_SIZE_BYTES - 1);
  interp_set_config(interp1, 0, &cfg);

  interp1->base[0] = (uintptr_t)flash_buffer;
}

void __time_critical_func(display_loop)() 
{
  setup_dma_channels();
  setup_interpolators();

#ifndef IMAGE_IN_RAM
  flash_set_stream(image_dat, image_dat_len, false);
#endif

#if 1
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      channel[i].buffer[j][0] = BLACK99;
      channel[i].buffer[j][1] = BLACK61;
    }
  }
#endif

  while(1) 
  {
      uint instr = multicore_fifo_pop_blocking();
      uint cmd = instr & 0xff;
      uint data = instr >> 8;
      switch (cmd)
      {
        case CORE_CMD_INIT_FRAME:
        {
          data_pos = image_dat;
          display_row = 0;
          image_row = 0;

          irq_set_enabled(DMA_IRQ_0, false);

          flash_reset_stream();
          read_compression_tables();
          setup_next_line_ptr_and_len();
          
          dma_hw->ints0 = vga_dma_channel_mask;
          complete_dma_channel_bits = 0;
          transfer_next_line();

          irq_set_enabled(DMA_IRQ_0, true);

          break;
        }
        default:
        {
          //!!
          __breakpoint();
        }
      }
  }
}

void __no_inline_not_in_flash_func(display_start_new_frame)() 
{
  multicore_fifo_push_blocking(CORE_CMD_INIT_FRAME);
}

void __no_inline_not_in_flash_func(display_end_frame)() 
{
  irq_set_enabled(DMA_IRQ_0, false);
  dma_channel_abort(vga_red_dma);
  dma_channel_abort(vga_green_dma);
  dma_channel_abort(vga_blue_dma);
  dma_hw->ints0 = vga_dma_channel_mask;
}