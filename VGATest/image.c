// Copyright (C) 2021 Michael Bell

#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/interp.h"
#include "pico/sd_card.h"

#include "vga.h"
#include "sdring.h"
#include "vgaaudio.h"

//#include "feeding_duck320.h"
//#include "perseverance.h"
//#include "pcrane720.h"
//#include "feeding_duck720.h"

//#include "pcrane1078.h"

static uint32_t* data_pos;

#define IMAGE_ROWS 675
#define IMAGE_PAD_TOP 20

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
static bool abort_frame = false;

static uint32_t zero = 0;

volatile uint32_t debug_flag = 0;

static volatile uint16_t core1_row_done = -1;

// Commands to core 0
#define CORE0_CMD_INIT_FRAME               1
#define CORE0_CMD_END_FRAME                2

// Commands to core 1
#define CORE1_CMD_DECOMPRESS_GREEN_CHANNEL 10
#define CORE1_CMD_TRANSFER_AUDIO           11
#define CORE1_CMD_NONE                     12

#define GRAYSCALE_BIT (1u << 31)

#define BLACK99 0b01000001111100000111110000011111  // 99 pixels of black
#define BLACK61 0b01000001111100000110000000000000  // 61 pixels of black

#define WHITE99 0x7fffffff  // 99 pixels of white
#define WHITE92 0x7ffffff8  // 92 pixels of white

const uint32_t white_line[] = {
  WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, 
  WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE99, WHITE92, 0
};

static uint32_t compressed_bits[2];
static int32_t compressed_bit_len[2];

// Read bits from flash ringbuffer.
inline static uint32_t __attribute__((always_inline)) get_bits(int32_t bit_len)
{
  assert(bit_len < 32);

  interp1->add_raw[0] = bit_len;

  if (compressed_bit_len[0] < bit_len)
  {
    int32_t need_bits = bit_len - compressed_bit_len[0];
    uint32_t result = compressed_bits[0] >> (32 - bit_len);
    compressed_bits[0] = *(uint32_t*)interp1->peek[0];
    compressed_bit_len[0] = 32 - need_bits;
    result |= compressed_bits[0] >> compressed_bit_len[0];
    compressed_bits[0] <<= need_bits;

    return result;
  }
  else
  {
    compressed_bit_len[0] -= bit_len;
    uint32_t result = compressed_bits[0] >> (32 - bit_len);
    compressed_bits[0] <<= bit_len;
    return result;
  }
}

inline static uint32_t __attribute__((always_inline)) get_32_bits()
{
  interp1->add_raw[0] = 32;

  if (compressed_bit_len[0] == 0) {
    return *(uint32_t*)interp1->peek[0];
  }

  int32_t need_bits = 32 - compressed_bit_len[0];
  uint32_t result = compressed_bits[0];
  compressed_bits[0] = *(uint32_t*)interp1->peek[0];
  result |= compressed_bits[0] >> compressed_bit_len[0];
  compressed_bits[0] <<= need_bits;
  
  return result;
}

static void read_compression_tables()
{
  compressed_bits[0] = 0;
  compressed_bit_len[0] = 0;

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      interp1->accum[0] = sdring_get_data_in_ringbuffer_blocking(0, (SYMBOLS_IN_TABLE * 10) / 32) << 5;

      for (int k = 0; k < SYMBOLS_IN_TABLE; ++k)
        channel[i].symbol_table[j][k] = get_bits(10);
      
      sdring_release_ringbuffer(0);
    }
  }
}

inline static void __attribute__((always_inline)) run_inner_loop(int channel_idx, int offset, int bits_idx)
{
  uint32_t* cmd_ptr = channel[channel_idx].ptr + offset;
  uint16_t* pixel_table = channel[channel_idx].symbol_table[PIXEL_SYMBOLS_IDX];
  uint16_t* rle_table = channel[channel_idx].symbol_table[RLE_SYMBOLS_IDX];

  uint32_t bits, cmd;
  uint32_t cb = compressed_bits[bits_idx], cbl = compressed_bit_len[bits_idx];

    asm ( "movs r2, #32\n\t"            // interp1->add_raw[0] = 32;
          "str r2, [%[itp0], #116]\n\t"
          "cmp %[cbl], #0\n\t"
          "beq.n 7f\n\t"
          "subs r2, r2, %[cbl]\n\t"     // need_bits = 32 - compressed_bit_len;
          "mov r1, %[cb]\n\t"
          "ldr r3, [%[itp0], #96]\n\t"  // *(uint32_t*)interp1->peek[0];
          "ldr r3, [r3, #0]\n\t"
          "movs %[bits], r3\n\t"        // compressed_bits >> compressed_bit_len
          "lsrs %[bits], %[cbl]\n\t"         
          "orrs %[bits], r1\n\t"        // bits |= compressed_bits >> compressed_bit_len;
          "lsls r3, r2\n\t"             // compressed_bits << need_bits

          // Begin main loop
      "1:  lsls r1, %[bits], #1\n\t"    // if (!(bits & (1u << 30)))
          "cmp r1, #0\n\t"
          "blt.n 2f\n\t"

          // Decompress case
          "str %[bits], [%[itp0], #4]\n\t"  // interp0->accum[1] = bits;
          "cmp %[bits], #0\n\t"         // if (!(bits & (1u << 31)))
          "blt.n 3f\n\t"

          // RLE case
          "movs %[cmd], #4\n\t"
          "ldr r1, %[rle_table]\n\t"
      "5:  str r1, [%[itp0], #8]\n\t"  // interp->base[0] = table
          "lsls %[cmd], #28\n\t"
          "ldr r1, [%[itp0], #20]\n\t"   // Build command from table
          "ldrh r1, [r1, #0]\n\t"
          "orrs %[cmd], r1\n\t"
          "ldr r1, [%[itp0], #20]\n\t"
          "ldrh r1, [r1, #0]\n\t"
          "lsls r1, #10\n\t"
          "orrs %[cmd], r1\n\t"
          "ldr r1, [%[itp0], #20]\n\t"
          "ldrh r1, [r1, #0]\n\t"
          "lsls r1, #20\n\t"
          "orrs %[cmd], r1\n\t"
          "stm %[cmdp]!, {%[cmd]}\n\t"

          // Get 20 bits
          "movs r2, #20\n\t"
          "str r2, [%[itp0], #116]\n\t"
          "lsrs r1, r3, #12\n\t"
          "lsls %[bits], #20\n\t"   // bits = (bits << 20) | compressed_bits
          "orrs %[bits], r1\n\t"
          "subs %[cbl], %[cbl], r2\n\t"
          "blt.n 4f\n\t"

          "lsls r3, #20\n\t"
          "b.n 1b\n\t"

      "7:  ldr r3, [%[itp0], #96]\n\t"
          "ldr %[bits], [r3, #0]\n\t"
          "movs r3, #0\n\t"
          "b.n 1b\n\t"

          // Normal case
      "2:  stm %[cmdp]!, {%[bits]}\n\t"   // *cmd_ptr++ = bits;
          "ldr r1, [%[itp0], #100]\n\t"  // interp1->peek[1]
          "cmp r1, #0\n\t"
          "bge.n 6f\n\t"                // >= 0: done
          "movs r1, #32\n\t"
          "str r1, [%[itp0], #116]\n\t"
          "cmp %[cbl], #0\n\t"
          "beq.n 7b\n\t"
          "mov %[bits], r3\n\t"
          "ldr r3, [%[itp0], #96]\n\t" // *(uint32_t*)interp1->peek[0];
          "ldr r3, [r3, #0]\n\t"
          "movs r2, r3\n\t"
          "lsrs r2, %[cbl]\n\t"
          "orrs %[bits], r2\n\t"
          "subs r1, r1, %[cbl]\n\t"
          "lsls r3, r1\n\t"
          "b.n 1b\n\t"
          
          // Pixel case
      "3:  movs %[cmd], #12\n\t"
          "ldr r1, %[pixel_table]\n\t"
          "b.n 5b\n\t"

            // Get 20 bits, not enough bits
      "4:  ldr r3, [%[itp0], #96]\n\t" // *(uint32_t*)interp1->peek[0];
          "ldr r3, [r3, #0]\n\t"
          "adds %[cbl], #32\n\t"       // compressed_bit_len = 32 - need_bits;
          "movs r2, r3\n\t"
          "lsrs r2, %[cbl]\n\t"
          "orrs %[bits], r2\n\t"
          "movs r1, #32\n\t"
          "subs r1, r1, %[cbl]\n\t"
          "lsls r3, r1\n\t"
          "b.n 1b\n\t"

          // Exit
      "6:  mov %[cb], r3\n\t" 
      : [cmdp] "+l" (cmd_ptr),
        [cb] "+r" (cb),
        [cbl] "+l" (cbl),
        [bits] "=&l" (bits),
        [cmd] "=&l" (cmd)
      : [itp0] "l" (interp0),
        [rle_table] "m" (rle_table),
        [pixel_table] "m" (pixel_table)
      : "r1", "r2", "r3", "cc" );

    assert(interp1->peek[1] == 0);

  *cmd_ptr++ = 0;
  channel[channel_idx].len = cmd_ptr - channel[channel_idx].ptr;
  assert(channel[channel_idx].len < MAX_CMD_LINE_LEN);

  compressed_bit_len[bits_idx] = cbl; 
  compressed_bits[bits_idx] = cb;
}

static void read_lines(uint32_t offset)
{
  interp1->accum[0] = sdring_get_data_in_ringbuffer_blocking(0, 1) << 5;
  compressed_bits[0] = 0;
  compressed_bit_len[0] = 0;

  for (int channel_idx = 0; channel_idx < 3; ++channel_idx)
  {
    if (abort_frame)
    {
      if (channel_idx == 0)
      {
        core1_row_done = display_row;
      }
      break;
    }

    uint32_t len = get_bits(13);
    //__breakpoint();

    uint32_t cur_bit_idx = interp1->accum[0];

    // End of channel when accum[0] == len + cur_bit_idx / peek[1] == 0.
    interp1->base[1] = -(cur_bit_idx + len);

    // Fetch enough data.
    sdring_get_data_in_ringbuffer_blocking(0, (len - compressed_bit_len[0] + 31) / 32);
    
#define GREEN_ON_CORE1
#ifdef GREEN_ON_CORE1
    // Throw green channel decompress to core 1
    uint32_t blue_bit_offset;
    if (channel_idx == 0)
    {
      // Ensure we have enough bits to read the length.
      uint32_t green_bit_offset = (cur_bit_idx + len) & ((1 << (SDRING_BUF_LOG_SIZE_BYTES + 3)) - 1);
      int32_t bits_left_in_cur_word = 32 - ((green_bit_offset) & 0x1f);
      uint32_t bits_from_cur_word = 0;
      bits_from_cur_word = sdring_buffer_0[green_bit_offset >> 5] << ((green_bit_offset) & 0x1f);

      uint32_t green_len = bits_from_cur_word >> (32 - 13);
      bits_from_cur_word <<= 13;

      if (bits_left_in_cur_word < 13) {
        sdring_get_data_in_ringbuffer_blocking(0, 1);
        uint32_t next_idx = ((green_bit_offset >> 5) + 1) & SDRING_BUF_IDX_MASK;
        bits_from_cur_word = sdring_buffer_0[next_idx];
        bits_left_in_cur_word = (32 - (13 - bits_left_in_cur_word));
        green_len |= bits_from_cur_word >> bits_left_in_cur_word;
        bits_from_cur_word <<= (32 - bits_left_in_cur_word);
      }
      else
      {
        bits_left_in_cur_word -= 13;
      }

      sdring_get_data_in_ringbuffer_blocking(0, (green_len - bits_left_in_cur_word + 31) / 32);

      green_bit_offset = (green_bit_offset + 13);
      blue_bit_offset = green_bit_offset + green_len;
      sio_hw->fifo_wr = CORE1_CMD_DECOMPRESS_GREEN_CHANNEL;
      sio_hw->fifo_wr = green_bit_offset;
      sio_hw->fifo_wr = blue_bit_offset;
      sio_hw->fifo_wr = bits_from_cur_word;
      sio_hw->fifo_wr = bits_left_in_cur_word;
      __sev();
    }

#endif

#if 1
    run_inner_loop(channel_idx, offset, 0);
#else
    uint32_t bits = get_32_bits();
    while (true)
    {
      if (bits & (1u << 30))
      {
        *cmd_ptr++ = bits;
        if ((int32_t)interp1->peek[1] >= 0) break;
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

        // The compiler makes a right mess of this!
        cmd |= *(uint16_t*)interp0->pop[0];
        cmd |= (uint32_t)(*(uint16_t*)interp0->pop[0]) << 10;
        cmd |= (uint32_t)(*(uint16_t*)interp0->pop[0]) << 20;

        *cmd_ptr++ = cmd;

        // A compressed entry is not allowed to be the end of the line.
        bits = (bits << 20) | get_bits(20);
      }
    }
#endif

#ifdef GREEN_ON_CORE1
    // Fix up compressed_bits/len
    if (channel_idx == 0)
    {
      interp1->accum[0] = blue_bit_offset;
    }
#endif

    if (channel_idx != 2)
    {
      // Ensure we have enough bits to read the length.
      uint32_t bits_left_in_cur_word = 32 - (interp1->accum[0] & 0x1f);
      if (bits_left_in_cur_word == 32) bits_left_in_cur_word = 0;
      if (bits_left_in_cur_word < 13) sdring_get_data_in_ringbuffer_blocking(0, 1);
    }

#ifdef GREEN_ON_CORE1
    if (channel_idx == 0)
    {
      compressed_bits[0] = *(uint32_t*)interp1->peek[0];
      compressed_bit_len[0] = blue_bit_offset & 0x1f;
      compressed_bits[0] <<= compressed_bit_len[0];
      if (compressed_bit_len[0] != 0) compressed_bit_len[0] = 32 - compressed_bit_len[0];

      // Skip green on this core
      ++channel_idx;
    }
#endif
  }

#ifdef GREEN_ON_CORE1
  while (display_row != core1_row_done && !abort_frame);
#endif

  sdring_release_ringbuffer(0);
}

// This streams from flash
static void __time_critical_func(setup_next_line_ptr_and_len)()
{
  static uint16_t scroll = 0;

  if (display_row == 0) scroll = (scroll + 1) & 0x3ff;

  // This is useful for forcing the display to adjust to the picture if you've
  // confused it.
  if (debug_flag)//(false && (display_row == 0 || display_row >= DISPLAY_ROWS - 20))
  {
    debug_flag = 0;
    if ((display_row & 1) == 0)
    {
      for (int i = 0; i < 3; ++i)
      {
        channel[i].len = count_of(white_line);
        channel[i].ptr = (uint32_t*)white_line;
      }
    }
    else
    {
      for (int i = 0; i < 3; ++i)
      {
        channel[i].len = 1;
        channel[i].ptr = &zero;
      }
    }
    multicore_fifo_push_blocking(CORE1_CMD_TRANSFER_AUDIO);
  }
  else if (image_row < IMAGE_ROWS && display_row >= IMAGE_PAD_TOP) // && display_row < DISPLAY_ROWS - 20)
  {
    bufnum ^= 1;
    ++image_row;

    {
      const uint32_t offset = 0;

      for (int i = 0; i < 3; ++i)
      {
        channel[i].ptr = channel[i].buffer[bufnum];
      }
      read_lines(offset);
    }
    //assert(data_pos <= image_dat + image_dat_len);
  }
  else
  {
    if (image_row == IMAGE_ROWS)
      multicore_fifo_push_blocking(CORE1_CMD_TRANSFER_AUDIO);
    else
      multicore_fifo_push_blocking(CORE1_CMD_NONE);
    for (int i = 0; i < 3; ++i)
    {
      channel[i].len = 1;
      channel[i].ptr = &zero;
    }
    while (display_row != core1_row_done && !abort_frame);
  }
}

#define vga_red_dma   5
#define vga_green_dma 6
#define vga_blue_dma  7
#define vga_dma_channel_mask (0b111 << 5)
static uint32_t complete_dma_channel_bits = 0;

static void __time_critical_func(transfer_next_line)()
{
  if (abort_frame) return;
  dma_channel_transfer_from_buffer_now(vga_red_dma, red_chan.ptr, red_chan.len);
  dma_channel_transfer_from_buffer_now(vga_green_dma, green_chan.ptr, green_chan.len);
  dma_channel_transfer_from_buffer_now(vga_blue_dma, blue_chan.ptr, blue_chan.len);
  if (display_row + 1 < DISPLAY_ROWS) 
  {
    ++display_row;
    setup_next_line_ptr_and_len();
  }
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
  interp_config_set_mask(&cfg, 2, SDRING_BUF_LOG_SIZE_BYTES - 1);
  interp_set_config(interp1, 0, &cfg);
  interp1->base[0] = (uintptr_t)sdring_buffer_0;

  // interp1 lane 1 indicates the number of bits left to read
  // base1 is set to minus the number of bits that will have been read at
  // the end of the line.  The line is therefore finished when result1 is zero.
  // This saves a register.
  cfg = interp_default_config();
  interp_config_set_cross_input(&cfg, true);
  interp_set_config(interp1, 1, &cfg);  
}

#include "image_lens.h"

static uint64_t total_time = 0;

void __time_critical_func(display_loop)() 
{
  uint32_t frame_number = 0;

  sdring_init(true);
  setup_dma_channels();
  setup_interpolators();

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
        case CORE0_CMD_INIT_FRAME:
        {
          while (core1_row_done != display_row);

          //data_pos = image_dat;
          display_row = 0;
          image_row = 0;
          core1_row_done = -1;
          abort_frame = false;

          irq_set_enabled(DMA_IRQ_0, false);

          if (frame_number++ == 0) {
            // Reset audio, don't display this frame to get audio buffers full
            audio_reset();
            sdring_set_stream(0, 0, 0, false, false);
            core1_row_done = 0;
            abort_frame = true;

            // Prime the audio.
            //absolute_time_t start_time = get_absolute_time();
            while (audio_buffers_transferred < 32)
              audio_transfer(false);
            //total_time = absolute_time_diff_us(start_time, get_absolute_time());
            //__breakpoint();
          }
          else
          {
            uint32_t image_idx = (frame_number - 1) / FRAMES_BEFORE_CHANGE;
            if (frame_number == NUM_IMAGES * FRAMES_BEFORE_CHANGE) frame_number = 0;

            sdring_set_stream(0, FIRST_IMAGE_OFFSET + (IMAGE_OFFSET_GAP * image_idx), image_lengths[image_idx], true, false);
            read_compression_tables();
            setup_next_line_ptr_and_len();
            
            dma_hw->ints0 = vga_dma_channel_mask;
            complete_dma_channel_bits = 0;
            transfer_next_line();

            irq_set_enabled(DMA_IRQ_0, true);
          }

          break;
        }
        case CORE0_CMD_END_FRAME:
        {
          //audio_transfer();
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

void __time_critical_func(display_core1_loop)()
{
  setup_interpolators();
  core1_row_done = 0;

  while(1) 
  {
      uint cmd = multicore_fifo_pop_blocking();
      switch (cmd)
      {
        case CORE1_CMD_DECOMPRESS_GREEN_CHANNEL:
          interp1->accum[0] = multicore_fifo_pop_blocking();
          interp1->base[1] = -multicore_fifo_pop_blocking();
          {
            compressed_bits[1] = multicore_fifo_pop_blocking();
            compressed_bit_len[1] = multicore_fifo_pop_blocking();
            run_inner_loop(1, 0, 1);
          }

          assert(!multicore_fifo_rvalid());
          core1_row_done = display_row;

          if (sdring_eof(0) && !abort_frame)
            audio_transfer(true);
          else
            sdring_words_available(0);
          break;

        case CORE1_CMD_TRANSFER_AUDIO:
          audio_transfer();
          assert(!multicore_fifo_rvalid());
          core1_row_done = display_row;
          break;

        case CORE1_CMD_NONE:
          assert(!multicore_fifo_rvalid());
          core1_row_done = display_row;
          break;

        default:
          __breakpoint();
      }
  }
}

void __no_inline_not_in_flash_func(display_start_new_frame)() 
{
  multicore_fifo_push_blocking(CORE0_CMD_INIT_FRAME);
}

void __no_inline_not_in_flash_func(display_end_frame)() 
{
  abort_frame = true;
  irq_set_enabled(DMA_IRQ_0, false);
  dma_channel_abort(vga_red_dma);
  dma_channel_abort(vga_green_dma);
  dma_channel_abort(vga_blue_dma);
  dma_hw->ints0 = vga_dma_channel_mask;
  //multicore_fifo_push_blocking(CORE0_CMD_END_FRAME);
}