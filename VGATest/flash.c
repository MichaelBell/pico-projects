// Copyright (C) 2021 Michael Bell
// You may use this under the terms of the BSD 3 clause license.

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/irq.h"

#define FLASH_BUF_LEN_WORDS (128 * 16)
#define FLASH_BUF_IDX_MASK  (FLASH_BUF_LEN_WORDS - 1)
static uint32_t flash_buffer[FLASH_BUF_LEN_WORDS];

// Location in buffer that was returned to the user on last read
// We should write past here.
static volatile uint32_t* flash_prev_buffer_ptr;

// Location in buffer that will be returned to the user on next read
static volatile uint32_t* flash_buffer_ptr;

// Whether we read to the end of the stream
static volatile bool flash_stream_at_end;

// Whether we are stopping streaming
static volatile bool flash_stream_stop;

static volatile bool flash_stream_starting;

// Next address in flash to stream from
#define flash_cur_stream_addr (uint32_t*)(xip_ctrl_hw->stream_addr | 0x10000000)

// Source data in flash
static uint32_t* flash_source_data_ptr;
static uint32_t flash_source_data_len;

// Note this is in the range of channels that the SD card uses
// Currently assuming we'll use either SD or flash.
#define flash_dma_chan 11

// ISR for when the DMA from the flash finishes
// in order to schedule the next transfer
// Can also be called manually to restart after
// the buffer got full
static void __no_inline_not_in_flash_func(flash_transfer)()
{
  uint32_t next_write_idx = (uint32_t*)dma_hw->ch[flash_dma_chan].write_addr - flash_buffer;
  next_write_idx &= FLASH_BUF_IDX_MASK;
  if (flash_buffer + next_write_idx == flash_prev_buffer_ptr)
  {
    // Fully caught up, don't start the next transfer yet.
    return;
  }

  if (flash_cur_stream_addr >= flash_source_data_ptr + flash_source_data_len)
  {
    // Finished stream.
    flash_stream_at_end = true;
    return;
  }

  uint32_t* next_write_addr = (flash_buffer + next_write_idx);
  if (next_write_addr < flash_prev_buffer_ptr)
  {
    uint32_t words_to_read = flash_prev_buffer_ptr - next_write_addr;
    if (words_to_read > 0)
    {
      xip_ctrl_hw->stream_ctr = words_to_read;
      dma_channel_transfer_to_buffer_now(flash_dma_chan, next_write_addr, words_to_read);
    }
  }
  else
  {
    uint32_t words_to_read = FLASH_BUF_LEN_WORDS - next_write_idx;
    xip_ctrl_hw->stream_ctr = words_to_read;
    dma_channel_transfer_to_buffer_now(flash_dma_chan, next_write_addr, words_to_read);
  }
}

void __no_inline_not_in_flash_func(flash_transfer_isr)()
{
  if (dma_hw->ints1 & (1u << flash_dma_chan))
  {
    flash_stream_starting = true;
    dma_hw->ints1 = 1u << flash_dma_chan;
    if (!flash_stream_stop && !dma_channel_is_busy(flash_dma_chan))
      flash_transfer();
    flash_stream_starting = false;
  }
}

void __time_critical_func(flash_reset_stream)()
{
  // Stop any running transfers
  if (dma_channel_is_busy(flash_dma_chan)) {
    flash_stream_stop = true;
    xip_ctrl_hw->stream_ctr = 0;
    dma_channel_abort(flash_dma_chan);

    // Required to ensure the next flash transfer runs correctly.
    (void)*(io_rw_32*)XIP_NOCACHE_NOALLOC_BASE;
  }

  // Clear the XIP FIFO
  while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
    (void) xip_ctrl_hw->stream_fifo;

  // Reset read pointers to beginning
  flash_buffer_ptr = flash_prev_buffer_ptr = flash_buffer;

  // Reset state tracking
  flash_stream_at_end = false;
  flash_stream_starting = false;

  // Setup the first transfer
  xip_ctrl_hw->stream_addr = (uintptr_t)flash_source_data_ptr;
  xip_ctrl_hw->stream_ctr = FLASH_BUF_LEN_WORDS;

  flash_stream_stop = false;
  dma_channel_transfer_to_buffer_now(flash_dma_chan, flash_buffer, FLASH_BUF_LEN_WORDS);
}

void flash_set_stream(uint32_t* data, uint32_t len)
{
  // Remember where we are transferring from
  flash_source_data_ptr = data;
  flash_source_data_len = len;

  flash_reset_stream();
}

void flash_init()
{
  // Setup the DMA channel
  dma_channel_config c = dma_channel_get_default_config(flash_dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, DREQ_XIP_STREAM);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);

  dma_channel_configure(
      flash_dma_chan,            // Channel to be configured
      &c,                        // The configuration we just created
      flash_buffer,              // The write address
      (const void*)XIP_AUX_BASE, // The read address
      FLASH_BUF_LEN_WORDS,       // Number of transfers - set later
      false                      // Don't start yet
  );

  // Currently not working.
  irq_set_exclusive_handler(DMA_IRQ_1, flash_transfer_isr);
  dma_hw->inte1 = 1u << flash_dma_chan;
  irq_set_enabled(DMA_IRQ_1, true);
}

// Request data from flash.  Returns amount of data that was
// available - may be less than amount requested, and sets ptr_out 
// to point to it.  The data returned will be discarded on next read.
uint32_t __no_inline_not_in_flash_func(flash_get_data)(uint32_t len_req, uint32_t** ptr_out)
{
  assert(len_req <= FLASH_BUF_LEN_WORDS / 2);
  assert(len_req > 0);

  // Can now write over previously returned buffer.
  uint32_t* write_ptr = (uint32_t*)dma_hw->ch[flash_dma_chan].write_addr;
  flash_prev_buffer_ptr = flash_buffer_ptr;
  if (!dma_channel_is_busy(flash_dma_chan) && 
      (dma_hw->ints1 & (1u << flash_dma_chan)) == 0 &&
      !flash_stream_starting)
  {
    flash_transfer();
  }

  uint32_t words_available;
  if (write_ptr < flash_buffer_ptr) {
    // Write pointer behind read pointer means we have wrapped.  Can read until end of buffer
    words_available = FLASH_BUF_LEN_WORDS - (flash_buffer_ptr - flash_buffer);
  }
  else
  {
    words_available = write_ptr - flash_buffer_ptr;
  }

  if (words_available < len_req) len_req = words_available;

  *ptr_out = (uint32_t*)flash_buffer_ptr;
  flash_buffer_ptr += len_req;
  if (flash_buffer_ptr == flash_buffer + FLASH_BUF_LEN_WORDS) 
  {
    flash_buffer_ptr = flash_buffer;
  }

  return len_req;
}

void __not_in_flash_func(flash_copy_data_blocking)(uint32_t* dst_ptr, uint32_t len_in_words)
{
  if (len_in_words > 108) __breakpoint();

  uint32_t words_to_read = len_in_words;
  uint32_t* write_ptr = dst_ptr;
  while (words_to_read > 0) {
    uint32_t* data_ptr;
    uint32_t len_req = MIN(words_to_read, FLASH_BUF_LEN_WORDS / 4);
    uint32_t words_read = flash_get_data(len_req, &data_ptr);
    memcpy(write_ptr, data_ptr, words_read * sizeof(uint32_t));
    write_ptr += words_read;
    words_to_read -= words_read;
    //if (words_to_read != 0 && flash_buffer_ptr != flash_buffer) __breakpoint();
  }
}