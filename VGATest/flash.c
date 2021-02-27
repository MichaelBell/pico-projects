// Copyright (C) 2021 Michael Bell
// You may use this under the terms of the BSD 3 clause license.

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/irq.h"
#include "hardware/structs/ssi.h"
#include "hardware/regs/ssi.h"
#include "pico/sync.h"

#include "flash.h"

#if defined(PICO_COPY_TO_RAM) && PICO_COPY_TO_RAM
#define USE_SSI_DMA
#endif

#ifdef USE_SSI_DMA
#define FLASH_MIN_TRANSFER 16
#else
#define FLASH_MIN_TRANSFER 4
#endif

uint32_t flash_buffer[FLASH_BUF_LEN_WORDS] __attribute__((aligned(1 << FLASH_BUF_LOG_SIZE_BYTES)));

// Location in buffer that was returned to the user on last read
// We should write past here.
static volatile uint32_t* flash_prev_buffer_ptr;

// Location in buffer that will be returned to the user on next read
static volatile uint32_t* flash_buffer_ptr;

// Words that have been requested
static uint32_t flash_total_words_requested;

// End of current transfer
static uint32_t* flash_target_write_addr;

// Source data in flash
static uint32_t* flash_source_data_ptr;
static uint32_t flash_source_data_len;

// Start streaming more bytes from flash if there is space in the buffer
static void __no_inline_not_in_flash_func(flash_transfer)()
{
  uint32_t next_write_idx = (uint32_t*)dma_hw->ch[flash_dma_chan].write_addr - flash_buffer;
  next_write_idx &= FLASH_BUF_IDX_MASK;
  if (flash_buffer + next_write_idx == flash_prev_buffer_ptr)
  {
    // Fully caught up, don't start the next transfer yet.
    return;
  }

  if (flash_total_words_requested >= flash_source_data_len)
  {
    // Finished stream.
    return;
  }

  uint32_t* next_write_addr = (flash_buffer + next_write_idx);
  uint32_t words_to_read;
  if (next_write_addr < flash_prev_buffer_ptr)
  {
    words_to_read = flash_prev_buffer_ptr - next_write_addr;
  }
  else
  {
    words_to_read = FLASH_BUF_LEN_WORDS - next_write_idx + (flash_prev_buffer_ptr - flash_buffer);
  }

  if (words_to_read > flash_source_data_len - flash_total_words_requested)
    words_to_read = flash_source_data_len - flash_total_words_requested;

  // Don't make very small transfers.  Don't make a transfer that would leave
  // a very small transfer to the end.
  if (words_to_read < FLASH_MIN_TRANSFER || 
      ((flash_total_words_requested + words_to_read != flash_source_data_len) &&
       (flash_total_words_requested + words_to_read + FLASH_MIN_TRANSFER > flash_source_data_len)))
    return;

#ifdef USE_SSI_DMA
  ssi_hw->ssienr = 0;
  ssi_hw->ctrlr1 = words_to_read - 1;
  ssi_hw->dmacr = SSI_DMACR_BITS;
  ssi_hw->ssienr = 1;
#endif

  dma_channel_transfer_to_buffer_now(flash_dma_chan, next_write_addr, words_to_read);
#ifdef USE_SSI_DMA
  ssi_hw->dr0 = ((uintptr_t)(flash_source_data_ptr + flash_total_words_requested) << 8) | 0xa0;
#endif

  flash_total_words_requested += words_to_read;
  flash_target_write_addr = flash_buffer + ((next_write_idx + words_to_read) & FLASH_BUF_IDX_MASK);
}

void __time_critical_func(flash_reset_stream)()
{
  // Stop any running transfers
  if (dma_channel_is_busy(flash_dma_chan)) {
#ifdef USE_SSI_DMA
    // TODO: Investigate if abort is possible.
    while (dma_channel_is_busy(flash_dma_chan));
#else
    dma_channel_abort(flash_dma_chan);
    xip_ctrl_hw->stream_ctr = 0;

    // Required to ensure the next flash transfer runs correctly.
    (void)*(io_rw_32*)XIP_NOCACHE_NOALLOC_BASE;
#endif
  }

  // Clear the receive FIFO
#ifndef USE_SSI_DMA
  while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
    (void) xip_ctrl_hw->stream_fifo;
#endif

  // Reset read pointers to beginning
  flash_buffer_ptr = flash_prev_buffer_ptr = flash_buffer;

  // Read one less word to prevent the write address wrapping back to the beginning
  // and then we can't tell whether no data has been written or all the data has been written.
  flash_total_words_requested = FLASH_BUF_LEN_WORDS - 1;
  if (flash_total_words_requested > flash_source_data_len) flash_total_words_requested = flash_source_data_len;

  // Setup the transfer
#ifdef USE_SSI_DMA
  ssi_hw->ssienr = 0;
  ssi_hw->ctrlr1 = flash_total_words_requested - 1;
  ssi_hw->dmacr = SSI_DMACR_BITS;
  ssi_hw->ssienr = 1;
#else
  xip_ctrl_hw->stream_addr = (uintptr_t)flash_source_data_ptr;
  xip_ctrl_hw->stream_ctr = flash_source_data_len;
#endif

  dma_channel_transfer_to_buffer_now(flash_dma_chan, flash_buffer, flash_total_words_requested);

#ifdef USE_SSI_DMA
  ssi_hw->dr0 = ((uintptr_t)flash_source_data_ptr << 8) | 0xa0;
#endif
}

void flash_set_stream(uint32_t* data, uint32_t len, bool start)
{
  // Remember where we are transferring from
  flash_source_data_ptr = data;
  flash_source_data_len = len;

  flash_reset_stream();
}

void flash_init(bool byte_swap)
{
  // Setup the DMA channel
  dma_channel_config c = dma_channel_get_default_config(flash_dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_ring(&c, true, FLASH_BUF_LOG_SIZE_BYTES);
  channel_config_set_bswap(&c, !byte_swap);  // Note data comes in byte swapped, so we reverse the request.

#ifdef USE_SSI_DMA
  channel_config_set_dreq(&c, DREQ_XIP_SSIRX);
  dma_channel_configure(
      flash_dma_chan,            // Channel to be configured
      &c,                        // The configuration we just created
      flash_buffer,              // The write address
      (const void*)&ssi_hw->dr0, // The read address
      0,                         // Number of transfers - set later
      false                      // Don't start yet
  );
#else
  channel_config_set_dreq(&c, DREQ_XIP_STREAM);
  dma_channel_configure(
      flash_dma_chan,            // Channel to be configured
      &c,                        // The configuration we just created
      flash_buffer,              // The write address
      (const void*)XIP_AUX_BASE, // The read address
      FLASH_BUF_LEN_WORDS,       // Number of transfers - set later
      false                      // Don't start yet
  );
#endif
}

// Request data from flash.  Returns amount of data that was
// available - may be less than amount requested, and sets ptr_out 
// to point to it.  The data returned will be discarded on next read.
uint32_t __no_inline_not_in_flash_func(flash_get_data)(uint32_t len_req, uint32_t** ptr_out)
{
  assert(len_req <= FLASH_BUF_LEN_WORDS / 2);
  assert(len_req > 0);

  uint32_t* write_ptr = (uint32_t*)dma_hw->ch[flash_dma_chan].write_addr;

  uint32_t words_available = 0;
  uint32_t true_words_available = 0;
  if (write_ptr < flash_buffer_ptr) {
    // Write pointer behind read pointer means we have wrapped.  Can read until end of buffer
    words_available = FLASH_BUF_LEN_WORDS - (flash_buffer_ptr - flash_buffer);
    true_words_available = words_available + (write_ptr - flash_buffer);
  }
  else if (write_ptr > flash_buffer_ptr)
  {
    words_available = write_ptr - flash_buffer_ptr;
    true_words_available = words_available;
  }

  // Can now write over previously returned buffer.
  if (words_available > 0)
    flash_prev_buffer_ptr = flash_buffer_ptr;

  // Check whether we should now restart the transfer
  if (!dma_channel_is_busy(flash_dma_chan))
  {
    flash_transfer();
  }
#ifndef USE_SSI_DMA
  else if (true_words_available < FLASH_BUF_LEN_WORDS / 2 && 
           dma_hw->ch[flash_dma_chan].transfer_count < FLASH_BUF_LEN_WORDS / 4 &&
           flash_source_data_len - flash_total_words_requested >= FLASH_BUF_LEN_WORDS / 4)
  {
    // Low buffer and near the end of the previous transfer so there's
    // more space for a longer transfer.  Restart to avoid getting to the end
    // of a transfer and having to wait for another read to restart it.
    dma_channel_abort(flash_dma_chan);
    flash_total_words_requested -= (flash_target_write_addr - (uint32_t*)dma_hw->ch[flash_dma_chan].write_addr) & FLASH_BUF_IDX_MASK;
    flash_transfer();
  }
#endif

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
  uint32_t words_to_read = len_in_words;
  uint32_t* write_ptr = dst_ptr;
  while (words_to_read > 0) {
    uint32_t* data_ptr;
    uint32_t len_req = MIN(words_to_read, FLASH_BUF_LEN_WORDS / 4);
    uint32_t words_read = flash_get_data(len_req, &data_ptr);
    memcpy(write_ptr, data_ptr, words_read * sizeof(uint32_t));
    write_ptr += words_read;
    words_to_read -= words_read;
  }
}

uint32_t __not_in_flash_func(flash_get_data_in_ringbuffer_blocking)(uint32_t len_req)
{
  assert(len_req <= FLASH_BUF_LEN_WORDS / 2);
  assert(len_req > 0);

  uint32_t words_available = ((uint32_t*)dma_hw->ch[flash_dma_chan].write_addr - flash_buffer_ptr) & FLASH_BUF_IDX_MASK;

  if (words_available < FLASH_BUF_LEN_WORDS / 4 && 
      !dma_channel_is_busy(flash_dma_chan) && 
      flash_total_words_requested < flash_source_data_len) 
  {
    assert((uint32_t*)dma_hw->ch[flash_dma_chan].write_addr != flash_prev_buffer_ptr);
    flash_transfer();
  }

  while (words_available < len_req) {
    if (!dma_channel_is_busy(flash_dma_chan))
    {
      assert((uint32_t*)dma_hw->ch[flash_dma_chan].write_addr != flash_prev_buffer_ptr);
      flash_transfer();
    }
    words_available = ((uint32_t*)dma_hw->ch[flash_dma_chan].write_addr - flash_buffer_ptr) & FLASH_BUF_IDX_MASK;
  }

  uint32_t idx_out = flash_buffer_ptr - flash_buffer;
  uint32_t new_idx = (idx_out + len_req) & FLASH_BUF_IDX_MASK;
  flash_buffer_ptr = flash_buffer + new_idx;

  return idx_out;
}

void __not_in_flash_func(flash_release_ringbuffer)()
{
  assert(flash_prev_buffer_ptr != flash_buffer_ptr);

  // Set prev pointer to one less than buffer pointer as buffer pointer must always be
  // one ahead of write pointer to prevent lock up.
  flash_prev_buffer_ptr = flash_buffer + (((flash_buffer_ptr - flash_buffer) - 1) & FLASH_BUF_IDX_MASK);

  // Check whether we should now restart the transfer
  if (!dma_channel_is_busy(flash_dma_chan))
  {
    flash_transfer();
  }
}