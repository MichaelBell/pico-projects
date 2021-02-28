#include <stdio.h>
#include "pico/stdlib.h"

#include "pico/sd_card.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "sdring.h"

#define SDRING_BLOCKS_PER_READ 32
#define SDRING_WORDS_PER_READ (SDRING_BLOCKS_PER_READ * 128)

uint32_t sdring_buffer[SDRING_BUF_LEN_WORDS] __attribute__((aligned(1 << SDRING_BUF_LOG_SIZE_BYTES)));

// Location in buffer that was returned to the user on last read
// We should write past here.
static uint32_t* sdring_prev_buffer_ptr;

// Location in buffer that will be returned to the user on next read
static uint32_t* sdring_buffer_ptr;

// Start and end of current transfer
static uint32_t* sdring_start_write_addr;
static uint32_t* sdring_target_write_addr;

// Next block to request
static uint32_t sdring_next_block_to_request;

// Source data
static uint32_t sdring_source_start_block;
static uint32_t sdring_source_end_block;  // Actually one past end
static uint32_t sdring_source_data_len;  // Note this is in bytes.

inline static bool sdring_eof()
{
  return ((sdring_next_block_to_request - sdring_source_start_block) << 9) >= sdring_source_data_len;
}

static void sdring_start_transfer()
{
  if (!sd_scatter_read_complete(NULL, NULL))
    return;

  // Previous transfer is complete
  sdring_start_write_addr = sdring_target_write_addr;

  if (sdring_eof()) 
    return;

  // Set window for next transfer
  uint32_t next_idx = sdring_target_write_addr - sdring_buffer;
  next_idx = (next_idx + SDRING_WORDS_PER_READ) & SDRING_BUF_IDX_MASK;

  // Don't catch up with the read pointer.
  uint32_t read_idx = sdring_prev_buffer_ptr - sdring_buffer;
  uint32_t space_available = (read_idx - next_idx) & SDRING_BUF_IDX_MASK;
  if (space_available <= SDRING_WORDS_PER_READ)
    return;

  // Read
  uint32_t blocks_to_read = MIN(SDRING_BLOCKS_PER_READ, sdring_source_end_block - sdring_next_block_to_request);
  sd_readblocks_async(sdring_start_write_addr, sdring_next_block_to_request, blocks_to_read);
  sdring_target_write_addr = sdring_buffer + next_idx;
  sdring_next_block_to_request += SDRING_BLOCKS_PER_READ;
}

// Configure the flash streaming resources
void sdring_init(bool byte_swap)
{
  sd_init_4pins();
  sd_set_clock_divider(2);
}

// Set the data to stream and optionally start streaming.
// If start_streaming is false then start the stream by calling reset_stream.
void sdring_set_stream(uint32_t start_block, uint32_t len_bytes, bool start_streaming)
{
  sdring_source_start_block = start_block;
  sdring_source_end_block = start_block + (len_bytes + 511) / 512;
  sdring_source_data_len = len_bytes;

  sdring_reset_stream();
}

// Reset the stream to the beginning.
void sdring_reset_stream()
{
  // Reset state
  sdring_next_block_to_request = sdring_source_start_block;
  sdring_start_write_addr = sdring_buffer;
  sdring_target_write_addr = sdring_buffer;
  sdring_prev_buffer_ptr = sdring_buffer + SDRING_BUF_LEN_WORDS - 1;
  sdring_buffer_ptr = sdring_buffer;

  // Wait for any active transfers to complete.
  while (!sd_scatter_read_complete(NULL, NULL))
  {
    //__breakpoint();
  }

  // Kick off first read
  sdring_start_transfer();
}

// Request a fixed amount of data from flash and access in ring buffer.
// The flash ring buffer is aligned and you may need to wrap the returned index
// Total length requested at any given time must be <= FLASH_BUF_LEN_WORDS - 1,
// and ideally you should hold less than FLASH_BUF_LEN_WORDS / 2 so that
// new data can be buffered.
// Blocks until the requested amount of data has been read from flash.
// Calling this a subsequent time does *not* release the data, it must be
// released explicitly.
uint32_t sdring_get_data_in_ringbuffer_blocking(uint32_t len_req)
{
  assert(len_req <= SDRING_BUF_LEN_WORDS / 2);
  assert(len_req > 0);

  uint32_t words_available = (sdring_start_write_addr - sdring_buffer_ptr) & SDRING_BUF_IDX_MASK;

  if (words_available < SDRING_BUF_LEN_WORDS / 4 && 
      sd_scatter_read_complete(NULL, NULL) && 
      !sdring_eof())
  {
    assert(sdring_start_write_addr != sdring_prev_buffer_ptr);
    sdring_start_transfer();
  }

  int sectors_read = 0;
  while (words_available + (128 * sectors_read) < len_req) {
    sectors_read = 0;
    if (sd_scatter_read_complete(NULL, &sectors_read))
    {
      assert(sdring_start_write_addr != sdring_prev_buffer_ptr);
      sdring_start_transfer();
    }
    words_available = (sdring_start_write_addr - sdring_buffer_ptr) & SDRING_BUF_IDX_MASK;
  }

  uint32_t idx_out = sdring_buffer_ptr - sdring_buffer;
  uint32_t new_idx = (idx_out + len_req) & SDRING_BUF_IDX_MASK;
  sdring_buffer_ptr = sdring_buffer + new_idx;

  return idx_out;
}

// Release all data previously fetched with ringbuffer_blocking.
void sdring_release_ringbuffer()
{
  assert(sdring_prev_buffer_ptr != sdring_buffer_ptr);

  // Set prev pointer to one less than buffer pointer as buffer pointer must always be
  // one ahead of write pointer to prevent lock up.
  sdring_prev_buffer_ptr = sdring_buffer + (((sdring_buffer_ptr - sdring_buffer) - 1) & SDRING_BUF_IDX_MASK);

  // Check whether we should now restart the transfer
  if (sd_scatter_read_complete(NULL, NULL))
  {
    sdring_start_transfer();
  }
}

