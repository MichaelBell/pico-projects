#include <stdio.h>
#include "pico/stdlib.h"

#include "pico/sd_card.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/sync.h"

#include "sdring.h"

uint32_t sdring_buffer_0[SDRING_BUF_LEN_WORDS] __attribute__((aligned(1 << SDRING_BUF_LOG_SIZE_BYTES)));
uint32_t sdring_buffer_1[SDRING_BUF_LEN_WORDS] __attribute__((aligned(1 << SDRING_BUF_LOG_SIZE_BYTES)));

static uint32_t sdring_ctrl_words[2][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) + 4];
static uint32_t sdring_crc_dump[2];

typedef struct stream_data {
  uint32_t* buffer;

  // Location in buffer that was returned to the user on last read
  // We should write past here.
  uint32_t* prev_buffer_ptr;

  // Location in buffer that will be returned to the user on next read
  uint32_t* buffer_ptr;

  // Start and end of current transfer
  uint32_t* start_write_addr;
  uint32_t* target_write_addr;

  // Record of the first control word sent to compute number of sectors read
  uint32_t* first_ctrl_word;
  uint32_t end_ctrl_word_idx;
  uint32_t sectors_stolen;

  // Next block to request
  uint32_t next_block_to_request;

  // Source data
  uint32_t source_start_block;
  uint32_t source_end_block;  // Actually one past end
  uint32_t source_data_len;  // Note this is in bytes.

  // Config
  uint8_t blocks_per_read;
  uint8_t blocks_on_next_read;
  bool byte_swap;

} stream_data_t;

stream_data_t streams[2];

#define STRM streams[stream_idx]

bool sdring_eof(uint32_t stream_idx)
{
  return (STRM.next_block_to_request >= STRM.source_end_block) &&
         (STRM.start_write_addr == STRM.target_write_addr);
}

static void sdring_start_transfer(uint32_t stream_idx, bool start_other_stream_if_idle)
{
  auto_init_mutex(transfer_mutex);

  if (!mutex_try_enter(&transfer_mutex, NULL))
    return;

  if (!sd_scatter_read_complete(NULL, NULL))
  {
    mutex_exit(&transfer_mutex);
    return;
  }

  // Previous transfers are complete
  streams[0].start_write_addr = streams[0].target_write_addr;
  streams[1].start_write_addr = streams[1].target_write_addr;

  if (sdring_eof(stream_idx)) 
  {
    mutex_exit(&transfer_mutex);
    if (start_other_stream_if_idle)
      sdring_start_transfer(stream_idx ^ 1, false);
    return;
  }

  // Set window for next transfer
  uint32_t start_write_idx = STRM.target_write_addr - STRM.buffer;

  // Read
  uint32_t blocks_to_read = MIN(STRM.blocks_on_next_read, STRM.source_end_block - STRM.next_block_to_request);

  // Don't catch up with the read pointer.
  uint32_t read_idx = STRM.prev_buffer_ptr - STRM.buffer;
  uint32_t space_available = (read_idx - start_write_idx) & SDRING_BUF_IDX_MASK;
  if (space_available < (blocks_to_read << 7))
  {
    if (blocks_to_read == STRM.blocks_per_read && (space_available >= (blocks_to_read << 6)))
    {
      blocks_to_read >>= 1;

      // Maintain alignment by doing a small read next time too.
      STRM.blocks_on_next_read = blocks_to_read;
    }
    else
    {
      mutex_exit(&transfer_mutex);
      if (start_other_stream_if_idle)
        sdring_start_transfer(stream_idx ^ 1, false);
      return;
    }
  }
  else
  {
    // Must be aligned after read unles EOF
    assert(((start_write_idx + (blocks_to_read << 7)) & (128 * STRM.blocks_per_read - 1)) == 0 ||
           (blocks_to_read + STRM.next_block_to_request == STRM.source_end_block));
    STRM.blocks_on_next_read = STRM.blocks_per_read;
  }

  uint32_t target_write_idx = (start_write_idx + (blocks_to_read << 7)) & SDRING_BUF_IDX_MASK;

  sd_set_byteswap_on_read(STRM.byte_swap);
  
  // The last transfer will have blatted the first control entry
  sdring_ctrl_words[stream_idx][(start_write_idx >> 6)] = (uint32_t)STRM.start_write_addr;
  sdring_ctrl_words[stream_idx][(start_write_idx >> 6) + 1] = 130;

  // Ensure the transfer ends!
  // Don't need to do this if the write goes to the end of the buffer, as that is handled specially.
  if (target_write_idx != 0)
  {
    sdring_ctrl_words[stream_idx][(target_write_idx >> 6)] = 0;
    sdring_ctrl_words[stream_idx][(target_write_idx >> 6) + 1] = 0;
  }

  STRM.first_ctrl_word = &sdring_ctrl_words[stream_idx][(start_write_idx >> 6)];
  STRM.end_ctrl_word_idx = (target_write_idx >> 6);
  sd_readblocks_scatter_async(STRM.first_ctrl_word, STRM.next_block_to_request, blocks_to_read);

  STRM.target_write_addr = STRM.buffer + target_write_idx;
  STRM.next_block_to_request += blocks_to_read;
  STRM.sectors_stolen = 0;

  mutex_exit(&transfer_mutex);
}

static void sdring_configure_ctrl_words(uint32_t stream_idx)
{
  for (int i = 0; i < (1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)); i += 2)
  {
    sdring_ctrl_words[stream_idx][i] = (uint32_t)(STRM.buffer + (i << 6));
    sdring_ctrl_words[stream_idx][i + 1] = 130;
  }
  sdring_ctrl_words[stream_idx][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) - 1] = 128;
  sdring_ctrl_words[stream_idx][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) + 0] = (uint32_t)sdring_crc_dump;
  sdring_ctrl_words[stream_idx][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) + 1] = 2;
  sdring_ctrl_words[stream_idx][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) + 2] = 0;
  sdring_ctrl_words[stream_idx][(1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) + 3] = 0;
}

// Configure the flash streaming resources
void sdring_init(bool byte_swap)
{
  sd_init_4pins();
  sd_set_clock_divider(2);
  sd_set_byteswap_on_read(byte_swap);

  streams[0].buffer = sdring_buffer_0;
  streams[0].buffer_ptr = sdring_buffer_0;
  streams[0].prev_buffer_ptr = sdring_buffer_0;
  streams[0].source_start_block = 0;
  streams[0].source_end_block = 0;
  streams[0].source_data_len = 0;
  streams[0].next_block_to_request = 0;
  streams[0].first_ctrl_word = sdring_ctrl_words[0];
  streams[0].end_ctrl_word_idx = 0;
  streams[0].sectors_stolen = 0;
  streams[0].start_write_addr = sdring_buffer_0;
  streams[0].target_write_addr = sdring_buffer_0;
  streams[0].blocks_per_read = 64;
  streams[0].blocks_on_next_read = 64;
  streams[0].byte_swap = true;
  sdring_configure_ctrl_words(0);

  streams[1].buffer = sdring_buffer_1;
  streams[1].buffer_ptr = sdring_buffer_1;
  streams[1].prev_buffer_ptr = sdring_buffer_1;
  streams[1].source_start_block = 0;
  streams[1].source_end_block = 0;
  streams[1].source_data_len = 0;
  streams[1].next_block_to_request = 0;
  streams[1].sectors_stolen = 0;
  streams[1].first_ctrl_word = sdring_ctrl_words[1];
  streams[1].end_ctrl_word_idx = 0;
  streams[1].start_write_addr = sdring_buffer_1;
  streams[1].target_write_addr = sdring_buffer_1;
  streams[1].blocks_per_read = 4;
  streams[1].blocks_on_next_read = 4;
  streams[1].byte_swap = false;
  sdring_configure_ctrl_words(1);
}

// Set the data to stream and optionally start streaming.
// If start_streaming is false then start the stream by calling reset_stream.
void sdring_set_stream(uint32_t stream_idx, uint32_t start_block, uint32_t len_bytes, bool start_streaming, bool prime_buffer)
{
  STRM.source_start_block = start_block;
  STRM.source_end_block = start_block + (len_bytes / 512) + 1;
  STRM.source_data_len = len_bytes;

  if (start_streaming)
    sdring_reset_stream(stream_idx, prime_buffer);
}

// Reset the stream to the beginning.
void sdring_reset_stream(uint32_t stream_idx, bool prime_buffer)
{
  // Reset state
  STRM.next_block_to_request = STRM.source_start_block;
  STRM.start_write_addr = STRM.buffer;
  STRM.target_write_addr = STRM.buffer;
  STRM.prev_buffer_ptr = STRM.buffer + SDRING_BUF_LEN_WORDS - 1;
  STRM.buffer_ptr = STRM.buffer;

  if (STRM.end_ctrl_word_idx < (1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)))
  {
    sdring_ctrl_words[stream_idx][STRM.end_ctrl_word_idx] = (uint32_t)(STRM.buffer + ((STRM.end_ctrl_word_idx) << 6));
    if (STRM.end_ctrl_word_idx != (1 << (SDRING_BUF_LOG_SIZE_BYTES - 8)) - 2)
    {
      sdring_ctrl_words[stream_idx][STRM.end_ctrl_word_idx + 1] = 130;
    }
    else
    {
      sdring_ctrl_words[stream_idx][STRM.end_ctrl_word_idx + 1] = 128;
    }
  }

  // Wait for any active transfers to complete.
  while (!sd_scatter_read_complete(NULL, NULL))
  {
    //__breakpoint();
  }

  // Kick off first read
  if (prime_buffer)
    STRM.blocks_on_next_read = 64;
  else
    STRM.blocks_on_next_read = STRM.blocks_per_read;
  sdring_start_transfer(stream_idx, false);
}

uint32_t sdring_words_available(uint32_t stream_idx)
{
  
  if (sd_scatter_read_complete(NULL, NULL))
  {
    // No transfer currently running, start a new one.
    assert(STRM.start_write_addr != STRM.prev_buffer_ptr);
    sdring_start_transfer(stream_idx, false);
  }
  else if (STRM.target_write_addr != STRM.start_write_addr)
  {
    // We are reading from this stream, work out how many blocks have been
    // written by inspecting the SD chain DMA
    int sectors_read = (((uint32_t*)dma_channel_hw_addr(9)->read_addr - STRM.first_ctrl_word) >> 1) - 1;

    if (sectors_read > STRM.sectors_stolen)
    {
      // There is a transfer running for this stream, update how
      // many sectors we can "steal", i.e. read ahead into because
      // the running transfer has already written them.
      uint32_t new_sectors = sectors_read - STRM.sectors_stolen;
      STRM.sectors_stolen += new_sectors;
      STRM.start_write_addr += new_sectors * 128;
    }
  } 
  return (STRM.start_write_addr - STRM.buffer_ptr) & SDRING_BUF_IDX_MASK;
}

// Request a fixed amount of data from flash and access in ring buffer.
// The flash ring buffer is aligned and you may need to wrap the returned index
// Total length requested at any given time must be <= FLASH_BUF_LEN_WORDS - 1,
// and ideally you should hold less than FLASH_BUF_LEN_WORDS / 2 so that
// new data can be buffered.
// Blocks until the requested amount of data has been read from flash.
// Calling this a subsequent time does *not* release the data, it must be
// released explicitly.
uint32_t sdring_get_data_in_ringbuffer_blocking(uint32_t stream_idx, uint32_t len_req)
{
  assert(len_req <= SDRING_BUF_LEN_WORDS / 2);
  assert(len_req > 0);

  uint32_t words_available = (STRM.start_write_addr - STRM.buffer_ptr) & SDRING_BUF_IDX_MASK;

#if 1
  if (words_available < SDRING_BUF_LEN_WORDS / 4 && 
      sd_scatter_read_complete(NULL, NULL) && 
      !sdring_eof(stream_idx))
  {
    assert(STRM.start_write_addr != STRM.prev_buffer_ptr);
    sdring_start_transfer(stream_idx, false);
  }
#endif

  int sectors_read = 0;
  while (words_available < len_req) {
    words_available = sdring_words_available(stream_idx);
  }

  uint32_t idx_out = STRM.buffer_ptr - STRM.buffer;
  uint32_t new_idx = (idx_out + len_req) & SDRING_BUF_IDX_MASK;
  STRM.buffer_ptr = STRM.buffer + new_idx;

  return idx_out;
}

// Release all data previously fetched with ringbuffer_blocking.
void sdring_release_ringbuffer(uint32_t stream_idx)
{
  assert(STRM.prev_buffer_ptr != STRM.buffer_ptr);

  // Set prev pointer to one less than buffer pointer as buffer pointer must always be
  // one ahead of write pointer to prevent lock up.
  STRM.prev_buffer_ptr = STRM.buffer + (((STRM.buffer_ptr - STRM.buffer) - 1) & SDRING_BUF_IDX_MASK);

  // Check whether we should now restart the transfer
  if (sd_scatter_read_complete(NULL, NULL))
  {
    sdring_start_transfer(stream_idx, true);
  }
}
