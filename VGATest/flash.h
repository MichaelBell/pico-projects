// Copyright (C) 2021 Michael Bell
// You may use this under the terms of the BSD 3 clause license.

// Size of the ring buffer - must be a power of 2 <= 2^15 bytes.
#define FLASH_BUF_LOG_SIZE_BYTES 14
#define FLASH_BUF_LEN_WORDS (1 << (FLASH_BUF_LOG_SIZE_BYTES - 2))
#define FLASH_BUF_IDX_MASK  (FLASH_BUF_LEN_WORDS - 1)

// Note this is in the range of channels that the SD card uses
// Currently assuming we'll use either SD or flash.
#define flash_dma_chan 11

// Configure the flash streaming resources
void flash_init(bool byte_swap);

// Set the data to stream and optionally start streaming.
// If start_streaming is false then start the stream by calling reset_stream.
void flash_set_stream(uint32_t* data, uint32_t len, bool start_streaming);

// Request data from flash.  Returns amount of data that was
// available - may be less than amount requested, and sets ptr_out 
// to point to it.  The data returned will be discarded on next read.
uint32_t flash_get_data(uint32_t len_req_in_words, uint32_t** ptr_out);

// Copy data out of the stream - user friendly interface to the above.
void flash_copy_data_blocking(uint32_t* dst_ptr, uint32_t len_in_words);

// Reset the stream to the beginning.
void flash_reset_stream();

// Request a fixed amount of data from flash and access in ring buffer.
// The flash ring buffer is aligned and you may need to wrap the returned index
// Total length requested at any given time must be <= FLASH_BUF_LEN_WORDS - 1,
// and ideally you should hold less than FLASH_BUF_LEN_WORDS / 2 so that
// new data can be buffered.
// Blocks until the requested amount of data has been read from flash.
// Calling this a subsequent time does *not* release the data, it must be
// released explicitly.
uint32_t flash_get_data_in_ringbuffer_blocking(uint32_t len_req_in_words);

// Release all data previously fetched with ringbuffer_blocking.
void flash_release_ringbuffer();

extern uint32_t flash_buffer[FLASH_BUF_LEN_WORDS] __attribute__((aligned(1 << FLASH_BUF_LOG_SIZE_BYTES)));