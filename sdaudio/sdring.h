// Copyright (C) 2021 Michael Bell
// You may use this under the terms of the BSD 3 clause license.

// Size of the ring buffer - must be a power of 2 >= 2^11 and <= 2^15 bytes.
#define SDRING_BUF_LOG_SIZE_BYTES 16
#define SDRING_BUF_LEN_WORDS (1 << (SDRING_BUF_LOG_SIZE_BYTES - 2))
#define SDRING_BUF_IDX_MASK  (SDRING_BUF_LEN_WORDS - 1)

// Configure the flash streaming resources
void sdring_init(bool byte_swap);

// Set the data to stream and optionally start streaming.
// If start_streaming is false then start the stream by calling reset_stream.
void sdring_set_stream(uint32_t start_block, uint32_t len_bytes, bool start_streaming);

// Reset the stream to the beginning.
void sdring_reset_stream();

// Request a fixed amount of data from flash and access in ring buffer.
// The flash ring buffer is aligned and you may need to wrap the returned index
// Total length requested at any given time must be <= FLASH_BUF_LEN_WORDS - 1,
// and ideally you should hold less than FLASH_BUF_LEN_WORDS / 2 so that
// new data can be buffered.
// Blocks until the requested amount of data has been read from flash.
// Calling this a subsequent time does *not* release the data, it must be
// released explicitly.
uint32_t sdring_get_data_in_ringbuffer_blocking(uint32_t len_req_in_words);

// Release all data previously fetched with ringbuffer_blocking.
void sdring_release_ringbuffer();

extern uint32_t sdring_buffer[SDRING_BUF_LEN_WORDS] __attribute__((aligned(1 << SDRING_BUF_LOG_SIZE_BYTES)));