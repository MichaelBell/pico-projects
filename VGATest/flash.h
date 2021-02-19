// Copyright (C) 2021 Michael Bell
// You may use this under the terms of the BSD 3 clause license.

// Configure the flash streaming resources
void flash_init();

// Set the data to stream and start streaming.
void flash_set_stream(uint32_t* data, uint32_t len);

// Request data from flash.  Returns amount of data that was
// available - may be less than amount requested, and sets ptr_out 
// to point to it.  The data returned will be discarded on next read.
uint32_t flash_get_data(uint32_t len_req_in_words, uint32_t** ptr_out);

// Copy data out of the stream - user friendly interface to the above.
void flash_copy_data_blocking(uint32_t* dst_ptr, uint32_t len_in_words);

// Reset the stream to the beginning.
void flash_reset_stream();

