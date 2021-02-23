// Restart the stream
void decomp_reset_stream();

// Setup a new compressed stream reading from flash at the given address.
// If start_streaming is false then start the stream by calling reset_stream.
void decomp_set_stream(uint32_t* data, uint32_t len, bool start_streaming);

// Read out a symbol table
#define SYMBOLS_IN_TABLE 64
void decomp_read_table(uint16_t dst_table[SYMBOLS_IN_TABLE]);

// Read out bits.
uint32_t decomp_get_bits(int32_t bit_len);

// Read out a single 10-bit symbol using the given table for decode
uint16_t decomp_get_symbol(uint16_t* table);

// Number of compressed bits that have been read.  This may be reset by the user.
extern uint32_t compressed_bits_read;