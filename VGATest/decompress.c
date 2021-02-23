#include "pico/stdlib.h"
#include "flash.h"

static uint8_t* compressed_data_ptr;
static uint8_t* compressed_data_end;
static uint32_t compressed_bits;
static uint8_t compressed_bit_len;
uint32_t compressed_bits_read;

void decomp_reset_stream()
{
  flash_reset_stream();
  compressed_data_ptr = NULL;
  compressed_data_end = NULL;
  compressed_bits = 0;
  compressed_bit_len = 0;
  compressed_bits_read = 0;
}

// Setup a new compressed stream reading from flash at the given address.
void decomp_set_stream(uint32_t* data, uint32_t len, bool start_streaming)
{
  flash_set_stream(data, len, false);
  if (start_streaming)
    decomp_reset_stream();
}

static void read_more_data()
{
  uint32_t* new_data;
  uint32_t words_read = 0;
  while (!words_read) words_read = flash_get_data(128, &new_data);
  compressed_data_ptr = (uint8_t*)new_data;
  compressed_data_end = (uint8_t*)(new_data + words_read);
}

// Read out bits.
uint32_t decomp_get_bits(uint8_t bit_len)
{
  while (compressed_bit_len < bit_len)
  {
    if (compressed_data_ptr == compressed_data_end)
    {
      read_more_data();
    }
    compressed_bits <<= 8;
    compressed_bits |= *compressed_data_ptr++;
    compressed_bit_len += 8;
  }

  compressed_bit_len -= bit_len;
  compressed_bits_read += bit_len;

  uint32_t result = compressed_bits >> compressed_bit_len;
  compressed_bits &= (1u << compressed_bit_len) - 1;
  return result;
}

// Read out a symbol table
#define SYMBOLS_IN_TABLE 64
void decomp_read_table(uint16_t dst_table[SYMBOLS_IN_TABLE])
{
  for (int i = 0; i < SYMBOLS_IN_TABLE; ++i)
  {
    dst_table[i] = decomp_get_bits(10);
  }
}

// Read out a single 10-bit symbol using the given table for decode
uint32_t decomp_get_symbol(uint16_t* table)
{
  if (decomp_get_bits(1))
  {
    // Not encoded
    return decomp_get_bits(10);
  }
  else
  {
    return table[decomp_get_bits(6)];
  }
}