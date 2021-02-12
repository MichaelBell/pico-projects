#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/xip_ctrl.h"

#include "flash_data.h"

#define FLASH_BUFFER_LEN 512
uint flash_dma_chan;
uint32_t flash_read_buffer[FLASH_BUFFER_LEN / 4];
uintptr_t flash_last_read_addr = 0;

void flash_dma_init()
{
  flash_dma_chan = dma_claim_unused_channel(true);

  dma_channel_config cfg = dma_channel_get_default_config(flash_dma_chan);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_XIP_STREAM);
  dma_channel_configure(
            flash_dma_chan,
            &cfg,
            (void *) flash_read_buffer,   // Write addr
            (const void *) XIP_AUX_BASE,  // Read addr
            FLASH_BUFFER_LEN / 4,         // Transfer count
            false
    );
}

void flash_start_dma(uint8_t* addr)
{
  while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
    (void) xip_ctrl_hw->stream_fifo;
  xip_ctrl_hw->stream_addr = (uintptr_t)addr & (~3u);
  xip_ctrl_hw->stream_ctr = FLASH_BUFFER_LEN / 4;

  dma_channel_transfer_to_buffer_now(flash_dma_chan, flash_read_buffer, FLASH_BUFFER_LEN / 4);
  flash_last_read_addr = (uintptr_t)addr & (~3u);
}

void flash_stop_dma()
{
  if (dma_channel_is_busy(flash_dma_chan)) {
    xip_ctrl_hw->stream_ctr = 0;
    dma_channel_abort(flash_dma_chan);
    //sleep_us(1);  // Transfer reliable if you uncomment this
  }
}

int main()
{
    stdio_init_all();

    flash_dma_init();

    puts("Hello, world!");

    // Start the streaming transfer from flash
    flash_start_dma((uint8_t*)flash_data);

    uint count_correct = 0;
    while (1) {
      uint32_t buff[4];

      // Wait until write address on the DMA has been incremented twice
      uint32_t read_end_addr = ((uintptr_t)flash_read_buffer + 7);
      while (dma_hw->ch[flash_dma_chan].write_addr < read_end_addr);

      // Copy out the beginning of the buffer being DMA'd into 
      memcpy(buff, flash_read_buffer, 16);

      // Stop and immediately restart the streaming transfer
      flash_stop_dma();
      flash_start_dma((uint8_t*)flash_data);

      // First word to be transferred should always be 0xffff
      // Rest of the words just count up from 1.
      if (buff[0] != 0xffff) {
        // Print out the first four words that were in the receiving buffer, 
        // and the count of correct transfers
        printf("%08x %08x %08x %08x %d\n", buff[0], buff[1], buff[2], buff[3], count_correct);

        // Reset and pause
        count_correct = 0;
        flash_stop_dma();
        sleep_ms(100);

        // Clear the buffer and start again
        memset(flash_read_buffer, 0, FLASH_BUFFER_LEN);
        flash_start_dma((uint8_t*)flash_data);
      }
      else count_correct++;
    }

    return 0;
}
