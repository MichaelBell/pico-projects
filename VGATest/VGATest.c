#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "vga.h"
#include "vga.pio.h"

const uint CAPTURE_PIN_BASE = 0;
const uint CAPTURE_PIN_COUNT = 32;
const uint CAPTURE_N_SAMPLES = 200;

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div);
void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level);
void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples);

const uint analyser_sm = 3;

 // Timings from Basys 2 manual
#define TIMING_V_PULSE    2
#define TIMING_V_BACK    (30 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (480 + TIMING_V_BACK)
#define TIMING_V_FRONT   (9 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   16
#define TIMING_H_PULSE   96
#define TIMING_H_BACK    48
#define TIMING_H_DISPLAY 640

uint16_t timing_row = 0;

#define NEED_DISPLAY() (timing_row >= (TIMING_V_BACK - 2) && timing_row < (TIMING_V_DISPLAY - 2) - 1)
#define GET_NEXT_DISPLAY_ROW() (timing_row - (TIMING_V_BACK - 2) + 1)

void __no_inline_not_in_flash_func(drive_timing)()
{
    // TODO: If we have other interrupt load on this core then only
    //       queueing one line ahead could get dicey.  Would be better
    //       to just fill the channel as it empties.

    // Front Porch
    uint32_t instr = 0x4000A042u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_FRONT - 3) << 16;
    pio_sm_put_blocking(vga_pio, vga_timing_sm, instr);

    // HSYNC
    instr = 0x0000A042u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_PULSE - 3) << 16;
    pio_sm_put_blocking(vga_pio, vga_timing_sm, instr);

    // Back Porch, trigger pixel channels if in display window
    instr = 0x4000C004u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    if (timing_row >= TIMING_V_BACK && timing_row < TIMING_V_DISPLAY) instr |= 0xC004u;
    else instr |= 0xA042u;
    instr |= (TIMING_H_BACK - 3) << 16;
    pio_sm_put_blocking(vga_pio, vga_timing_sm, instr);

    // Display, trigger next line at end
    instr = 0x4000C001u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_DISPLAY - 3) << 16;
    pio_sm_put_blocking(vga_pio, vga_timing_sm, instr);

    if (++timing_row >= TIMING_V_FRONT) timing_row = 0;
}

void __no_inline_not_in_flash_func(end_of_line_isr)() {
    hw_clear_bits(&vga_pio->irq, 0x2);
    drive_timing();

    if (timing_row == 0)
    {
        display_start_new_frame();
    }
    else if (NEED_DISPLAY())
    {
        display_next_row(GET_NEXT_DISPLAY_ROW());
    }
    if (timing_row == TIMING_V_DISPLAY + 3)
    {
        display_end_frame();
    }
    else if (timing_row == TIMING_V_DISPLAY + 4)
    {
        // Force SMs to get back into a good state:
        //       Disable
        //       Reset PC
        //       Clear OSR (it is read when SM is re-enabled)
        //       Drain FIFOs
        //       Re-enable
        for (uint sm = vga_red_sm; sm <= vga_blue_sm; ++sm)
        {
            pio_sm_set_enabled(vga_pio, sm, false);
            pio_sm_exec(vga_pio, sm, pio_encode_jmp(vga_channel_offset_end));
            pio_sm_exec(vga_pio, sm, pio_encode_mov(pio_osr, pio_null));
            pio_sm_drain_tx_fifo(vga_pio, sm);
            pio_sm_set_enabled(vga_pio, sm, true);
        }
    }
}

// Setup must happen on core 1, to ensure interrupts are serviced fast enough.
void vga_entry() {
    uint offset = pio_add_program(vga_pio, &vga_channel_program);
    assert(offset == 0);
    vga_channel_program_init(vga_pio, vga_red_sm, 0);
    vga_channel_program_init(vga_pio, vga_green_sm, 6);
    vga_channel_program_init(vga_pio, vga_blue_sm, 11);
    offset = pio_add_program(vga_pio, &vga_timing_program);
    vga_timing_program_init(vga_pio, vga_timing_sm, offset, 16);

    hw_set_bits(&vga_pio->inte0, 0xf00);
    irq_set_exclusive_handler(PIO0_IRQ_0, end_of_line_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Setup DMAs
    dma_channel_config c = dma_channel_get_default_config(vga_red_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_red_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_red_dma,               // Channel to be configured
        &c,                        // The configuration we just created
        &vga_pio->txf[vga_red_sm], // The write address
        NULL,                      // The initial read address - set later
        0,                         // Number of transfers - set later
        false                      // Don't start yet
    );

    c = dma_channel_get_default_config(vga_green_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_green_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_green_dma,               // Channel to be configured
        &c,                          // The configuration we just created
        &vga_pio->txf[vga_green_sm], // The write address
        NULL,                        // The initial read address - set later
        0,                           // Number of transfers - set later
        false                        // Don't start yet
    );

    c = dma_channel_get_default_config(vga_blue_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(vga_pio, vga_blue_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        vga_blue_dma,               // Channel to be configured
        &c,                         // The configuration we just created
        &vga_pio->txf[vga_blue_sm], // The write address
        NULL,                       // The initial read address - set later
        0,                          // Number of transfers - set later
        false                       // Don't start yet
    );

    // Start display
    display_start_new_frame();

    // Prime the timing SM
    drive_timing();
    drive_timing();

    while (1) {
        __wfi();
    }
}

int main()
{
    // 50 MHz clock, for 25MHz pixel clock
    set_sys_clock_pll(1500 * MHZ, 6, 5);

    // Tell the periperal clock the new sys clock speed
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    50 * MHZ,
                    50 * MHZ);

    //stdio_init_all();

    multicore_launch_core1(vga_entry);

    display_loop();

    return 0;
}
