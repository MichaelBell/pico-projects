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

#if 0
 // Timings for 640x480
#define TIMING_V_PULSE    2
#define TIMING_V_BACK    (30 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (480 + TIMING_V_BACK)
#define TIMING_V_FRONT   (9 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   16
#define TIMING_H_PULSE   96
#define TIMING_H_BACK    48
#define TIMING_H_DISPLAY 640
#define CLOCK_VCO   1500
#define CLOCK_PD1   6
#define CLOCK_PD2   5
#define CLOCK_RATE  (50 * MHZ)
#else
 // Timings for 720p
#define TIMING_V_PULSE    5
#define TIMING_V_BACK    (20 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (720 + TIMING_V_BACK)
#define TIMING_V_FRONT   (3 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   64
#define TIMING_H_PULSE   128
#define TIMING_H_BACK    192
#define TIMING_H_DISPLAY 1280
#define CLOCK_VCO   1044
#define CLOCK_PD1   7
#define CLOCK_PD2   1
#define CLOCK_RATE  149142857
#endif

uint16_t timing_row = 0;

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

    // Prime the timing SM
    drive_timing();
    drive_timing();

    while (1) {
        __wfi();
    }
}

int main()
{
    // Set appropriate clock
    set_sys_clock_pll(CLOCK_VCO * MHZ, CLOCK_PD1, CLOCK_PD2);

    // Tell the periperal clock the new sys clock speed
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    CLOCK_RATE,
                    CLOCK_RATE);

    //stdio_init_all();

    multicore_launch_core1(vga_entry);

    display_loop();

    return 0;
}
