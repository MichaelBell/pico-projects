#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

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
const uint vga_red_sm = 0;
const uint vga_green_sm = 1;
const uint vga_blue_sm = 2;
const uint vga_timing_sm = 3;

#define NEED_DISPLAY() (timing_row >= (TIMING_V_BACK - 2) && timing_row < (TIMING_V_DISPLAY - 2))
#define GET_NEXT_DISPLAY_ROW() (timing_row - (TIMING_V_BACK - 2))

void __no_inline_not_in_flash_func(drive_timing)()
{
    // TODO: If we have other interrupt load on this core then only
    //       queueing one line ahead could get dicey.  Would be better
    //       to just fill the channel as it empties.

    // Front Porch
    uint32_t instr = 0x4000A042u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_FRONT - 3) << 16;
    pio_sm_put_blocking(pio0, vga_timing_sm, instr);

    // HSYNC
    instr = 0x0000A042u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_PULSE - 3) << 16;
    pio_sm_put_blocking(pio0, vga_timing_sm, instr);

    // Back Porch, trigger pixel channels if in display window
    instr = 0x4000C004u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    if (timing_row >= TIMING_V_BACK && timing_row < TIMING_V_DISPLAY) instr |= 0xC004u;
    else instr |= 0xA042u;
    instr |= (TIMING_H_BACK - 3) << 16;
    pio_sm_put_blocking(pio0, vga_timing_sm, instr);

    // Display, trigger next line at end
    instr = 0x4000C001u;
    if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
    instr |= (TIMING_H_DISPLAY - 3) << 16;
    pio_sm_put_blocking(pio0, vga_timing_sm, instr);

    if (++timing_row >= TIMING_V_FRONT) timing_row = 0;
}

void __no_inline_not_in_flash_func(end_of_line_isr)() {
    hw_clear_bits(&pio0->irq, 0x2);
    drive_timing();

    if (NEED_DISPLAY()) {
        // Output test pattern, this will be done by DMA.
        uint32_t delay = (timing_row & 0x1f);
        pio_sm_put(pio0, vga_red_sm, 0x40000000u | delay);
        pio_sm_put(pio0, vga_red_sm, 0xffffffffu);
        pio_sm_put(pio0, vga_red_sm, 0b11100001000010000100001000010000u);
        pio_sm_put(pio0, vga_red_sm, 0);
        delay |= 12 << 10;
        pio_sm_put(pio0, vga_green_sm, 0x40000000u | delay);
        pio_sm_put(pio0, vga_green_sm, 0xffffffffu);
        pio_sm_put(pio0, vga_green_sm, 0b11100001000010000100001000010000u);
        pio_sm_put(pio0, vga_green_sm, 0);
        delay |= 12 << 20;
        pio_sm_put(pio0, vga_blue_sm, 0x40000000u | delay);
        pio_sm_put(pio0, vga_blue_sm, 0xffffffffu);
        pio_sm_put(pio0, vga_blue_sm, 0b11100001000010000100001000010000u);
        pio_sm_put(pio0, vga_blue_sm, 0);
    }
    else if (timing_row == 0)
    {
        pio_sm_drain_tx_fifo(pio0, vga_red_sm);
        pio_sm_drain_tx_fifo(pio0, vga_green_sm);
        pio_sm_drain_tx_fifo(pio0, vga_blue_sm);
    }
}

// Setup must happen on core 1, to ensure interrupts are serviced fast enough.
void vga_entry() {
    uint offset = pio_add_program(pio0, &vga_channel_program);
    assert(offset == 0);
    vga_channel_program_init(pio0, vga_red_sm, 0);
    vga_channel_program_init(pio0, vga_green_sm, 6);
    vga_channel_program_init(pio0, vga_blue_sm, 11);
    offset = pio_add_program(pio0, &vga_timing_program);
    vga_timing_program_init(pio0, vga_timing_sm, offset, 16);

    hw_set_bits(&pio0->inte0, 0xf00);
    irq_set_exclusive_handler(PIO0_IRQ_0, end_of_line_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Prime the timing SM
    drive_timing();
    drive_timing();

    while (1);
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

    stdio_init_all();
    //sleep_ms(5000);
    puts("Hello, world!");

    logic_analyser_init(pio1, analyser_sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 96.f);
    
    uint32_t capture_buf[(CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32];
    logic_analyser_arm(pio1, analyser_sm, 0, capture_buf, 
            (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
            CAPTURE_PIN_BASE, true);

    multicore_launch_core1(vga_entry);

    dma_channel_wait_for_finish_blocking(0);
    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);

    absolute_time_t time = get_absolute_time();
    while (1) {
        printf("%d\n", timing_row);
        time = delayed_by_us(time, 100000);
        sleep_us(absolute_time_diff_us(get_absolute_time(), time));
    }

    return 0;
}
