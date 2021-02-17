#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "vga.pio.h"

const uint CAPTURE_PIN_BASE = 0;
const uint CAPTURE_PIN_COUNT = 8;
const uint CAPTURE_N_SAMPLES = 96;

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div);
void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level);
void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples);

int main()
{
    stdio_init_all();
    puts("Hello, world!");

    uint vga_channel_sm = 0;
    uint vga_trigger_sm = 1;
    uint analyser_sm = 3;

    uint offset = pio_add_program(pio0, &vga_channel_program);
    assert(offset == 0);
    vga_channel_program_init(pio0, vga_channel_sm, 0);
    offset = pio_add_program(pio0, &vga_trigger_program);
    vga_trigger_program_init(pio0, vga_trigger_sm, offset, 5);

    logic_analyser_init(pio0, analyser_sm, 0, CAPTURE_PIN_COUNT, 2.f);
    
    //                                 | |    |    |    |    |    |    |
    pio_sm_put(pio0, vga_channel_sm, 0b11000010000000000000000000000000u);
    pio_sm_put(pio0, vga_channel_sm, 0b11000100001000011011111111011000u);
    pio_sm_put(pio0, vga_channel_sm, 0b11110001110011100111001110001100u);
    pio_sm_put(pio0, vga_channel_sm, 0b01000000000100000000010000000110u);
    //pio_sm_put(pio0, vga_channel_sm, 0b01000010000000010000000010000000u);
    //pio_sm_put(pio0, vga_channel_sm, 0b01000010000100010000010010000001u);
    //pio_sm_put(pio0, vga_channel_sm, 0b11000010001100111011111111101010u);
    pio_sm_put(pio0, vga_channel_sm, 0);

    uint32_t capture_buf[(CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32];
    logic_analyser_arm(pio0, analyser_sm, 0, capture_buf, 
            (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
            CAPTURE_PIN_BASE, true);

    pio_sm_put(pio0, vga_trigger_sm, 0xC0000001u);

    dma_channel_wait_for_finish_blocking(0);
    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);

    return 0;
}
