// Copyright (C) 2021 Michael Bell

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/regs/busctrl.h"
#include "hardware/structs/ssi.h"
#include "hardware/vreg.h"

#include "vga.h"
#include "vga.pio.h"

#include "sdring.h"
#include "vgaaudio.h"

//#define RES_640_480_DOUBLE
//#define RES_720p_SINGLE
#define RES_720p_DOUBLE
//#define RES_1080p_SINGLE

#ifdef RES_640_480_DOUBLE
 // Timings for 640x480
#define TIMING_V_PULSE    2
#define TIMING_V_BACK    (30 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (480 + TIMING_V_BACK)
#define TIMING_V_FRONT   (9 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   (16 * 2)
#define TIMING_H_PULSE   (96 * 2)
#define TIMING_H_BACK    (48 * 2)
#define TIMING_H_DISPLAY (640 * 2)
#define CLOCK_VCO   1500
#define CLOCK_PD1   5
#define CLOCK_PD2   3
#define CLOCK_RATE  (100 * MHZ)
#define CHANNEL_CLK_DIV 2
#define VREG_VSEL VREG_VOLTAGE_1_10
#endif
#ifdef RES_720p_SINGLE
 // Timings for 720p
#define TIMING_V_PULSE    5
#define TIMING_V_BACK    (19 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (720 + TIMING_V_BACK)
#define TIMING_V_FRONT   (4 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   64
#define TIMING_H_PULSE   128
#define TIMING_H_BACK    192
#define TIMING_H_DISPLAY 1280
#define CLOCK_VCO   1044
#define CLOCK_PD1   7
#define CLOCK_PD2   1
#define CLOCK_RATE  149142857
#define CHANNEL_CLK_DIV 1
#define VREG_VSEL VREG_VOLTAGE_1_10
#endif
#ifdef RES_720p_DOUBLE
#if 1
// Timings for 720p at double sys clock
#define TIMING_V_PULSE    5
#define TIMING_V_BACK    (21 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (720 + TIMING_V_BACK)
#define TIMING_V_FRONT   (4 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   (64 * 2)
#define TIMING_H_PULSE   (130 * 2)
#define TIMING_H_BACK    (192 * 2)
#define TIMING_H_DISPLAY (1280 * 2)
//#define CLOCK_VCO   1488
#define CLOCK_VCO   1500
#define CLOCK_PD1   5
#define CLOCK_PD2   1
//#define CLOCK_RATE  297600 * KHZ
#define CLOCK_RATE  300000 * KHZ
#define CHANNEL_CLK_DIV 2
#define VREG_VSEL VREG_VOLTAGE_1_15
#else
#define TIMING_V_PULSE    5
#define TIMING_V_BACK    (12 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (720 + TIMING_V_BACK)
#define TIMING_V_FRONT   (4 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   (64 * 2)
#define TIMING_H_PULSE   (128 * 2)
#define TIMING_H_BACK    (192 * 2)
#define TIMING_H_DISPLAY (1280 * 2)
#define CLOCK_VCO   1140
#define CLOCK_PD1   4
#define CLOCK_PD2   1
#define CLOCK_RATE  285000 * KHZ
#define CHANNEL_CLK_DIV 2
#define VREG_VSEL VREG_VOLTAGE_1_15
#endif
#endif
#ifdef RES_1080p_SINGLE
 // Timings for 1080p
#define TIMING_V_PULSE    8
#define TIMING_V_BACK    (6 + TIMING_V_PULSE)
#define TIMING_V_DISPLAY (1080 + TIMING_V_BACK)
#define TIMING_V_FRONT   (17 + TIMING_V_DISPLAY)
#define TIMING_H_FRONT   8
#define TIMING_H_PULSE   32
#define TIMING_H_BACK    40
#define TIMING_H_DISPLAY 1920
#define CLOCK_VCO   1332
#define CLOCK_PD1   5
#define CLOCK_PD2   1
#define CLOCK_RATE  266400 * KHZ
#define CHANNEL_CLK_DIV 1
#define VREG_VSEL VREG_VOLTAGE_1_10
#endif


static uint16_t timing_row = 0;
static uint16_t timing_phase = 0;
static uint16_t eol_row = 0;

void __no_inline_not_in_flash_func(drive_timing)()
{
    while (!pio_sm_is_tx_fifo_full(vga_pio, vga_timing_sm)) {
        uint32_t instr;
        switch (timing_phase) {
            case 0:
                // Front Porch
                instr = 0x4000A042u;
                if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
                instr |= (TIMING_H_FRONT - 3) << 16;
                pio_sm_put(vga_pio, vga_timing_sm, instr);
                break;

            case 1:
                // HSYNC
                instr = 0x0000A042u;
                if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
                instr |= (TIMING_H_PULSE - 3) << 16;
                pio_sm_put(vga_pio, vga_timing_sm, instr);
                break;

            case 2:
                // Back Porch, trigger pixel channels if in display window
                instr = 0x4000C004u;
                if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
                if (timing_row >= TIMING_V_BACK && timing_row < TIMING_V_DISPLAY) instr |= 0xC004u;
                else instr |= 0xA042u;
                instr |= (TIMING_H_BACK - 3) << 16;
                pio_sm_put(vga_pio, vga_timing_sm, instr);
                break;

            case 3:
                // Display, trigger next line at end
                instr = 0x4000C000u;
                if (timing_row == TIMING_V_FRONT - 1) instr = 0x4000C001u;
                if (timing_row >= TIMING_V_PULSE) instr |= 0x80000000u;
                instr |= (TIMING_H_DISPLAY - 3) << 16;
                pio_sm_put(vga_pio, vga_timing_sm, instr);

                if (++timing_row >= TIMING_V_FRONT) timing_row = 0;
                break;
        }

        timing_phase = (timing_phase + 1) & 3;
    }
}

void __no_inline_not_in_flash_func(timing_isr)() {
    drive_timing();
}

void __no_inline_not_in_flash_func(end_of_line_isr)() {
    if (vga_pio->irq & 2) eol_row = 0;
    else eol_row++;
    hw_clear_bits(&vga_pio->irq, 0x3);

    if (eol_row == TIMING_V_DISPLAY || eol_row == TIMING_V_DISPLAY + 1)
    {
        display_end_frame();
    }
    else if (eol_row == (TIMING_V_DISPLAY + 2) % (TIMING_V_FRONT))
    {
        // Force SMs to get back into a good state:
        //       Disable
        //       Reset PC
        //       Clear OSR (it is read when SM is re-enabled)
        //       Drain FIFOs
        //       Re-enable
        vga_pio->ctrl = 0x008;
        for (uint sm = vga_red_sm; sm <= vga_blue_sm; ++sm)
        {
            pio_sm_drain_tx_fifo(vga_pio, sm);
            pio_sm_exec_wait_blocking(vga_pio, sm, pio_encode_jmp(vga_channel_offset_end));
            pio_sm_exec_wait_blocking(vga_pio, sm, pio_encode_mov(pio_osr, pio_null));
        }
        vga_pio->ctrl = 0x70f;
    }
    else if (eol_row == (TIMING_V_DISPLAY + 3) % (TIMING_V_FRONT))
    {
        display_start_new_frame();
    }
}

// Setup must happen on core 1, to ensure interrupts are serviced fast enough.
void vga_entry() {
    // Give core 1 priority access to the bus as it drives timing directly
    // and the shared PIO bus connection can get very busy with pixel data traffic
    //bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    uint offset = pio_add_program(vga_pio, &vga_channel_program);
    assert(offset == 0);
    vga_channel_program_init(vga_pio, vga_red_sm, 0, CHANNEL_CLK_DIV);
    vga_channel_program_init(vga_pio, vga_green_sm, 6, CHANNEL_CLK_DIV);
    vga_channel_program_init(vga_pio, vga_blue_sm, 11, CHANNEL_CLK_DIV);
    vga_pio->ctrl = 0x707;

    offset = pio_add_program(vga_pio, &vga_timing_program);
    vga_timing_program_init(vga_pio, vga_timing_sm, offset, 16);

    hw_set_bits(&vga_pio->inte0, 0xf00);
    irq_set_exclusive_handler(PIO0_IRQ_0, end_of_line_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    hw_set_bits(&vga_pio->inte1, 0x080);
    irq_set_exclusive_handler(PIO0_IRQ_1, timing_isr);
    irq_set_priority(PIO0_IRQ_1, 0xC0);
    irq_set_enabled(PIO0_IRQ_1, true);

    audio_init();

    // Notify setup is complete
    multicore_fifo_push_blocking(0);

    display_core1_loop();
}

int main()
{
    // Set required voltage
    vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);

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

    // Wait for setup complete then call into display code
    multicore_fifo_pop_blocking();
    display_loop();

    return 0;
}
