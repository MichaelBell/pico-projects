

#define vga_pio pio0
#define vga_red_sm    0 
#define vga_green_sm  1
#define vga_blue_sm   2
#define vga_timing_sm 3

#define vga_red_dma   7
#define vga_green_dma 6
#define vga_blue_dma  5

// Loop to run any main display code, called on core0 after setup.
void display_loop();

// These calls are made from timing critical code on core1,
// throw to core0 if anything interesting has to happen

// Call to user code to start transfers for next frame
void __no_inline_not_in_flash_func(display_start_new_frame)();

// Call to user to notify the next row to be displayed
void __no_inline_not_in_flash_func(display_next_row)(uint16_t row);
