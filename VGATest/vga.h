// Copyright (C) 2021 Michael Bell

#define vga_pio pio0
#define vga_red_sm    0 
#define vga_green_sm  1
#define vga_blue_sm   2
#define vga_timing_sm 3

// Loop to run any main display code, called on core0 after setup.
void display_loop();

// These calls are made from timing critical code on core1,
// throw to core0 if anything interesting has to happen

// Call to user code to start transfers for next frame
void display_start_new_frame();

// Call to user to notify that the frame has ended, and any running
// transfers should be aborted.
void display_end_frame();
