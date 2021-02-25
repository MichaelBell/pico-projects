#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/interp.h"
#include "hardware/structs/systick.h"

// Evil from https://stackoverflow.com/questions/8551418/c-preprocessor-macro-for-returning-a-string-repeated-a-certain-number-of-times
#define REP0(X)
#define REP1(X) X
#define REP2(X) REP1(X) X
#define REP3(X) REP2(X) X
#define REP4(X) REP3(X) X
#define REP5(X) REP4(X) X
#define REP6(X) REP5(X) X
#define REP7(X) REP6(X) X
#define REP8(X) REP7(X) X
#define REP9(X) REP8(X) X
#define REP10(X) REP9(X) X

#define REP(HUNDREDS,TENS,ONES,X) \
  REP##HUNDREDS(REP10(REP10(X))) \
  REP##TENS(REP10(X)) \
  REP##ONES(X)

void core1_entry() {

    systick_hw->csr = 0b101;
    systick_hw->rvr = 0x10000;

    while (1) {
        systick_hw->cvr = 0;

        uint32_t count;
        asm (
            "movs r6, #0\n\t"
            "str r6, [%[st], #8]\n\t"

            REP(0,6,4,"ldr r6, [%[itp0], #20]\n\t")
            
            "ldr %[count], [%[st], #8]"
            : [count] "=l" (count) : [itp0] "l" (interp0), [st] "l" (systick_hw) : "r6");

        count = 0x10000 - count;
    
        multicore_fifo_push_blocking(count);

        sleep_ms(1000);
    }
}

int main()
{
    stdio_init_all();

    sleep_ms(3000);

    puts("Hello, world!");

    multicore_launch_core1(core1_entry);

    while (1) {
        uint32_t count = multicore_fifo_pop_blocking();

        printf("Systick count: %d\n", count);
    }

    return 0;
}
