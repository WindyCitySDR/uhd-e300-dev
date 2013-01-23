#include "b250_init.h"
#include "b250_defs.h"

#include <wb_utils.h>
#include <u3_net_stack.h>
#include <printf.h>

int main(void)
{
    b250_init();
    while(1)
    {
        //makes leds do something alive
        const uint32_t counter = wb_peek32(RB0_BASE + 0*4);
        wb_poke32(SET0_BASE + 0*4, counter/CPU_CLOCK);

        //run the serial loader - poll and handle
        b250_serial_loader_run1();

        //run the network stack - poll and handle
        u3_net_stack_handle_one();
    }
    return 0;
}
