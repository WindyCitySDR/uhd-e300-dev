#include "b250_init.h"
#include "b250_defs.h"

#include <wb_utils.h>

int main(void)
{
    b250_init();
    while(1)
    {
        b250_serial_loader_run1();
        const uint32_t counter = wb_peek32(RB0_BASE + 0*4);
        wb_poke32(SET0_BASE + 0*4, counter/CPU_CLOCK);
    }
    return 0;
}
