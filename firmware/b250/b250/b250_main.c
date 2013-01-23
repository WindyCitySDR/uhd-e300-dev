#include "b250_init.h"
#include "b250_defs.h"

#include <wb_utils.h>
//#include <wb_pkt_iface64.h>
#include <u3_net_stack.h>
#include <printf.h>

int main(void)
{
    b250_init();
    //wb_pkt_iface64_config_t config = wb_pkt_iface64_init(PKT_RAM0_BASE, 0x1ffc);
    //printf("PKT RAM0 BASE %u\n", (&config)->base);
    //u3_net_stack_init(&config);
    while(1)
    {
        b250_serial_loader_run1();
        const uint32_t counter = wb_peek32(RB0_BASE + 0*4);
        wb_poke32(SET0_BASE + 0*4, counter/CPU_CLOCK);

/*
        size_t num_bytes = 0;
        const void *ptr = wb_pkt_iface64_rx_try_claim(&config, &num_bytes);

        if (ptr != NULL)
        {
            printf("Got %u \n", (unsigned)num_bytes);
            wb_pkt_iface64_tx_submit(&config, ptr, num_bytes);
            wb_pkt_iface64_rx_release(&config);
            //printf("ok - done\n");
            //while(1){}
        }
*/
        u3_net_stack_handle_one();
    }
    return 0;
}
