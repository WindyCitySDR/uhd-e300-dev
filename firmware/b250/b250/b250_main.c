#include "b250_init.h"
#include "b250_defs.h"

#include <wb_utils.h>
#include <wb_pkt_iface64.h>
#include <printf.h>

int main(void)
{
    b250_init();
    void *ptr = NULL;
    wb_pkt_iface64_config_t config = wb_pkt_iface64_init(PKT_RAM0_BASE);
    while(1)
    {
        b250_serial_loader_run1();
        const uint32_t counter = wb_peek32(RB0_BASE + 0*4);
        wb_poke32(SET0_BASE + 0*4, counter/CPU_CLOCK);
        if (ptr == NULL)
        {
            size_t num_bytes = 0;
            ptr = wb_pkt_iface64_rx_try_claim(&config, &num_bytes);
            if (ptr != NULL) printf("Got packet %d bytes!\n", (int)num_bytes);
        }
    }
    return 0;
}
