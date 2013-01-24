#include "b250_init.h"
#include "b250_defs.h"

#include <wb_utils.h>
#include <u3_net_stack.h>
#include <printf.h>

void handle_udp_ctrl(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t src_port, const uint16_t dst_port,
    const void *buff, const size_t num_bytes
)
{
    printf("handle_udp_ctrl %u bytes\n", num_bytes);
    const uint32_t *cmd = (const uint32_t *)buff;
    const uint32_t addr = cmd[0];
    const uint32_t data = cmd[1];
    uint32_t reply[2];
    reply[0] = addr;
    reply[1] = 0;
    if (num_bytes > 4)
    {
        printf("udp poke 0x%x = 0x%x\n", addr, data);
        wb_poke32(addr, data);
    }
    else
    {
        reply[1] = wb_peek32(addr);
        printf("udp peek 0x%x = 0x%x\n", addr, reply[1]);
    }

    u3_net_stack_send_udp_pkt(ethno, dst, src, dst_port, src_port, &reply, sizeof(reply));
}

int main(void)
{
    b250_init();
    u3_net_stack_register_udp_handler(12345, &handle_udp_ctrl);
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
