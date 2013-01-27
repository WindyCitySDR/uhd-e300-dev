// Copyright 2013 Ettus Research LLC

#include "b250_init.h"
#include "b250_defs.h"
#include "b250_fw_common.h"

#include <wb_utils.h>
#include <u3_net_stack.h>
#include <printf.h>
#include <string.h>

void handle_udp_fw_comms(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t src_port, const uint16_t dst_port,
    const void *buff, const size_t num_bytes
)
{
    const b250_fw_comms_t *request = (const b250_fw_comms_t *)buff;
    b250_fw_comms_t reply; memcpy(&reply, buff, sizeof(reply));

    //check for error and set error flag
    if (num_bytes < sizeof(b250_fw_comms_t))
    {
        reply.flags |= B250_FW_COMMS_FLAGS_ERROR;
    }

    //otherwise, run the actions set by the flags
    else
    {
        if (request->flags & B250_FW_COMMS_FLAGS_PEEK32)
        {
            reply.data = wb_peek32(request->addr);
        }

        if (request->flags & B250_FW_COMMS_FLAGS_POKE32)
        {
            wb_poke32(request->addr, request->data);
        }
    }

    //send a reply if ack requested
    if (request->flags & B250_FW_COMMS_FLAGS_ACK)
    {
        u3_net_stack_send_udp_pkt(ethno, dst, src, dst_port, src_port, &reply, sizeof(reply));
    }
}

int main(void)
{
    b250_init();
    u3_net_stack_register_udp_handler(B250_FW_COMMS_UDP_PORT, &handle_udp_fw_comms);
    while(true)
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
