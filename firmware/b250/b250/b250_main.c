// Copyright 2013 Ettus Research LLC

#include "b250_init.h"
#include "b250_defs.h"
#include "b250_fw_common.h"

#include <wb_utils.h>
#include <u3_net_stack.h>
#include <printf.h>
#include <string.h>

#define ETH_FRAMER_SRC_MAC_HI 0
#define ETH_FRAMER_SRC_MAC_LO 1
#define ETH_FRAMER_SRC_IP_ADDR 2
#define ETH_FRAMER_SRC_UDP_PORT 3
#define ETH_FRAMER_DST_RAM_ADDR 4
#define ETH_FRAMER_DST_IP_ADDR 5
#define ETH_FRAMER_DST_UDP_MAC 6
#define ETH_FRAMER_DST_MAC_LO 7

void handle_udp_prog_framer(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t src_port, const uint16_t dst_port,
    const void *buff, const size_t num_bytes
)
{
    const size_t ethbase = (ethno == 0)? SR_ETHINT0 : SR_ETHINT1;
    const uint32_t sid = ((const uint32_t *)buff)[0];
    const size_t vdest = (sid >> 16) & 0xff;
    printf("handle_udp_prog_framer sid %u vdest %u\n", sid, vdest);

    //setup source framer
    const eth_mac_addr_t *src_mac = u3_net_stack_get_mac_addr(0);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_MAC_HI),
        (((uint32_t)src_mac->addr[0]) << 8) | (((uint32_t)src_mac->addr[1]) << 0));
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_MAC_LO),
        (((uint32_t)src_mac->addr[2]) << 24) | (((uint32_t)src_mac->addr[3]) << 16) |
        (((uint32_t)src_mac->addr[4]) << 8) | (((uint32_t)src_mac->addr[5]) << 0));
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_IP_ADDR), u3_net_stack_get_ip_addr(0)->addr);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_UDP_PORT), dst_port);

    //setup destination framer
    const eth_mac_addr_t *dst_mac = u3_net_stack_arp_cache_lookup(src);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_DST_RAM_ADDR), vdest);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_DST_IP_ADDR), src->addr);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_DST_UDP_MAC),
        (((uint32_t)src_port) << 16) |
        (((uint32_t)dst_mac->addr[0]) << 8) | (((uint32_t)dst_mac->addr[1]) << 0));
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_DST_MAC_LO),
        (((uint32_t)dst_mac->addr[2]) << 24) | (((uint32_t)dst_mac->addr[3]) << 16) |
        (((uint32_t)dst_mac->addr[4]) << 8) | (((uint32_t)dst_mac->addr[5]) << 0));
}

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
    u3_net_stack_register_udp_handler(B250_VITA_UDP_PORT, &handle_udp_prog_framer);
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
