// Copyright 2013 Ettus Research LLC

#include "x300_init.h"
#include "x300_defs.h"
#include "x300_fw_common.h"
#include "xge_phy.h"
#include "ethernet.h"
#include "chinch.h"
#include "mdelay.h"

#include <wb_utils.h>
#include <udp_uart.h>
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
    const eth_mac_addr_t *src_mac = u3_net_stack_get_mac_addr(ethno);
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_MAC_HI),
        (((uint32_t)src_mac->addr[0]) << 8) | (((uint32_t)src_mac->addr[1]) << 0));
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_MAC_LO),
        (((uint32_t)src_mac->addr[2]) << 24) | (((uint32_t)src_mac->addr[3]) << 16) |
        (((uint32_t)src_mac->addr[4]) << 8) | (((uint32_t)src_mac->addr[5]) << 0));
    wb_poke32(SR_ADDR(SET0_BASE, ethbase + ETH_FRAMER_SRC_IP_ADDR), u3_net_stack_get_ip_addr(ethno)->addr);
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
    const x300_fw_comms_t *request = (const x300_fw_comms_t *)buff;
    x300_fw_comms_t reply; memcpy(&reply, buff, sizeof(reply));

    //check for error and set error flag
    if (num_bytes < sizeof(x300_fw_comms_t))
    {
        reply.flags |= X300_FW_COMMS_FLAGS_ERROR;
    }

    //otherwise, run the actions set by the flags
    else
    {
        if (request->flags & X300_FW_COMMS_FLAGS_PEEK32)
        {
            if (request->addr & 0x00100000) {
                chinch_peek32(request->addr & 0x000FFFFF, &reply.data);
            } else {
                reply.data = wb_peek32(request->addr);
            }
        }

        if (request->flags & X300_FW_COMMS_FLAGS_POKE32)
        {
            if (request->addr & 0x00100000) {
                chinch_poke32(request->addr & 0x000FFFFF, request->data);
            } else {
                wb_poke32(request->addr, request->data);
            }
        }
    }

    //send a reply if ack requested
    if (request->flags & X300_FW_COMMS_FLAGS_ACK)
    {
        u3_net_stack_send_udp_pkt(ethno, dst, src, dst_port, src_port, &reply, sizeof(reply));
    }
}

void handle_udp_fpga_prog(
    const uint8_t ethno,
    const struct ip_addr *src, const struct ip_addr *dst,
    const uint16_t src_port, const uint16_t dst_port,
    const void *buff, const size_t num_bytes
)
{
    const x300_fpga_prog_t *request = (const x300_fpga_prog_t *)buff;
    x300_fpga_prog_flags_t reply = {0};
    bool status = true;

    if (num_bytes < offsetof(x300_fpga_prog_t, data)) {
        reply.flags |= X300_FPGA_PROG_FLAGS_ERROR;
    } else {
        if (request->flags & X300_FPGA_PROG_FLAGS_INIT) {
            STATUS_MERGE(chinch_flash_init(), status);
        } else if (request->flags & X300_FPGA_PROG_FLAGS_CLEANUP) {
            chinch_flash_cleanup();
        } else if (request->flags & X300_FPGA_PROG_CONFIGURE) {
            //This is a self-destructive operation and will most likely not return an ack.
            chinch_start_posc();
        } else if (request->flags & X300_FPGA_PROG_CONFIG_STATUS) {
            if (chinch_get_posc_status() != CHINCH_POSC_COMPLETED)
                reply.flags |= X300_FPGA_PROG_FLAGS_ERROR;
        } else {
            STATUS_MERGE(chinch_flash_select_sector(request->sector), status);
            if (request->flags & X300_FPGA_PROG_FLAGS_ERASE)
                STATUS_CHAIN(chinch_flash_erase_sector(), status);

            uint32_t num_buff_writes = (request->size / CHINCH_FLASH_MAX_BUF_WRITES) +
                                       (request->size % CHINCH_FLASH_MAX_BUF_WRITES == 0 ? 0 : 1);
            uint32_t data_idx = 0;
            for (uint32_t buf_wr_i = 0; (buf_wr_i < num_buff_writes) && status; buf_wr_i++) {
                uint32_t wr_len = (request->size - data_idx) >= CHINCH_FLASH_MAX_BUF_WRITES ?
                    CHINCH_FLASH_MAX_BUF_WRITES : (request->size - data_idx);

                STATUS_MERGE(chinch_flash_write_buf((request->index + data_idx)*2,
                    (uint16_t*)request->data+data_idx, wr_len), status);
                data_idx += wr_len;
            }

            if (request->flags & X300_FPGA_PROG_FLAGS_VERIFY) {
                uint16_t data[request->size];
                STATUS_MERGE(chinch_flash_read_buf(request->index*2, data, request->size), status);
                for (uint32_t i = 0; i < request->size; i++) {
                    status &= (data[i] == request->data[i]);
                }
            }
        }
    }
    if (!status) reply.flags |= X300_FPGA_PROG_FLAGS_ERROR;

    //send a reply if ack requested
    if (request->flags & X300_FPGA_PROG_FLAGS_ACK)
    {
        u3_net_stack_send_udp_pkt(ethno, dst, src, dst_port, src_port, &reply, sizeof(reply));
    }
}

void run_flash_access_test()
{
    printf("Running flash access test...\n");
    bool status = true, result = true;

    chinch_poke32(0x200, 0);

    STATUS_CHAIN(chinch_flash_init(), status);
    STATUS_CHAIN(chinch_flash_select_sector(156), status);
    STATUS_CHAIN(chinch_flash_erase_sector(), status);

    uint16_t wr_data[4] = {0xDEAD, 0xBEEF, 0x1234, 0x5678};
    uint16_t rd_data[4] = {0x0000, 0x0000, 0x0000, 0x0000};

    STATUS_CHAIN(chinch_flash_write(0x20, 0x0ACE), status);
    STATUS_CHAIN(chinch_flash_write(0x22, 0xBA5E), status);
    STATUS_CHAIN(chinch_flash_write_buf(0x0, wr_data, 4), status);

    STATUS_CHAIN(chinch_flash_read_buf(0x0, rd_data, 4), status);
    for (uint32_t i = 0; i < 4; i++) result &= (rd_data[i] == wr_data[i]);

    uint16_t data = 0;
    STATUS_CHAIN(chinch_flash_read(0x10, &data), status);
    result &= (data == 0xFFFF);
    STATUS_CHAIN(chinch_flash_read(0x12, &data), status);
    result &= (data == 0xFFFF);
    STATUS_CHAIN(chinch_flash_read(0x20, &data), status);
    result &= (data == 0x0ACE);
    STATUS_CHAIN(chinch_flash_read(0x22, &data), status);
    result &= (data == 0xBA5E);

    chinch_flash_cleanup();
    result &= status;

    chinch_poke32(0x200, result);

    printf("[Debug] Flash access test %s\n", result?"PASSED":"FAILED");
}

static uint32_t get_xbar_total(const uint8_t port)
{
    #define get_xbar_stat(in_prt, out_prt) \
        wb_peek32(RB0_BASE+256+(((in_prt)*8+(out_prt))*4))
    uint32_t total = 0;
    for (size_t i = 0; i < 8; i++)
    {
        total += get_xbar_stat(port, i);
    }
    for (size_t i = 0; i < 8; i++)
    {
        total += get_xbar_stat(i, port);
    }
    if (port < 2) //also netstack if applicable
    {
        total += u3_net_stack_get_stat_counts(port);
    }
    return total;
}

static size_t popcntll(uint64_t num)
{
    size_t total = 0;
    for (size_t i = 0; i < sizeof(num)*8; i++)
    {
        total += (num >> i) & 0x1;
    }
    return total;
}

static void update_leds(void)
{
    //update activity status for all ports
    uint64_t activity_shreg[8];
    for (size_t i = 0; i < 8; i++)
    {
        static uint32_t last_total[8];
        const uint32_t total = get_xbar_total(i);
        activity_shreg[i] <<= 1;
        activity_shreg[i] |= (total == last_total[i])? 0 : 1;
        last_total[i] = total;
    }

    static uint32_t counter = 0;
    counter++;

    const size_t cnt0 = popcntll(activity_shreg[0]);
    const size_t cnt1 = popcntll(activity_shreg[1]);
    const bool act0 = cnt0*8 > (counter % 64);
    const bool act1 = cnt1*8 > (counter % 64);
    const bool link0 = ethernet_get_link_up(0);
    const bool link1 = ethernet_get_link_up(1);

    wb_poke32(SET0_BASE + SR_LEDS*4, 0
        | (link0? LED_LINK2 : 0)
        | (link1? LED_LINK1 : 0)
        | (act0? LED_ACT2 : 0)
        | (act1? LED_ACT1 : 0)
        | ((act0 || act1)? LED_LINKACT : 0)
    );
}

static void garp(void)
{
    static size_t count = 0;
    if (count++ == 60000) //60 seconds
    {
        count = 0;
        for (size_t e = 0; e < ethernet_ninterfaces(); e++)
        {
            if (!ethernet_get_link_up(e)) continue;
            u3_net_stack_send_arp_request(e, u3_net_stack_get_ip_addr(e));
        }
    }
}

int main(void)
{
    x300_init();
    u3_net_stack_register_udp_handler(X300_FW_COMMS_UDP_PORT, &handle_udp_fw_comms);
    u3_net_stack_register_udp_handler(X300_VITA_UDP_PORT, &handle_udp_prog_framer);
    u3_net_stack_register_udp_handler(X300_FPGA_PROG_UDP_PORT, &handle_udp_fpga_prog);

    uint32_t last_cronjob = 0;

    while(true)
    {
        //jobs that happen once every ms
        const uint32_t ticks_now = wb_peek32(SR_ADDR(RB0_BASE, RB_COUNTER));
        const uint32_t ticks_passed = ticks_now - last_cronjob;
        static const uint32_t tick_delta = CPU_CLOCK/1000;
        if (ticks_passed > tick_delta)
        {
            update_leds(); //run the link and activity leds
            garp(); //send periodic garps
            xge_poll_sfpp_status(0); // Every so often poll XGE Phy to look for SFP+ hotplug events.
            xge_poll_sfpp_status(1); // Every so often poll XGE Phy to look for SFP+ hotplug events.
            last_cronjob = wb_peek32(SR_ADDR(RB0_BASE, RB_COUNTER));
        }

        //run the network stack - poll and handle
        u3_net_stack_handle_one();

        //run the PCIe listener - poll and fwd to wishbone
        forward_pcie_user_xact_to_wb();

        //run the udp uart handler for incoming serial data
        udp_uart_poll();
    }
    return 0;
}
