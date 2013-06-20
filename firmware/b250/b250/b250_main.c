// Copyright 2013 Ettus Research LLC

#include "b250_init.h"
#include "b250_defs.h"
#include "b250_fw_common.h"
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
            if (request->addr & 0x00100000) {
                chinch_peek32(request->addr & 0x000FFFFF, &reply.data);
            } else {
                reply.data = wb_peek32(request->addr);
            }
        }

        if (request->flags & B250_FW_COMMS_FLAGS_POKE32)
        {
            if (request->addr & 0x00100000) {
                chinch_poke32(request->addr & 0x000FFFFF, request->data);
            } else {
                wb_poke32(request->addr, request->data);
            }
        }
    }

    //send a reply if ack requested
    if (request->flags & B250_FW_COMMS_FLAGS_ACK)
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
    const b250_fpga_prog_t *request = (const b250_fpga_prog_t *)buff;
    b250_fpga_prog_flags_t reply = {0};
    bool status = true;

    if (num_bytes < offsetof(b250_fpga_prog_t, data)) {
        reply.flags |= B250_FPGA_PROG_FLAGS_ERROR;
    } else {
        if (request->flags & B250_FPGA_PROG_FLAGS_INIT) {
            STATUS_MERGE(chinch_flash_init(), status);
        } else if (request->flags & B250_FPGA_PROG_FLAGS_CLEANUP) {
            chinch_flash_cleanup();
        } else if (request->flags & B250_FPGA_PROG_CONFIGURE) {
            //This is a self-destructive operation and will most likely not return an ack.
            chinch_start_posc();
        } else if (request->flags & B250_FPGA_PROG_CONFIG_STATUS) {
            if (chinch_get_posc_status() != CHINCH_POSC_COMPLETED)
                reply.flags |= B250_FPGA_PROG_FLAGS_ERROR;
        } else {
            STATUS_MERGE(chinch_flash_select_sector(request->sector), status);
            if (request->flags & B250_FPGA_PROG_FLAGS_ERASE)
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

            if (request->flags & B250_FPGA_PROG_FLAGS_VERIFY) {
                uint16_t data[request->size];
                STATUS_MERGE(chinch_flash_read_buf(request->index*2, data, request->size), status);
                for (uint32_t i = 0; i < request->size; i++) {
                    status &= (data[i] == request->data[i]);
                }
            }
        }
    }
    if (!status) reply.flags |= B250_FPGA_PROG_FLAGS_ERROR;

    //send a reply if ack requested
    if (request->flags & B250_FPGA_PROG_FLAGS_ACK)
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

int main(void)
{
    uint32_t last_counter = 0;
    uint32_t xge_sfpp_hotplug_count = 0;

    b250_init();
    u3_net_stack_register_udp_handler(B250_FW_COMMS_UDP_PORT, &handle_udp_fw_comms);
    u3_net_stack_register_udp_handler(B250_VITA_UDP_PORT, &handle_udp_prog_framer);
    u3_net_stack_register_udp_handler(B250_FPGA_PROG_UDP_PORT, &handle_udp_fpga_prog);

//    run_flash_access_test();

    while(true)
    {
        //makes leds do something alive
        const uint32_t counter = wb_peek32(RB0_BASE + 0*4);
        wb_poke32(SET0_BASE + 0*4, counter/CPU_CLOCK);
        if (counter/CPU_CLOCK != last_counter/CPU_CLOCK) {
            last_counter = counter;
            wb_poke32(SET0_BASE + 5*4, counter/CPU_CLOCK);
            wb_poke32(SET0_BASE + 6*4, counter);
        }

        //run the network stack - poll and handle
        u3_net_stack_handle_one();

        //run the PCIe listener - poll and fwd to wishbone
        forward_pcie_user_xact_to_wb();

        //run the udp uart handler for incoming serial data
        udp_uart_poll();

        if ((xge_sfpp_hotplug_count++) == 1000) {
              // Every so often poll XGE Phy to look for SFP+ hotplug events.
              xge_sfpp_hotplug_count = 0;
          xge_poll_sfpp_status(0);

        }
    }
    return 0;
}
