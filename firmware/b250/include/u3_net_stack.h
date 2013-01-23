
// Copyright 2012 Ettus Research LLC

#ifndef INCLUDED_U3_NET_STACK_H
#define INCLUDED_U3_NET_STACK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <wb_pkt_iface64.h>

//----------------------------------------------------------------------

#include <lwip/ip_addr.h>
#include <lwip/ip.h>
#include <lwip/udp.h>
#include <lwip/icmp.h>
#include <if_arp.h>
#include <ethertype.h>

typedef struct
{
    uint8_t addr[6];
} eth_mac_addr_t;

typedef struct
{
    uint8_t ethno;
    uint8_t pad[5];
    eth_mac_addr_t dst;
    eth_mac_addr_t src;
    uint16_t ethertype;
} padded_eth_hdr_t;

//----------------------------------------------------------------------

void u3_net_stack_init(wb_pkt_iface64_config_t *config);

void u3_net_stack_init_eth(const uint8_t ethno, const eth_mac_addr_t *mac, const struct ip_addr *ip);

const struct ip_addr *u3_net_stack_get_ip_addr(const uint8_t ethno);

const eth_mac_addr_t *u3_net_stack_get_mac_addr(const uint8_t ethno);

void u3_net_stack_handle_one(void);

void u3_net_stack_arp_cache_update(const struct ip_addr *ip_addr, const eth_mac_addr_t * mac_addr);

const eth_mac_addr_t *u3_net_stack_arp_cache_lookup(const struct ip_addr *ip_addr);

#endif /* INCLUDED_U3_NET_STACK_H */
