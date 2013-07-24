
// Copyright 2012 Ettus Research LLC

#include <u3_net_stack.h>
#include <string.h> //memcmp
#include <printf.h>

typedef struct
{
    padded_eth_hdr_t eth;
    struct arp_eth_ipv4 arp;
} padded_arp_t;

typedef struct
{
    padded_eth_hdr_t eth;
    struct ip_hdr ip;
    struct icmp_echo_hdr icmp;
} padded_icmp_t;

typedef struct
{
    padded_eth_hdr_t eth;
    struct ip_hdr ip;
    struct udp_hdr udp;
} padded_udp_t;

/***********************************************************************
 * 16-bit one's complement sum
 **********************************************************************/
static uint32_t chksum_buffer(
    uint16_t *buf, size_t nshorts,
    uint32_t initial_chksum
){
    uint32_t chksum = initial_chksum;
    for (size_t i = 0; i < nshorts; i++) chksum += buf[i];

    while (chksum >> 16) chksum = (chksum & 0xffff) + (chksum >> 16);

    return chksum;
}

/***********************************************************************
 * ARP Cache implementation
 **********************************************************************/
#define ARP_CACHE_NENTRIES 32

static size_t arp_cache_wr_index;

static struct ip_addr arp_cache_ips[ARP_CACHE_NENTRIES];
static eth_mac_addr_t arp_cache_macs[ARP_CACHE_NENTRIES];

void u3_net_stack_arp_cache_update(const struct ip_addr *ip_addr, const eth_mac_addr_t *mac_addr)
{
    for (size_t i = 0; i < ARP_CACHE_NENTRIES; i++)
    {
        if (memcmp(ip_addr, arp_cache_ips+i, sizeof(struct ip_addr)) == 0)
        {
            memcpy(arp_cache_macs+i, mac_addr, sizeof(eth_mac_addr_t));
            return;
        }
    }
    if (arp_cache_wr_index >= ARP_CACHE_NENTRIES) arp_cache_wr_index = 0;
    memcpy(arp_cache_ips+arp_cache_wr_index, ip_addr, sizeof(struct ip_addr));
    memcpy(arp_cache_macs+arp_cache_wr_index, mac_addr, sizeof(eth_mac_addr_t));
    arp_cache_wr_index++;
}

const eth_mac_addr_t *u3_net_stack_arp_cache_lookup(const struct ip_addr *ip_addr)
{
    for (size_t i = 0; i < ARP_CACHE_NENTRIES; i++)
    {
        if (memcmp(ip_addr, arp_cache_ips+i, sizeof(struct ip_addr)) == 0)
        {
            return &arp_cache_macs[i];
        }
    }
    return NULL;
}


/***********************************************************************
 * Net stack config
 **********************************************************************/
static wb_pkt_iface64_config_t *pkt_iface_config = NULL;

void u3_net_stack_init(wb_pkt_iface64_config_t *config)
{
    pkt_iface_config = config;
}

#define MAX_NETHS 4
static struct ip_addr net_conf_ips[MAX_NETHS];
static eth_mac_addr_t net_conf_macs[MAX_NETHS];
static eth_mac_addr_t net_conf_subnets[MAX_NETHS];

void u3_net_stack_init_eth(
    const uint8_t ethno,
    const eth_mac_addr_t *mac,
    const struct ip_addr *ip,
    const struct ip_addr *subnet
)
{
    memcpy(&net_conf_macs[ethno], mac, sizeof(eth_mac_addr_t));
    memcpy(&net_conf_ips[ethno], ip, sizeof(struct ip_addr));
    memcpy(&net_conf_subnets[ethno], subnet, sizeof(struct ip_addr));
}

const struct ip_addr *u3_net_stack_get_ip_addr(const uint8_t ethno)
{
    return &net_conf_ips[ethno];
}

const eth_mac_addr_t *u3_net_stack_get_mac_addr(const uint8_t ethno)
{
    return &net_conf_macs[ethno];
}

/***********************************************************************
 * Ethernet handlers - send packet w/ payload
 **********************************************************************/
static void send_eth_pkt(
    const void *p0, const size_t l0,
    const void *p1, const size_t l1,
    const void *p2, const size_t l2
)
{
    void *ptr = wb_pkt_iface64_tx_claim(pkt_iface_config);
    size_t buff_i = 0;

    uint32_t *buff32 = (uint32_t *)ptr;
    for (size_t i = 0; i < (l0+3)/4; i++)
    {
        buff32[buff_i++] = ((const uint32_t *)p0)[i];
    }
    for (size_t i = 0; i < (l1+3)/4; i++)
    {
        buff32[buff_i++] = ((const uint32_t *)p1)[i];
    }
    for (size_t i = 0; i < (l2+3)/4; i++)
    {
        buff32[buff_i++] = ((const uint32_t *)p2)[i];
    }

    // Fixes issue where we don't write an even number of 32bit words leaving last data stranded in H/W.
    if ((buff_i%2) == 1) buff32[buff_i++] = 0;

    wb_pkt_iface64_tx_submit(pkt_iface_config, l0 + l1 + l2);
}

/***********************************************************************
 * ARP handlers
 **********************************************************************/
static void send_arp_reply(
    const uint8_t ethno,
    const struct arp_eth_ipv4 *req,
    const eth_mac_addr_t *our_mac
){
    padded_arp_t reply;
    reply.eth.ethno = ethno;
    memcpy(&reply.eth.dst, (eth_mac_addr_t *)req->ar_sha, sizeof(eth_mac_addr_t));
    memcpy(&reply.eth.src, u3_net_stack_get_mac_addr(ethno), sizeof(eth_mac_addr_t));
    reply.eth.ethertype = ETHERTYPE_ARP;

    reply.arp.ar_hrd = req->ar_hrd;
    reply.arp.ar_pro = req->ar_pro;
    reply.arp.ar_hln = req->ar_hln;
    reply.arp.ar_pln = req->ar_pln;
    reply.arp.ar_op = ARPOP_REPLY;
    memcpy(reply.arp.ar_sha, our_mac,     sizeof(eth_mac_addr_t));
    memcpy(reply.arp.ar_sip, req->ar_tip, sizeof(struct ip_addr));
    memcpy(reply.arp.ar_tha, req->ar_sha, sizeof(eth_mac_addr_t));
    memcpy(reply.arp.ar_tip, req->ar_sip, sizeof(struct ip_addr));

    send_eth_pkt(&reply, sizeof(reply), NULL, 0, NULL, 0);
}

void u3_net_stack_send_arp_request(const uint8_t ethno, const struct ip_addr *addr)
{
    padded_arp_t req;
    req.eth.ethno = ethno;
    memset(&req.eth.dst, 0xff, sizeof(eth_mac_addr_t)); //bcast
    memcpy(&req.eth.src, u3_net_stack_get_mac_addr(ethno), sizeof(eth_mac_addr_t));
    req.eth.ethertype = ETHERTYPE_ARP;

    req.arp.ar_hrd = ARPHRD_ETHER;
    req.arp.ar_pro = ETHERTYPE_IPV4;
    req.arp.ar_hln = sizeof(eth_mac_addr_t);
    req.arp.ar_pln = sizeof(struct ip_addr);
    req.arp.ar_op = ARPOP_REQUEST;
    memcpy(req.arp.ar_sha, u3_net_stack_get_mac_addr(ethno), sizeof(eth_mac_addr_t));
    memcpy(req.arp.ar_sip, u3_net_stack_get_ip_addr(ethno), sizeof(struct ip_addr));
    memset(req.arp.ar_tha, 0x00, sizeof(eth_mac_addr_t));
    memcpy(req.arp.ar_tip, addr, sizeof(struct ip_addr));

    send_eth_pkt(&req, sizeof(req), NULL, 0, NULL, 0);
}

static void handle_arp_packet(const uint8_t ethno, const struct arp_eth_ipv4 *p)
{
    //printf("handle_arp_packet\n");
    if (p->ar_hrd != ARPHRD_ETHER
      || p->ar_pro != ETHERTYPE_IPV4
      || p->ar_hln != sizeof(eth_mac_addr_t)
      || p->ar_pln != sizeof(struct ip_addr))
    return;

    if (p->ar_op == ARPOP_REPLY)
    {
        //printf("ARPOP_REPLY\n");
        struct ip_addr ip_addr;
        memcpy(&ip_addr, p->ar_sip, sizeof(ip_addr));
        eth_mac_addr_t mac_addr;
        memcpy(&mac_addr, p->ar_sha, sizeof(mac_addr));
        u3_net_stack_arp_cache_update(&ip_addr, &mac_addr);
    }

    if (p->ar_op == ARPOP_REQUEST)
    {
        //printf("ARPOP_REQUEST\n");
        if (memcmp(p->ar_tip, u3_net_stack_get_ip_addr(ethno), sizeof(struct ip_addr)) == 0)
        {
            send_arp_reply(ethno, p, u3_net_stack_get_mac_addr(ethno));
        }
    }
}

/***********************************************************************
 * UDP handlers
 **********************************************************************/
#define UDP_NHANDLERS 16

static uint16_t udp_handler_ports[UDP_NHANDLERS];
static u3_net_stack_udp_handler_t udp_handlers[UDP_NHANDLERS];
static size_t udp_handlers_index = 0;

void u3_net_stack_register_udp_handler(
    const uint16_t port,
    const u3_net_stack_udp_handler_t handler
)
{
    if (udp_handlers_index < UDP_NHANDLERS)
    {
        udp_handler_ports[udp_handlers_index] = port;
        udp_handlers[udp_handlers_index] = handler;
        udp_handlers_index++;
    }
}

void u3_net_stack_send_udp_pkt(
    const uint8_t ethno,
    const struct ip_addr *src,
    const struct ip_addr *dst,
    const uint16_t src_port,
    const uint16_t dst_port,
    const void *buff,
    const size_t num_bytes
)
{
    const eth_mac_addr_t *dst_mac_addr = u3_net_stack_arp_cache_lookup(dst);
    if (dst_mac_addr == NULL)
    {
        printf("u3_net_stack_send_udp_pkt arp_cache_lookup fail\n");
        return;
    }

    padded_udp_t reply;

    reply.eth.ethno = ethno;
    memcpy(&reply.eth.dst, dst_mac_addr,                     sizeof(eth_mac_addr_t));
    memcpy(&reply.eth.src, u3_net_stack_get_mac_addr(ethno), sizeof(eth_mac_addr_t));
    reply.eth.ethertype = ETHERTYPE_IPV4;

    IPH_VHLTOS_SET(&reply.ip, 4, 5, 0);
    IPH_LEN_SET(&reply.ip, IP_HLEN + UDP_HLEN + num_bytes);
    IPH_ID_SET(&reply.ip, 0);
    IPH_OFFSET_SET(&reply.ip, IP_DF);	/* don't fragment */
    IPH_TTL_SET(&reply.ip, 32);
    IPH_PROTO_SET(&reply.ip, IP_PROTO_UDP);
    IPH_CHKSUM_SET(&reply.ip, 0);
    memcpy(&reply.ip.src, src, sizeof(struct ip_addr));
    memcpy(&reply.ip.dest, dst, sizeof(struct ip_addr));

    IPH_CHKSUM_SET(&reply.ip, ~chksum_buffer(
        (unsigned short *) &reply.ip, sizeof(reply.ip)/sizeof(short), 0
    ));

    reply.udp.src = src_port;
    reply.udp.dest = dst_port;
    reply.udp.len = UDP_HLEN + num_bytes;
    reply.udp.chksum = 0;

    send_eth_pkt(&reply, sizeof(reply), buff, num_bytes, NULL, 0);
}

static void handle_udp_packet(
    const uint8_t ethno,
    const struct ip_addr *src,
    const struct ip_addr *dst,
    const struct udp_hdr *udp,
    const size_t num_bytes
){
    for (size_t i = 0; i < udp_handlers_index; i++)
    {
        if (udp_handler_ports[i] == udp->dest)
        {
            udp_handlers[i](
                ethno, src, u3_net_stack_get_ip_addr(ethno), udp->src, udp->dest,
                ((const uint8_t *)udp) + sizeof(struct udp_hdr),
                num_bytes - UDP_HLEN
            );
            return;
        }
    }
    printf("Unhandled UDP packet src=%u, dest=%u\n", udp->src, udp->dest);
    //TODO send destination unreachable
}

/***********************************************************************
 * ICMP handlers
 **********************************************************************/
static void handle_icmp_packet(
    const uint8_t ethno,
    const struct ip_addr *src,
    const struct ip_addr *dst,
    const struct icmp_echo_hdr *icmp,
    const size_t num_bytes
){
    //printf("handle_icmp_packet\n");
    if (icmp->type == ICMP_ECHO)
    {
        const void *icmp_data_buff = ((uint8_t*)icmp) + sizeof(struct icmp_echo_hdr);
        const size_t icmp_data_len = num_bytes - sizeof(struct icmp_echo_hdr);

        const eth_mac_addr_t *dst_mac_addr = u3_net_stack_arp_cache_lookup(src);
        if (dst_mac_addr == NULL)
        {
            printf("handle_icmp_packet arp_cache_lookup fail\n");
            return;
        }

        padded_icmp_t reply;

        reply.eth.ethno = ethno;
        memcpy(&reply.eth.dst, dst_mac_addr,                     sizeof(eth_mac_addr_t));
        memcpy(&reply.eth.src, u3_net_stack_get_mac_addr(ethno), sizeof(eth_mac_addr_t));
        reply.eth.ethertype = ETHERTYPE_IPV4;

        IPH_VHLTOS_SET(&reply.ip, 4, 5, 0);
        IPH_LEN_SET(&reply.ip, IP_HLEN + sizeof(reply.icmp) + icmp_data_len);
        IPH_ID_SET(&reply.ip, 0);
        IPH_OFFSET_SET(&reply.ip, IP_DF);	/* don't fragment */
        IPH_TTL_SET(&reply.ip, 32);
        IPH_PROTO_SET(&reply.ip, IP_PROTO_ICMP);
        IPH_CHKSUM_SET(&reply.ip, 0);
        memcpy(&reply.ip.src, dst, sizeof(struct ip_addr));
        memcpy(&reply.ip.dest, src, sizeof(struct ip_addr));

        IPH_CHKSUM_SET(&reply.ip, ~chksum_buffer(
            (unsigned short *) &reply.ip, sizeof(reply.ip)/sizeof(short), 0
        ));

        reply.icmp.type = 0;
        reply.icmp.code = 0;
        reply.icmp.chksum = 0;
        reply.icmp.id = icmp->id;
        reply.icmp.seqno = icmp->seqno;
        reply.icmp.chksum = ~chksum_buffer( //data checksum
            (unsigned short *)icmp_data_buff,
            icmp_data_len/sizeof(short),
            chksum_buffer(                  //header checksum
                (unsigned short *)&reply.icmp,
                sizeof(reply.icmp)/sizeof(short),
            0)
        );

        send_eth_pkt(&reply, sizeof(reply), icmp_data_buff, icmp_data_len, NULL, 0);
    }

    if (icmp->type == ICMP_DUR && icmp->code == ICMP_DUR_PORT)
    {
        struct ip_hdr *ip = (struct ip_hdr *)(((uint8_t*)icmp) + sizeof(struct icmp_echo_hdr));
        struct udp_hdr *udp = (struct udp_hdr *)(((char *)ip) + IP_HLEN);
        if (IPH_PROTO(ip) != IP_PROTO_UDP) return;
        for (size_t i = 0; i < udp_handlers_index; i++)
        {
            if (udp_handler_ports[i] == udp->src)
            {
                udp_handlers[i](ethno,
                    src, u3_net_stack_get_ip_addr(ethno),
                    udp->src, udp->dest, NULL, 0
                );
                return;
            }
        }
    }
}

/***********************************************************************
 * Ethernet handlers
 **********************************************************************/
static void handle_eth_packet(const void *buff, const size_t num_bytes)
{
    const padded_eth_hdr_t *eth_hdr = (padded_eth_hdr_t *)buff;
    const uint8_t *eth_body = ((const uint8_t *)buff) + sizeof(padded_eth_hdr_t);
    //printf("handle_eth_packet got ethertype 0x%x\n", (unsigned)eth_hdr->ethertype);

    if (eth_hdr->ethertype == ETHERTYPE_ARP)
    {
        //printf("eth_hdr->ethertype == ETHERTYPE_ARP\n");
        const struct arp_eth_ipv4 *arp = (const struct arp_eth_ipv4 *)eth_body;
        handle_arp_packet(eth_hdr->ethno, arp);
    }
    else if (eth_hdr->ethertype == ETHERTYPE_IPV4)
    {
        //printf("eth_hdr->ethertype == ETHERTYPE_IPV4\n");
        const struct ip_hdr *ip = (const struct ip_hdr *)eth_body;
        const uint8_t *ip_body = eth_body + IP_HLEN;

        if (IPH_V(ip) != 4 || IPH_HL(ip) != 5) return;// ignore pkts w/ bad version or options
        if (IPH_OFFSET(ip) & (IP_MF | IP_OFFMASK)) return;// ignore fragmented packets

        u3_net_stack_arp_cache_update(&ip->src, &eth_hdr->src);

        if (IPH_PROTO(ip) == IP_PROTO_UDP)
        {
            handle_udp_packet(
                eth_hdr->ethno, &ip->src, &ip->dest,
                (const struct udp_hdr *)ip_body,
                IPH_LEN(ip) - IP_HLEN
            );
        }

        if (IPH_PROTO(ip) == IP_PROTO_ICMP)
        {
            handle_icmp_packet(
                eth_hdr->ethno, &ip->src, &ip->dest,
                (const struct icmp_echo_hdr *)ip_body,
                IPH_LEN(ip) - IP_HLEN
            );
        }
    }
    else return;	// Not ARP or IPV4, ignore
}

void u3_net_stack_handle_one(void)
{
    size_t num_bytes = 0;
    const void *ptr = wb_pkt_iface64_rx_try_claim(pkt_iface_config, &num_bytes);
    if (ptr != NULL)
    {
        //printf("u3_net_stack_handle_one got %u bytes\n", (unsigned)num_bytes);
        handle_eth_packet(ptr, num_bytes);
        wb_pkt_iface64_rx_release(pkt_iface_config);
    }
}
