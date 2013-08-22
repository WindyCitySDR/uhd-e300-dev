#include "x300_init.h"
#include "x300_defs.h"
#include "ethernet.h"
#include "mdelay.h"
#include <wb_utils.h>
#include <wb_uart.h>
#include <wb_i2c.h>
#include <stdint.h>
#include <stdbool.h>
#include <printf.h>
#include <wb_pkt_iface64.h>
#include <u3_net_stack.h>
#include <udp_uart.h>
#include "x300_fw_common.h"

static wb_pkt_iface64_config_t pkt_config;

struct x300_eeprom_map
{
    //indentifying numbers
    unsigned char revision[2];
    unsigned char product[2];
    uint8_t _pad0[4];

    //all the mac addrs
    uint8_t mac_addr0[6];
    uint8_t _pad1[2];
    uint8_t mac_addr1[6];
    uint8_t _pad2[2];

    //all the IP addrs
    uint32_t gateway;
    uint32_t subnet[4];
    uint32_t ip_addr[4];
    uint8_t _pad3[16];
};

static struct x300_eeprom_map default_map = {
    .mac_addr0 = {0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff},
    .mac_addr1 = {0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33},
    .gateway = (192 << 24 | 168 << 16 | 10  << 8  | 1 << 0),
    .subnet = {
        (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0),
        (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0),
        (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0),
        (255 << 24 | 255 << 16 | 255  << 8  | 0 << 0),
    },
    .ip_addr = {
        (192 << 24 | 168 << 16 | 10  << 8  | 2 << 0),
        (192 << 24 | 168 << 16 | 20  << 8  | 2 << 0),
        (192 << 24 | 168 << 16 | 30  << 8  | 2 << 0),
        (192 << 24 | 168 << 16 | 40  << 8  | 2 << 0),
    },
};

const void *pick_inited_field(const void *eeprom, const void *def, const size_t len)
{
    bool all_ones = true;
    bool all_zeros = true;
    for (size_t i = 0; i < len; i++)
    {
        const uint8_t b = ((const uint8_t *)eeprom)[i];
        if (b != 0x00) all_zeros = false;
        if (b != 0xff) all_ones = false;
    }
    if (all_zeros) return def;
    if (all_ones) return def;
    return eeprom;
}

static void init_network(void)
{
    pkt_config = wb_pkt_iface64_init(PKT_RAM0_BASE, 0x1ffc);
    printf("PKT RAM0 BASE %u\n", (&pkt_config)->base);
    u3_net_stack_init(&pkt_config);

    //read everything from eeprom
    static const uint8_t eeprom_cmd[2] = {0, 0}; //the command is 16 bits of address offset
    struct x300_eeprom_map eeprom_map = default_map;
    wb_i2c_write(I2C1_BASE, MBOARD_EEPROM_ADDR, eeprom_cmd, 2);
    wb_i2c_read(I2C1_BASE, MBOARD_EEPROM_ADDR, (uint8_t *)(&eeprom_map), sizeof(eeprom_map));

    //determine interface number
    const size_t eth0no = wb_peek32(SR_ADDR(RB0_BASE, RB_ETH_TYPE0))? 2 : 0;
    const size_t eth1no = wb_peek32(SR_ADDR(RB0_BASE, RB_ETH_TYPE1))? 3 : 1;

    //pick the address from eeprom or default
    const eth_mac_addr_t *my_mac0 = (const eth_mac_addr_t *)pick_inited_field(&eeprom_map.mac_addr0, &default_map.mac_addr0, 6);
    const eth_mac_addr_t *my_mac1 = (const eth_mac_addr_t *)pick_inited_field(&eeprom_map.mac_addr1, &default_map.mac_addr1, 6);
    const struct ip_addr *my_ip0 = (const struct ip_addr *)pick_inited_field(&eeprom_map.ip_addr[eth0no], &default_map.ip_addr[eth0no], 4);
    const struct ip_addr *subnet0 = (const struct ip_addr *)pick_inited_field(&eeprom_map.subnet[eth0no], &default_map.subnet[eth0no], 4);
    const struct ip_addr *my_ip1 = (const struct ip_addr *)pick_inited_field(&eeprom_map.ip_addr[eth1no], &default_map.ip_addr[eth1no], 4);
    const struct ip_addr *subnet1 = (const struct ip_addr *)pick_inited_field(&eeprom_map.subnet[eth1no], &default_map.subnet[eth1no], 4);

    //init eth0
    u3_net_stack_init_eth(0, my_mac0, my_ip0, subnet0);
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 0), (my_mac0->addr[5] << 0) | (my_mac0->addr[4] << 8) | (my_mac0->addr[3] << 16) | (my_mac0->addr[2] << 24));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 1), (my_mac0->addr[1] << 0) | (my_mac0->addr[0] << 8));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 2), my_ip0->addr);

    //init eth1
    u3_net_stack_init_eth(1, my_mac1, my_ip1, subnet1);
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 0), (my_mac1->addr[5] << 0) | (my_mac1->addr[4] << 8) | (my_mac1->addr[3] << 16) | (my_mac1->addr[2] << 24));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 1), (my_mac1->addr[1] << 0) | (my_mac1->addr[0] << 8));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 2), my_ip1->addr);
}

static void putc(void *p, char c)
{
    wb_uart_putc(UART1_BASE, c);
}

void x300_init(void)
{
    //first - uart
    wb_uart_init(UART0_BASE, CPU_CLOCK/UART0_BAUD);
    wb_uart_init(UART1_BASE, CPU_CLOCK/UART1_BAUD);
    init_printf(NULL,putc);
    //udp_uart_init(UART0_BASE, X300_GPSDO_UDP_PORT);

    //now we can init the rest with prints
    printf("B250 ZPU Init Begin -- CPU CLOCK is %d MHz\n", CPU_CLOCK/1000000);

    //i2c rate init
    wb_i2c_init(I2C0_BASE, CPU_CLOCK);
    wb_i2c_init(I2C1_BASE, CPU_CLOCK);
    wb_i2c_init(I2C2_BASE, CPU_CLOCK);

    //hold phy in reset
    wb_poke32(SR_ADDR(SET0_BASE, SR_SW_RST), SW_RST_PHY);

    printf("DEBUG: eth0 is %2dG\n",(wb_peek32(SR_ADDR(RB0_BASE, RB_ETH_TYPE0))==1) ? 10 : 1);
    printf("DEBUG: eth1 is %2dG\n",(wb_peek32(SR_ADDR(RB0_BASE, RB_ETH_TYPE1))==1) ? 10 : 1);

    //setup net stack and eth state machines
    init_network();

    //phy reset release
    wb_poke32(SR_ADDR(SET0_BASE, SR_SW_RST), 0);

    // For eth interfaces, initialize the PHY's
    mdelay(100);
    xge_ethernet_init(0);
    xge_ethernet_init(1);
}
