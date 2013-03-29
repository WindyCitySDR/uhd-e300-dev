#include "b250_init.h"
#include "b250_defs.h"
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
#include "b250_fw_common.h"



static wb_pkt_iface64_config_t pkt_config;

static void init_network(void)
{
    pkt_config = wb_pkt_iface64_init(PKT_RAM0_BASE, 0x1ffc);
    printf("PKT RAM0 BASE %u\n", (&pkt_config)->base);
    u3_net_stack_init(&pkt_config);

    static struct ip_addr my_ip0 = {(192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)};
    //IJB static struct ip_addr my_ip0 = {(192 << 24 | 168 << 16 | 10  << 8  | 2 << 0)};
    static eth_mac_addr_t my_mac0 = {{0x00, 0x50, 0xC2, 0x85, 0x3f, 0xff}};
    u3_net_stack_init_eth(0, &my_mac0, &my_ip0);

    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 0), (my_mac0.addr[5] << 0) | (my_mac0.addr[4] << 8) | (my_mac0.addr[3] << 16) | (my_mac0.addr[2] << 24));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 1), (my_mac0.addr[1] << 0) | (my_mac0.addr[0] << 8));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT0 + 8 + 2), my_ip0.addr);

    static struct ip_addr my_ip1 = {(192 << 24 | 168 << 16 | 20  << 8  | 2 << 0)};
    static eth_mac_addr_t my_mac1 = {{0x00, 0x50, 0xC2, 0x85, 0x3f, 0x33}};
    u3_net_stack_init_eth(1, &my_mac1, &my_ip1);

    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 0), (my_mac1.addr[5] << 0) | (my_mac1.addr[4] << 8) | (my_mac1.addr[3] << 16) | (my_mac1.addr[2] << 24));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 1), (my_mac1.addr[1] << 0) | (my_mac1.addr[0] << 8));
    wb_poke32(SR_ADDR(SET0_BASE, SR_ETHINT1 + 8 + 2), my_ip1.addr);
}

static void putc(void *p, char c)
{
    //wb_uart_putc(UART0_BASE, c);
}

/*
DCO driving Ethernet transceiver. B250 Instance U508.

 It's going to come up at 10MHz. To program it you're supposed to read out a
calibration value and work from there, but you can get close with static
values.

Everything is vanilla I2C, address 0x55.

Step 1: Freeze the DCO by setting addr 137 (0x89) bit 4. It's the only bit
in that reg so you can probably just write it without worrying about
masking.
Step 2: Write:
    0xe0 to addr 0x07
    0xc3 to addr 0x08
    0x02 to addr 0x09
    0x01 to addr 0x0a
    0x3b to addr 0x0b
    0x64 to addr 0x0c
Step 3: Unfreeze the DCO by clearing 0x89 bit 4.
Step 4: Assert the new frequency by setting 0x87 bit 6. These are all
self-clearing bits so just set bit 6 only, no masking required.

--n
*/

//
// 64bit division of unsigned integers. Remainder is not calculated.
//
uint64_t div_u64(const uint64_t dividend, const uint64_t divisor)
{
  uint64_t x,z=0;
  int8_t y=0;
  
  while (dividend >= (divisor << y))
    y++;
  // Early return if divisor larger than dividend
  if (y==0) return(0);

  x = dividend;

  while((--y) >= 0) {
    z = z << 1;
    // Try to subtract the shifted divisor and if the result is non-negative record a one
    // for this bit in the result and recalculate the remaining dividend.
    // 
    if (x >= (divisor << y)) {
      z = z | 1;
      x = x - (divisor << y);
    }
  }
  
  return (z);
}


void dco_write(const uint8_t addr, const uint8_t val)
{
    uint8_t buff[2];
    buff[0] = addr;
    buff[1] = val;
    wb_i2c_write(I2C0_BASE, 0x55, buff, 2);
}

uint8_t dco_read(const uint8_t addr)
{
  uint8_t val;
  wb_i2c_write(I2C0_BASE, 0x55, &addr, 1);
  wb_i2c_read(I2C0_BASE, 0x55, &val, 1);
  return val;
}

void dco_show_config(void)
{
  printf("DEBUG: DCO configuration:\n");
  printf("0x7=%x\n",dco_read(0x07));
  printf("0x8=%x\n",dco_read(0x08));
  printf("0x9=%x\n",dco_read(0x09));
  printf("0xA=%x\n",dco_read(0x0A));
  printf("0xB=%x\n",dco_read(0x0B));
  printf("0xC=%x\n",dco_read(0x0C));
}

/*
1) Read Si570 current register values.
2) Try to guess if it's currently programmed for 10/125/156.25MHz
3) Based on guess, determine actual Fxtal.
4) Program new Fout configuration
*/
// ***NOTE: This SI750 programming code is still unproven and suspect!***

void set_si570_freq(const dco_freq_t new_freq)
{
  uint8_t si570_regs[6];
  uint32_t x;
  uint64_t hs_div; 
  uint64_t n1; 
  uint64_t rfreq; 
  uint64_t fxtal; 
  //  uint32_t fout;

  for (x=0;x<6;x++) 
    si570_regs[x] = dco_read(x+7);

  rfreq =
    (uint64_t)si570_regs[5] |
    ((uint64_t)si570_regs[4] << 8) |
    ((uint64_t)si570_regs[3] << 16) |
    ((uint64_t)si570_regs[2] << 24) |
    (((uint64_t)si570_regs[1] & 0x3F) << 32);

  n1 =
    (
     ((uint64_t)(si570_regs[1] & 0xC0) >> 6) |
     ((uint64_t)(si570_regs[0] & 0x1F) << 2)
     ) + 1;

  hs_div =  ((uint64_t)(si570_regs[0] & 0xE0) >> 5) + 4;
  
  /* // Fxtal is approximately 114.3MHz. Use this knowledge to guess current programmed Fout. */
  //fxtal = (10^7 * hs_div * n1 * 2^28) / rfreq;
  
  fxtal = (156250000 * hs_div * n1 * (2<<27));
  fxtal = div_u64(fxtal, rfreq);

  if ((fxtal < 114000000) || (fxtal > 115000000)) {
    // It's not currently programmed for 156.25MHz
    // Try 125MHz....
    fxtal = (125000000 * hs_div * n1 * (2<<27));
    fxtal = div_u64(fxtal, rfreq);
    if ((fxtal < 114000000) || (fxtal > 115000000)) {
      // It's not currently programmed for 125MHz
      // Try (Assume!) 10MHz....
      fxtal = (10000000 * hs_div * n1 * (2<<27));
      fxtal = div_u64(fxtal, rfreq);
    }
  }

  printf("DEBUG: original HS_DIV=%08lx%08lx\n",(unsigned long)(hs_div>>32),(unsigned long)(hs_div&0xFFFFFFFF));
  printf("DEBUG: original RFREQ=%08lx%08lx\n",(unsigned long)(rfreq>>32),(unsigned long)(rfreq&0xFFFFFFFF));
  printf("DEBUG: original N1=%08lx%08lx\n",(unsigned long)(n1>>32),(unsigned long)(n1&0xFFFFFFFF));
  printf("DEBUG: Calculated FXTAL=%08lx%08lx\n",(unsigned long)(fxtal>>32),(unsigned long)(fxtal&0xFFFFFFFF));
  printf("DEBUG: (In decimal FXTAL=%ld MHz)\n",(unsigned long)(fxtal&0xFFFFFFFF));
 
  switch(new_freq) {
  case DCO_156p25: 
    printf("DEBUG: New DCO Fout will be 156.25MHz\n");
    n1 = 4;
    hs_div = 9;
    rfreq =  div_u64((156250000 * n1 * hs_div * (2<<27)),fxtal);
    break;
  case DCO_125:
    printf("DEBUG: New DCO Fout will be 125MHz\n");
    n1 = 4;
    hs_div = 11;
    rfreq =  div_u64((125000000 * n1 * hs_div * (2<<27)),fxtal);
    break;
  case DCO_10:
    printf("DEBUG: New DCO Fout will be 10MHz\n");
    n1 = 54;
    hs_div = 9;
    rfreq =  div_u64((10000000 * n1 * hs_div * (2<<27)),fxtal);
    break;
  }
  printf("DEBUG: new HS_DIV=%08lx%08lx\n",(unsigned long)(hs_div>>32),(unsigned long)(hs_div&0xFFFFFFFF));
  printf("DEBUG: new RFREQ=%08lx%08lx\n",(unsigned long)(rfreq>>32),(unsigned long)(rfreq&0xFFFFFFFF));
  printf("DEBUG: new N1=%08lx%08lx\n",(unsigned long)(n1>>32),(unsigned long)(n1&0xFFFFFFFF));
  printf("DEBUG: Calculated RFREQ=%08lx%08lx\n",(unsigned long)(rfreq>>32),(unsigned long)(rfreq&0xFFFFFFFF));
  
  // Freeze DCO ready for update.
  dco_write(0x89, 1 << 4);
  // Program new clock.
  dco_write(0x07, ((hs_div-4)<<5)|((n1-1)>>2));
  dco_write(0x08, (((n1-1)&0x3)<<6)|((rfreq>>32)&0x3F));
  dco_write(0x09, (rfreq>>24)&0xFF);
  dco_write(0x0A, (rfreq>>16)&0xFF);
  dco_write(0x0B, (rfreq>>8)&0xFF);
  dco_write(0x0C, (rfreq)&0xFF);
  // Re-Enbale DCO
  dco_write(0x89, 0 << 4);
  dco_write(0x87, 1 << 6);
    
}
  

void b250_init(void)
{
    //first - uart
    wb_uart_init(UART0_BASE, CPU_CLOCK/UART0_BAUD);
    init_printf(NULL,putc);
    udp_uart_init(UART0_BASE, B250_GPSDO_UDP_PORT);

    //now we can init the rest with prints
    printf("B250 ZPU Init Begin -- CPU CLOCK is %d MHz\n", CPU_CLOCK/1000000);

    //i2c rate init
    wb_i2c_init(I2C0_BASE, CPU_CLOCK);

    //hold phy in reset
    wb_poke32(SR_ADDR(SET0_BASE, SR_SW_RST), SW_RST_PHY);

    // IJB. NOTE, SiLabs state that we should first read the factory default 10MHz settings
    // to determine Fxtal and hence derive and exact value for RFREQ. Currently we are just hard coding it.

    //init clock - i2c perif
    // Show power-on or after reset value of DCO.
    // dco_show_config();

    printf("DEBUG: Version reports %8x\n",wb_peek32(SR_ADDR(RB0_BASE, RB_VERSION)));

    if (wb_peek32(SR_ADDR(RB0_BASE, RB_VERSION)) == 0) {
      /* // 125MHZ */
      //  set_si570_freq(DCO_125);
      dco_write(0x89, 1 << 4);
      dco_write(0x07, 0xe0);
      dco_write(0x08, 0xc3);
      dco_write(0x09, 0x02);
      dco_write(0x0a, 0x01);
      dco_write(0x0b, 0x3b);
      dco_write(0x0c, 0x64);
      dco_write(0x89, 0 << 4);
      dco_write(0x87, 1 << 6);
      //    } else if (wb_peek32(SR_ADDR(RB0_BASE, RB_VERSION)) == 1) {

    } else {    // 156.25MHz
      // Do nothing. 10GE image currently uses dedicated 156.25MHz clock source from U509
      //  set_si570_freq(DCO_156p25);
      /* dco_write(0x89, 1 << 4); */
      /* dco_write(0x07, 0xA0); */
      /* dco_write(0x08, 0xC3);  */
      /* dco_write(0x09, 0x13);   */
      /* dco_write(0x0a, 0x1F);   */
      /* dco_write(0x0b, 0x3D);   */
      /* dco_write(0x0c, 0x66);   */
      /* dco_write(0x89, 0 << 4);   */
      /* dco_write(0x87, 1 << 6);   */
      //    } else printf("ERROR: Version %8x is unrecognized.\n",wb_peek32(SR_ADDR(RB0_BASE, RB_VERSION)));
    }    
    //setup net stack and eth state machines
    init_network();
  
    //phy reset release
    wb_poke32(SR_ADDR(SET0_BASE, SR_SW_RST), 0);
    // Run only for 10GE
    if (wb_peek32(SR_ADDR(RB0_BASE, RB_VERSION)) != 0) {
	 mdelay(100);
	 xge_ethernet_init(0);
    }


}

static uint32_t hex_char_to_num(const int ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    else return 0;
}

static void handle_loader(const int ch)
{
    //state variables
    static int buff[16];
    static size_t index = 0;

    //is this a hex digit?
    const bool is_hex =
        (ch >= '0' && ch <= '9') ||
        (ch >= 'a' && ch <= 'f') ||
        (ch >= 'A' && ch <= 'F');

    //perform load operation
    if (!is_hex && index == 16)
    {
        const uint32_t addr = 0
            | (hex_char_to_num(buff[0]) << 28)
            | (hex_char_to_num(buff[1]) << 24)
            | (hex_char_to_num(buff[2]) << 20)
            | (hex_char_to_num(buff[3]) << 16)
            | (hex_char_to_num(buff[4]) << 12)
            | (hex_char_to_num(buff[5]) << 8)
            | (hex_char_to_num(buff[6]) << 4)
            | (hex_char_to_num(buff[7]) << 0)
        ;
        const uint32_t data = 0
            | (hex_char_to_num(buff[8]) << 28)
            | (hex_char_to_num(buff[9]) << 24)
            | (hex_char_to_num(buff[10]) << 20)
            | (hex_char_to_num(buff[11]) << 16)
            | (hex_char_to_num(buff[12]) << 12)
            | (hex_char_to_num(buff[13]) << 8)
            | (hex_char_to_num(buff[14]) << 4)
            | (hex_char_to_num(buff[15]) << 0)
        ;
        for (size_t i = 0; i < 16; i++)
        {
            wb_uart_putc(UART0_BASE, buff[i]);
        }
        wb_uart_putc(UART0_BASE, '\r');
        wb_uart_putc(UART0_BASE, '\n');
        if (addr >= BOOT_RAM_BASE && addr < BOOT_RAM_BASE + 0x4000)
        {
            wb_poke32(addr, data);
        }
    }

    //save the digit
    if (is_hex && index < 16)
    {
        buff[index++] = ch;
    }

    //error, reset count
    else
    {
        index = 0;
    }
}

void b250_serial_loader_run1(void)
{
    int ch = wb_uart_getc(UART0_BASE);
    if (ch != -1) handle_loader(ch);
}
