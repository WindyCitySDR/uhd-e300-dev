#include "b250_init.h"
#include "b250_defs.h"
#include <wb_utils.h>
#include <wb_uart.h>
#include <wb_i2c.h>
#include <stdint.h>
#include <stdbool.h>
#include <printf.h>

static void b250_printf_emitter(char c, void *p)
{
    wb_uart_putc(UART0_BASE, c);
}

/*
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

void dco_write(const uint8_t addr, const uint8_t val)
{
    uint8_t buff[2];
    buff[0] = addr;
    buff[1] = val;
    wb_i2c_write(I2C0_BASE, 0x55, buff, 2);
}

void b250_init(void)
{
    //first - uart
    wb_uart_init(UART0_BASE, CPU_CLOCK/UART0_BAUD);
    printf_register(&b250_printf_emitter);

    //now we can init the rest with prints
    printf("B250 ZPU Init Begin -- CPU CLOCK is %d MHz\n", CPU_CLOCK/1000000);

    //i2c rate init
    wb_i2c_init(I2C0_BASE, CPU_CLOCK);

    //hold phy in reset
    wb_poke32(SR_PHY_RST, 1);

    //init clock - i2c perif
    dco_write(0x89, 1 << 4);
    dco_write(0x07, 0xe0);
    dco_write(0x08, 0xc3);
    dco_write(0x09, 0x02);
    dco_write(0x0a, 0x01);
    dco_write(0x0b, 0x3b);
    dco_write(0x0c, 0x64);
    dco_write(0x89, 0 << 4);
    dco_write(0x87, 1 << 6);

    //phy reset release
    wb_poke32(SR_PHY_RST, 0);
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
        if (addr >= RAM1_BASE && addr < RAM1_BASE + 0x4000)
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
