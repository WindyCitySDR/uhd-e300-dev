#include <stdint.h>
#include <wb_uart.h>
#include <wb_utils.h>
//#include <ctype.h>
#include <stdbool.h>

#define RAM0_BASE 0x0000
#define RAM1_BASE 0x4000
#define UART0_BASE 0x8000
#define I2C_BASE 0x8800
#define SET_BASE 0x9000

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
            wb_poke32(0x9000, addr); //lets see the address on the data bus...
            wb_poke32(0x9004, data); //lets see the address on the data bus...
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

int main(void)
{
    wb_uart_init(UART0_BASE, 125000000/115200);
    wb_uart_putc(UART0_BASE, 'H');
    wb_uart_putc(UART0_BASE, 'e');
    wb_uart_putc(UART0_BASE, 'l');
    wb_uart_putc(UART0_BASE, 'l');
    wb_uart_putc(UART0_BASE, 'o');
    wb_uart_putc(UART0_BASE, '\r');
    wb_uart_putc(UART0_BASE, '\n');
    int count = 0;
    while(1)
    {
        //wb_poke32(UART0_BASE+12, '\0');
        int ch = wb_uart_getc(UART0_BASE);
        if (ch != -1) handle_loader(ch);
        //wb_uart_putc(UART0_BASE, 'u');
        //wb_uart_putc(UART0_BASE, 'k');
        //wb_uart_putc(UART0_BASE, '\n');
        //wb_poke32(SET_BASE, count++);
    }
    return 0;
}
