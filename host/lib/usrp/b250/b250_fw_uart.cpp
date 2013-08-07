//
// Copyright 2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "b250_impl.hpp"
#include "wb_iface.hpp"
#include "b250_regs.hpp"
#include <uhd/utils/msg.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/exception.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

using namespace uhd;

struct b250_uart_iface : uart_iface
{
    b250_uart_iface(wb_iface::sptr iface):
        rxoffset(0), txoffset(0)
    {
        _iface = iface;
        rxoffset = _iface->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_RX_INDEX));
        _iface->poke32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_TX_INDEX), txoffset);
        //this->write_uart("HELLO UART\n");
        //this->read_uart(0.1);
    }

    void putchar(const char ch)
    {
        txoffset = (txoffset + 1) % X300_FW_SHMEM_UART_POOL_WORDS32;
        _iface->poke32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_TX_POOL+txoffset), ch);
        _iface->poke32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_TX_INDEX), txoffset);
    }

    void write_uart(const std::string &buff)
    {
        BOOST_FOREACH(const char ch, buff)
        {
            if (ch == '\n') this->putchar('\r');
            this->putchar(ch);
        }
    }

    int getchar(void)
    {
        if (_iface->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_RX_INDEX)) != rxoffset)
        {
            const char ch = _iface->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_UART_RX_POOL+rxoffset));
            rxoffset = (rxoffset + 1) % X300_FW_SHMEM_UART_POOL_WORDS32;
            return ch;
        }
        return -1;
    }

    std::string read_uart(double timeout)
    {
        const boost::system_time exit_time = boost::get_system_time() + boost::posix_time::microseconds(long(timeout*1e6));
        std::string buff;
        while (true)
        {
            const int ch = this->getchar();
            if (ch == -1)
            {
                if (boost::get_system_time() > exit_time) break;
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                continue;
            }
            if (ch == '\r') continue;
            buff += std::string(1, (char)ch);
            if (ch == '\n') break;
        }
        //UHD_VAR(buff);
        return buff;
    }

    wb_iface::sptr _iface;
    boost::uint32_t rxoffset, txoffset;
};

uart_iface::sptr b250_make_uart_iface(wb_iface::sptr iface)
{
    return uart_iface::sptr(new b250_uart_iface(iface));
}