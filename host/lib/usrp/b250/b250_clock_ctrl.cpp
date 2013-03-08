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

#include "b250_clock_ctrl.hpp"
#include <uhd/utils/safe_call.hpp>

using namespace uhd;

#define write_ad9510_reg(addr, data) \
    _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, ((addr) << 8) | (data), 24)

struct b250_clock_ctrl_impl : b250_clock_ctrl
{
    b250_clock_ctrl_impl(uhd::spi_iface::sptr spiface, const size_t slaveno):
        _spiface(spiface), _slaveno(slaveno)
    {
        write_ad9510_reg(0x3C, 0x08); // TEST_CLK on
        write_ad9510_reg(0x3D, 0x02); // NC off
        write_ad9510_reg(0x3E, 0x08); // RX_CLK on
        write_ad9510_reg(0x3F, 0x08); // TX_CLK on
        write_ad9510_reg(0x40, 0x02); // FPGA_CLK on
        write_ad9510_reg(0x41, 0x01); // NC off
        write_ad9510_reg(0x42, 0x01); // MIMO off
        write_ad9510_reg(0x43, 0x01); // NC off
        write_ad9510_reg(0x49, 0x80); // TEST_CLK bypass div
        write_ad9510_reg(0x4B, 0x80); // NC bypass div
        write_ad9510_reg(0x4D, 0x80); // RX_CLK bypass div
        write_ad9510_reg(0x4F, 0x80); // TX_CLK bypass div
        write_ad9510_reg(0x51, 0x80); // FPGA_CLK bypass div
        write_ad9510_reg(0x53, 0x80); // NC bypass div
        write_ad9510_reg(0x55, 0x80); // MIMO bypass div
        write_ad9510_reg(0x57, 0x80); // NC bypass div
        write_ad9510_reg(0x5a, 0x01); // Apply settings
    }

    ~b250_clock_ctrl_impl(void)
    {
        UHD_SAFE_CALL
        (
            //TODO
        )
    }

    double get_master_clock_rate(void)
    {
        
    }

    void enable_clock(const b250_clock_which_t which, const bool)
    {
        
    }

    void set_rate(const b250_clock_which_t which, double rate)
    {
        
    }


    std::vector<double> get_rates(const b250_clock_which_t which)
    {
        
    }

    const spi_iface::sptr _spiface;
    const size_t _slaveno;

};

b250_clock_ctrl::sptr b250_clock_ctrl::make(uhd::spi_iface::sptr spiface, const size_t slaveno)
{
    return sptr(new b250_clock_ctrl_impl(spiface, slaveno));
}
