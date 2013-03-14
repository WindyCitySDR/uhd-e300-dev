//
// Copyright 2010-2013 Ettus Research LLC
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

#include "b250_dac_ctrl.hpp"
#include <uhd/utils/msg.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/exception.hpp>
#include <boost/foreach.hpp>

using namespace uhd;

#define write_ad9146_reg(addr, data) \
    _iface->write_spi(_slaveno, spi_config_t::EDGE_RISE, ((addr) << 8) | (data), 16)
#define read_ad9146_reg(addr) \
    (_iface->read_spi(_slaveno, spi_config_t::EDGE_RISE, ((addr) << 8) | (1 << 15), 16) & 0xff)

/*!
 * A B250 codec control specific to the ad9146 ic.
 */
class b250_dac_ctrl_impl : public b250_dac_ctrl
{
public:
    b250_dac_ctrl_impl(uhd::spi_iface::sptr iface, const size_t slaveno):
        _iface(iface), _slaveno(slaveno)
    {
        write_ad9146_reg(0x0, 1 << 5); //reset
        write_ad9146_reg(0x0, 1 << 7); //config + out of reset
        write_ad9146_reg(0x1, 0x0); //out of power down
        write_ad9146_reg(0x2, 0x0); //out of power down
        write_ad9146_reg(0x3, 0x0); //2s comp, i first, no swap, byte mode
        write_ad9146_reg(0x4, 0x0); //no interrupt
        write_ad9146_reg(0x5, 0x0); //no interrupt

        write_ad9146_reg(0x8, (0xf << 4)); //duty correction, no pll
        write_ad9146_reg(0x1b, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 2)); //defaults
        write_ad9146_reg(0x1e, 1); //data path config - set for proper operation

        UHD_MSG(status) << std::hex << read_ad9146_reg(0x7f) << std::endl;
        UHD_MSG(status) << std::hex << read_ad9146_reg(0x1f) << std::endl;
    }

    ~b250_dac_ctrl_impl(void)
    {
        UHD_SAFE_CALL
        (
            write_ad9146_reg(0x1, 0xf); //total power down
            write_ad9146_reg(0x2, 0xf); //total power down
        )
    }


private:
    uhd::spi_iface::sptr _iface;
    const size_t _slaveno;
};

/***********************************************************************
 * Public make function for the DAC control
 **********************************************************************/
b250_dac_ctrl::sptr b250_dac_ctrl::make(uhd::spi_iface::sptr iface, const size_t slaveno)
{
    return sptr(new b250_dac_ctrl_impl(iface, slaveno));
}
