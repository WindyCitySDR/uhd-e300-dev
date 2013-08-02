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
#include <uhd/types/time_spec.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/exception.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp> //sleep

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
    b250_dac_ctrl_impl(uhd::spi_iface::sptr iface, const size_t slaveno, const double refclk):
        _iface(iface), _slaveno(slaveno)
    {
        write_ad9146_reg(0x00, 0x20); //reset
        write_ad9146_reg(0x00, 0x80); //config + out of reset
        write_ad9146_reg(0x1e, 0x01); //data path config - set for proper operation

        /* calculate N0 to be VCO friendly */
        const int N1 = 4;
        int N0_val, N0;
        for (N0_val = 0; N0_val < 3; N0_val++)
        {
            N0 = (1 << N0_val); //1, 2, 4
            if ((refclk * N0 * N1) >= 1e9) break;
        }
        UHD_ASSERT_THROW((refclk * N0 * N1) >= 1e9);
        UHD_ASSERT_THROW((refclk * N0 * N1) <= 2e9);

        /* Start PLL */
        write_ad9146_reg(0x0C, 0xD1);
        write_ad9146_reg(0x0D, 0xD1 | (N0_val << 2)); // N1=4, N2=16
        write_ad9146_reg(0x0A, 0xCF); // Auto init VCO band training
        write_ad9146_reg(0x0A, 0xA0); // See above.

        /* Verify PLL is Locked */
        const time_spec_t exit_time = time_spec_t::get_system_time() + time_spec_t(0.5);
        while (true)
        {
            const size_t reg_e = read_ad9146_reg(0x0E); /* Expect bit 7 = 0, bit 6 = 1 */
            if ((reg_e & ((1 << 7) | (1 << 6))) != 0) break;
            if (exit_time < time_spec_t::get_system_time()) throw uhd::runtime_error(
                "b250_dac_ctrl: timeout waiting for DAC PLL to lock"
            );
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }

	/* Skew DCI signal to find stable data eye */
	//write_ad9146_reg(0x16, 0x04); //Disable delay in DCI
	//write_ad9146_reg(0x16, 0x00); //165ps delay in DCI
	//write_ad9146_reg(0x16, 0x01); //375ps delay in DCI
	write_ad9146_reg(0x16, 0x02); //615ps delay in DCI
	//write_ad9146_reg(0x16, 0x03); //720ps delay in DCI
 
        this->set_iq_swap(false);
        write_ad9146_reg(0x10, 0x48); // Choose data rate mode
        write_ad9146_reg(0x17, 0x04); // Issue software FIFO reset
        write_ad9146_reg(0x18, 0x01); //
        write_ad9146_reg(0x18, 0x00); //
        write_ad9146_reg(0x1B, 0xA4); // Enable inverse SINC

        /* Configure interpolation filters */
        write_ad9146_reg(0x1C, 0x00); //
        write_ad9146_reg(0x1D, 0x00); //

    }

    ~b250_dac_ctrl_impl(void)
    {
        UHD_SAFE_CALL
        (
            write_ad9146_reg(0x1, 0xf); //total power down
            write_ad9146_reg(0x2, 0xf); //total power down
        )
    }

    void set_iq_swap(const bool swap)
    {
        //fpga wants I,Q in the sample word:
        //first transaction goes into low bits
        //second transaction goes into high bits
        //therefore, we want Q to go first (bit 6 == 1)
        const int bit = (swap)? 0 : (1 << 6);
        write_ad9146_reg(0x03, 0x00 | bit); //2s comp, i first, byte mode
    }

private:
    uhd::spi_iface::sptr _iface;
    const size_t _slaveno;
};

/***********************************************************************
 * Public make function for the DAC control
 **********************************************************************/
b250_dac_ctrl::sptr b250_dac_ctrl::make(uhd::spi_iface::sptr iface, const size_t slaveno, const double clock_rate)
{
    return sptr(new b250_dac_ctrl_impl(iface, slaveno, clock_rate));
}
