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

#include "b250_adc_ctrl.hpp"
#include "ads62p44_regs.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/exception.hpp>
#include <boost/foreach.hpp>

using namespace uhd;

/*!
 * A B250 codec control specific to the ads62p44 ic.
 */
class b250_adc_ctrl_impl : public b250_adc_ctrl
{
public:
    b250_adc_ctrl_impl(uhd::spi_iface::sptr iface, const size_t slaveno):
        _iface(iface), _slaveno(slaveno)
    {
        //power-up adc
       _ads62p44_regs.reset = 1;
        this->send_ads62p44_reg(0x00); //issue a reset to the ADC
        //everything else should be pretty much default, i think
        //_ads62p44_regs.decimation = DECIMATION_DECIMATE_1;
        _ads62p44_regs.override = 1;
        this->send_ads62p44_reg(0x14);
        _ads62p44_regs.power_down = ads62p44_regs_t::POWER_DOWN_NORMAL;
        _ads62p44_regs.output_interface = ads62p44_regs_t::OUTPUT_INTERFACE_LVDS;
        _ads62p44_regs.lvds_current = ads62p44_regs_t::LVDS_CURRENT_2_5MA;
        _ads62p44_regs.lvds_data_term = ads62p44_regs_t::LVDS_DATA_TERM_100;
        this->send_ads62p44_reg(0x11);
        this->send_ads62p44_reg(0x12);
        this->send_ads62p44_reg(0x14);
        this->set_rx_analog_gain(1);

        _ads62p44_regs.test_patterns = ads62p44_regs_t::TEST_PATTERNS_NORMAL;
        this->send_ads62p44_reg(22);

    }

    ~b250_adc_ctrl_impl(void)
    {
        UHD_SAFE_CALL
        (
            _ads62p44_regs.power_down = ads62p44_regs_t::POWER_DOWN_GLOBAL_PD;
            //this->send_ads62p44_reg(0x14);
        )
    }

    void set_rx_digital_gain(double gain)  //fine digital gain
    {
        _ads62p44_regs.fine_gain = int(gain/0.5);
        this->send_ads62p44_reg(0x17);
    }

    void set_rx_digital_fine_gain(double gain)  //gain correction   
    {   
        _ads62p44_regs.gain_correction = int(gain / 0.05);
        this->send_ads62p44_reg(0x1A);
    }

    void set_rx_analog_gain(bool /*gain*/)  //turns on/off analog 3.5dB preamp
    {
        _ads62p44_regs.coarse_gain = ads62p44_regs_t::COARSE_GAIN_3_5DB;//gain ? ads62p44_regs_t::COARSE_GAIN_3_5DB : ads62p44_regs_t::COARSE_GAIN_0DB;
        this->send_ads62p44_reg(0x14);
    }

private:
    ads62p44_regs_t _ads62p44_regs;
    uhd::spi_iface::sptr _iface;
    const size_t _slaveno;

    void send_ads62p44_reg(boost::uint8_t addr)
    {
        boost::uint16_t reg = _ads62p44_regs.get_write_reg(addr);
        _iface->write_spi(_slaveno, spi_config_t::EDGE_FALL, reg, 16);
    }
};

/***********************************************************************
 * Public make function for the usrp2 codec control
 **********************************************************************/
b250_adc_ctrl::sptr b250_adc_ctrl::make(uhd::spi_iface::sptr iface, const size_t slaveno)
{
    return sptr(new b250_adc_ctrl_impl(iface, slaveno));
}
