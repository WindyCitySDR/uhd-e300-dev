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

#include <uhd/usrp/dboard_iface.hpp>

using namespace uhd;
using namespace uhd::usrp;

using namespace uhd;
using namespace uhd::usrp;

class b250_dboard_iface : public dboard_iface
{
public:
    b250_dboard_iface(void){}
    ~b250_dboard_iface(void){}

    special_props_t get_special_props(void)
    {
        special_props_t props;
        props.soft_clock_divider = false;
        props.mangle_i2c_addrs = false;
        return props;
    }

    void write_aux_dac(unit_t, aux_dac_t, double){}
    double read_aux_adc(unit_t, aux_adc_t){}

    void _set_pin_ctrl(unit_t, boost::uint16_t){}
    void _set_atr_reg(unit_t, atr_reg_t, boost::uint16_t){}
    void _set_gpio_ddr(unit_t, boost::uint16_t){}
    void _set_gpio_out(unit_t, boost::uint16_t){}

    void set_gpio_debug(unit_t, int){}
    boost::uint16_t read_gpio(unit_t){}

    void write_i2c(boost::uint8_t, const byte_vector_t &){}
    byte_vector_t read_i2c(boost::uint8_t, size_t){}

    void set_clock_rate(unit_t, double){}
    double get_clock_rate(unit_t){}
    std::vector<double> get_clock_rates(unit_t){}
    void set_clock_enabled(unit_t, bool){}
    double get_codec_rate(unit_t){}

    void write_spi(
        unit_t unit,
        const spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits
    ){}

    boost::uint32_t read_write_spi(
        unit_t unit,
        const spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits
    ){}

};

/***********************************************************************
 * Make Function
 **********************************************************************/
dboard_iface::sptr b250_make_dboard_iface(void)
{
    return dboard_iface::sptr(new b250_dboard_iface());
}
