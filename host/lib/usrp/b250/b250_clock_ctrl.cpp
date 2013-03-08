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
#include "b250_impl.hpp"
#include <uhd/utils/safe_call.hpp>
#include "ad9510_regs.hpp"
#include <boost/cstdint.hpp>

using namespace uhd;

struct b250_clock_ctrl_impl : b250_clock_ctrl
{
    b250_clock_ctrl_impl(uhd::spi_iface::sptr spiface, const size_t slaveno):
        _spiface(spiface), _slaveno(slaveno)
    {
        _enables[B250_CLOCK_WHICH_TEST] = true;
        this->update_enables();
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
        return B250_RADIO_CLOCK_RATE;
    }

    void enable_clock(const b250_clock_which_t which, const bool enb)
    {
        _enables[which] = enb;
        this->update_enables();
    }

    void update_enables(void)
    {
        //0
        _ad9510_regs.power_down_lvpecl_out0 = _enables.get(B250_CLOCK_WHICH_TEST, false)?
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT0_NORMAL :
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT0_SAFE_PD;
        _ad9510_regs.output_level_lvpecl_out0 = ad9510_regs_t::OUTPUT_LEVEL_LVPECL_OUT0_810MV;
        _ad9510_regs.bypass_divider_out0 = 1;

        //1
        _ad9510_regs.power_down_lvpecl_out1 = ad9510_regs_t::POWER_DOWN_LVPECL_OUT1_SAFE_PD;
        _ad9510_regs.bypass_divider_out1 = 1;

        //2
        const bool enb_rx = _enables.get(B250_CLOCK_WHICH_DB0_RX, false) or _enables.get(B250_CLOCK_WHICH_DB1_RX, false);
        _ad9510_regs.power_down_lvpecl_out2 = enb_rx?
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT2_NORMAL :
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT2_SAFE_PD;
        _ad9510_regs.output_level_lvpecl_out2 = ad9510_regs_t::OUTPUT_LEVEL_LVPECL_OUT2_810MV;
        _ad9510_regs.bypass_divider_out2 = 1;

        //3
        const bool enb_tx = _enables.get(B250_CLOCK_WHICH_DB0_TX, false) or _enables.get(B250_CLOCK_WHICH_DB1_TX, false);
        _ad9510_regs.power_down_lvpecl_out3 = enb_tx?
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT3_NORMAL :
            ad9510_regs_t::POWER_DOWN_LVPECL_OUT3_SAFE_PD;
        _ad9510_regs.output_level_lvpecl_out3 = ad9510_regs_t::OUTPUT_LEVEL_LVPECL_OUT3_810MV;
        _ad9510_regs.bypass_divider_out3 = 1;

        //4
        _ad9510_regs.power_down_lvds_cmos_out4 = 0; //always FPGA CLOCK ON
        _ad9510_regs.lvds_cmos_select_out4 = ad9510_regs_t::LVDS_CMOS_SELECT_OUT4_LVDS;
        _ad9510_regs.output_level_lvds_out4 = ad9510_regs_t::OUTPUT_LEVEL_LVDS_OUT4_1_75MA;
        _ad9510_regs.bypass_divider_out4 = 1;

        //5
        _ad9510_regs.power_down_lvds_cmos_out5 = 1; //NC off
        _ad9510_regs.lvds_cmos_select_out5 = ad9510_regs_t::LVDS_CMOS_SELECT_OUT5_LVDS;
        _ad9510_regs.output_level_lvds_out5 = ad9510_regs_t::OUTPUT_LEVEL_LVDS_OUT5_1_75MA;
        _ad9510_regs.bypass_divider_out5 = 1;

        //6
        _ad9510_regs.power_down_lvds_cmos_out6 = 1; //MIMO off
        _ad9510_regs.lvds_cmos_select_out6 = ad9510_regs_t::LVDS_CMOS_SELECT_OUT6_LVDS;
        _ad9510_regs.output_level_lvds_out6 = ad9510_regs_t::OUTPUT_LEVEL_LVDS_OUT6_1_75MA;
        _ad9510_regs.bypass_divider_out6 = 1;

        //7
        _ad9510_regs.power_down_lvds_cmos_out7 = 1; //NC off
        _ad9510_regs.lvds_cmos_select_out7 = ad9510_regs_t::LVDS_CMOS_SELECT_OUT7_LVDS;
        _ad9510_regs.output_level_lvds_out7 = ad9510_regs_t::OUTPUT_LEVEL_LVDS_OUT7_1_75MA;
        _ad9510_regs.bypass_divider_out7 = 1;

        for (size_t i = 0; i < 4; i++) this->write_reg(0x3C+i);
        for (size_t i = 0; i < 4; i++) this->write_reg(0x40+i);
        for (size_t i = 0; i < 8; i++) this->write_reg(0x48+(i*2));
        for (size_t i = 0; i < 8; i++) this->write_reg(0x49+(i*2));
        this->update_regs();
    }

    void set_rate(const b250_clock_which_t, double)
    {
        //TODO
    }

    std::vector<double> get_rates(const b250_clock_which_t)
    {
        std::vector<double> rates;
        for (size_t i = 1; i <= 16+16; i++) rates.push_back(get_master_clock_rate()/i);
        return rates;
    }

    /*!
     * Write a single register to the spi regs.
     * \param addr the address to write
     */
    void write_reg(boost::uint8_t addr)
    {
        boost::uint32_t data = _ad9510_regs.get_write_reg(addr);
        _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, data, 24);
    }

    /*!
     * Tells the ad9510 to latch the settings into the operational registers.
     */
    void update_regs(void)
    {
        _ad9510_regs.update_registers = 1;
        this->write_reg(0x5A);
    }

    const spi_iface::sptr _spiface;
    const size_t _slaveno;
    ad9510_regs_t _ad9510_regs;
    uhd::dict<b250_clock_which_t, bool> _enables;

};

b250_clock_ctrl::sptr b250_clock_ctrl::make(uhd::spi_iface::sptr spiface, const size_t slaveno)
{
    return sptr(new b250_clock_ctrl_impl(spiface, slaveno));
}
