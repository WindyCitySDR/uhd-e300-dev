
#include "lmk04816_regs.hpp"
#include "x300_clock_ctrl.hpp"
#include <uhd/utils/safe_call.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#include <stdexcept>
#include <cstdlib>

// FIXME These are here for debug purposes only.
#include <cstdio>
#include <iostream>

using namespace uhd;

class x300_clock_ctrl_impl : public x300_clock_ctrl    {

public:

~x300_clock_ctrl_impl(void) {}

x300_clock_ctrl_impl(uhd::spi_iface::sptr spiface,
    const size_t slaveno,
    const size_t hw_rev,
    const double master_clock_rate,
    const double system_ref_rate):
    _spiface(spiface),
    _slaveno(slaveno),
    _hw_rev(hw_rev),
    _master_clock_rate(master_clock_rate),
    _system_ref_rate(system_ref_rate)
{
    set_master_clock_rate(master_clock_rate);
}

void sync_clocks(void) {
    //soft sync:
    //put the sync IO into output mode - FPGA must be input
    //write low, then write high - this triggers a soft sync
    _lmk04816_regs.SYNC_POL_INV = lmk04816_regs_t::SYNC_POL_INV_SYNC_LOW;
    this->write_regs(11);
    _lmk04816_regs.SYNC_POL_INV = lmk04816_regs_t::SYNC_POL_INV_SYNC_HIGH;
    this->write_regs(11);
}

double get_master_clock_rate(void) {
    return _master_clock_rate;
}

void set_master_clock_rate(double clock_rate) {
    /* The X3xx, internall, has two primary rates. The first is the
     * _system_ref_rate, which is sourced from the "clock_source"/"value" field
     * of the property tree, and whose value can be 100e6, 30.72e6, or 200e6.
     * The _system_ref_rate is the input to the clocking system, and
     * what comes out is a disciplined master clock running at the
     * _master_clock_rate. As such, only certain combinations of
     * system reference rates and master clock rates are supported.
     * Additionally, a subset of these will operate in "zero delay" mode. */

    enum opmode {m10M_200M_NOZDEL, // used for debug purposes only
                 m10M_200M_ZDEL,
                 m30_72M_184_32M_ZDEL,
                 m10M_184_32M_NOZDEL };
    opmode clocking_mode = m10M_200M_ZDEL;

    bool valid_rates = false;
    bool zero_delay_mode = false;

    if(_system_ref_rate == 10e6) {
        if(clock_rate == 184.32e6) {
            /* 10MHz reference, 184.32 MHz master clock out */
            valid_rates = true;

            clocking_mode = m10M_184_32M_NOZDEL;
        } else if(clock_rate == 200e6) {
            /* 10MHz reference, 200 MHz master clock out, Zero Delay */
            valid_rates = true;
            zero_delay_mode = true;

            clocking_mode = m10M_200M_ZDEL;
        }
    } else if(_system_ref_rate == 30.72e6) {
        if(clock_rate == 184.32e6) {
            /* 30.72MHz reference, 184.32 MHz master clock out, Zero Delay */
            valid_rates = true;
            zero_delay_mode = true;

            clocking_mode = m30_72M_184_32M_ZDEL;
        }
    }

    if(!valid_rates) {
        throw uhd::runtime_error(str(boost::format("A master clock rate of %f cannot be derived from a system reference rate of %f") % clock_rate % _system_ref_rate));
    }

    /* For any rate other than 184.32e6, the VCO is run at 2400 MHz.
     * For the LTE/CPRI rate of 184.32 MHz, the VCO run at 2580.48 MHz. */
    int vco_div = 0;

    lmk04816_regs_t::MODE_t lmkmode;
    int pll_1_n_div = 0; //int(clock_rate/refclk_rate);
    int pll_1_r_div = 0;
    int pll_2_n_div = 0;
    int pll_2_r_div = 0;
    lmk04816_regs_t::PLL1_CP_GAIN_27_t pll_1_cp_gain;
    lmk04816_regs_t::PLL2_CP_GAIN_26_t pll_2_cp_gain;

    // Note: PLL2 N2 prescaler is enabled for all cases
    //       PLL2 reference doubler is enabled for all cases
    switch (clocking_mode) {
        case m10M_200M_NOZDEL:
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT;
            pll_1_n_div = 48;
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 25;
            pll_2_r_div = 4;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            vco_div = 12;
            break;
        case m10M_200M_ZDEL:
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
            pll_1_n_div = 100;
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 25;
            pll_2_r_div = 4;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            vco_div = 12;
            break;
        case m30_72M_184_32M_ZDEL:
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
            pll_1_n_div = 90;
            pll_1_r_div = 15;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 168;
            pll_2_r_div = 25;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            vco_div=14;
            _lmk04816_regs.PLL2_R3_LF=lmk04816_regs_t:: PLL2_R3_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_R4_LF=lmk04816_regs_t:: PLL2_R4_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_C3_LF=lmk04816_regs_t:: PLL2_C3_LF_39PF;
            _lmk04816_regs.PLL2_C4_LF=lmk04816_regs_t:: PLL2_C4_LF_34PF;
            break;
        case m10M_184_32M_NOZDEL:
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT;
            pll_1_n_div = 48;
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 168;
            pll_2_r_div = 25;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            vco_div=14;
            _lmk04816_regs.PLL2_R3_LF=lmk04816_regs_t:: PLL2_R3_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_R4_LF=lmk04816_regs_t:: PLL2_R4_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_C3_LF=lmk04816_regs_t:: PLL2_C3_LF_39PF;
            _lmk04816_regs.PLL2_C4_LF=lmk04816_regs_t:: PLL2_C4_LF_34PF;
            break;
    };

    /* Reset the LMK clock controller. */
    _lmk04816_regs.RESET = lmk04816_regs_t::RESET_RESET;
    this->write_regs(0);
    _lmk04816_regs.RESET = lmk04816_regs_t::RESET_NO_RESET;
    this->write_regs(0);

    /* Initial power-up */
    _lmk04816_regs.CLKout0_1_PD = lmk04816_regs_t::CLKOUT0_1_PD_POWER_UP;
    this->write_regs(0);
    _lmk04816_regs.CLKout0_1_DIV = vco_div;
    this->write_regs(0);

    // Register 1
    _lmk04816_regs.CLKout2_3_PD = lmk04816_regs_t::CLKOUT2_3_PD_POWER_UP;
    _lmk04816_regs.CLKout2_3_DIV = vco_div;
    // Register 2
    _lmk04816_regs.CLKout4_5_PD = lmk04816_regs_t::CLKOUT4_5_PD_POWER_UP;
    _lmk04816_regs.CLKout4_5_DIV = vco_div;
    // Register 3
    _lmk04816_regs.CLKout6_7_DIV = vco_div;
    _lmk04816_regs.CLKout6_7_OSCin_Sel = lmk04816_regs_t::CLKOUT6_7_OSCIN_SEL_VCO;
    // Register 4
    _lmk04816_regs.CLKout8_9_DIV = vco_div;
    // Register 5
    _lmk04816_regs.CLKout10_11_PD = lmk04816_regs_t::CLKOUT10_11_PD_NORMAL;
    _lmk04816_regs.CLKout10_11_DIV = vco_div;

    // Register 6
    _lmk04816_regs.CLKout0_TYPE = lmk04816_regs_t::CLKOUT0_TYPE_LVDS; //FPGA
    _lmk04816_regs.CLKout2_TYPE = lmk04816_regs_t::CLKOUT2_TYPE_LVPECL_700MVPP; //DB_0_RX
    _lmk04816_regs.CLKout3_TYPE = lmk04816_regs_t::CLKOUT3_TYPE_LVPECL_700MVPP; //DB_1_RX
    // Register 7
    _lmk04816_regs.CLKout4_TYPE = lmk04816_regs_t::CLKOUT4_TYPE_LVPECL_700MVPP; //DB_1_TX
    _lmk04816_regs.CLKout5_TYPE = lmk04816_regs_t::CLKOUT5_TYPE_LVPECL_700MVPP; //DB_1_TX
    _lmk04816_regs.CLKout6_TYPE = lmk04816_regs_t::CLKOUT6_TYPE_LVPECL_700MVPP; // DB1_DAC
    _lmk04816_regs.CLKout7_TYPE = lmk04816_regs_t::CLKOUT7_TYPE_LVPECL_700MVPP; // DB1_DAC
    _lmk04816_regs.CLKout8_TYPE = lmk04816_regs_t::CLKOUT8_TYPE_LVPECL_700MVPP; // DB0_ADC
    // Register 8
    _lmk04816_regs.CLKout9_TYPE = lmk04816_regs_t::CLKOUT9_TYPE_LVPECL_700MVPP; //DB1_ADC
    _lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVDS; //REF_CLKOUT


    // Register 10
    _lmk04816_regs.EN_OSCout0 = lmk04816_regs_t::EN_OSCOUT0_ENABLED;
    _lmk04816_regs.FEEDBACK_MUX = 0; //use output 0 (FPGA clock) for feedback
    _lmk04816_regs.EN_FEEDBACK_MUX = lmk04816_regs_t::EN_FEEDBACK_MUX_ENABLED;

    // Register 11
    _lmk04816_regs.MODE = lmkmode;
    _lmk04816_regs.SYNC_QUAL = lmk04816_regs_t::SYNC_QUAL_FB_MUX;
    _lmk04816_regs.EN_SYNC = lmk04816_regs_t::EN_SYNC_ENABLE;
    _lmk04816_regs.NO_SYNC_CLKout0_1 = lmk04816_regs_t::NO_SYNC_CLKOUT0_1_CLOCK_XY_SYNC;
    _lmk04816_regs.NO_SYNC_CLKout2_3 = lmk04816_regs_t::NO_SYNC_CLKOUT2_3_CLOCK_XY_SYNC;
    _lmk04816_regs.NO_SYNC_CLKout4_5 = lmk04816_regs_t::NO_SYNC_CLKOUT4_5_CLOCK_XY_SYNC;
    _lmk04816_regs.NO_SYNC_CLKout8_9 = lmk04816_regs_t::NO_SYNC_CLKOUT8_9_CLOCK_XY_SYNC;
    _lmk04816_regs.NO_SYNC_CLKout10_11 = lmk04816_regs_t::NO_SYNC_CLKOUT10_11_CLOCK_XY_SYNC;
    _lmk04816_regs.SYNC_EN_AUTO = lmk04816_regs_t::SYNC_EN_AUTO_SYNC_INT_GEN;
    _lmk04816_regs.SYNC_POL_INV = lmk04816_regs_t::SYNC_POL_INV_SYNC_LOW;
    _lmk04816_regs.SYNC_TYPE = lmk04816_regs_t::SYNC_TYPE_INPUT;

    // Register 12
    _lmk04816_regs.LD_MUX = lmk04816_regs_t::LD_MUX_BOTH;

    /* Input Clock Configurations */
    // Register 13
    _lmk04816_regs.EN_CLKin0 = lmk04816_regs_t::EN_CLKIN0_NO_VALID_USE;
    _lmk04816_regs.EN_CLKin2 = lmk04816_regs_t::EN_CLKIN2_NO_VALID_USE;
    _lmk04816_regs.Status_CLKin1_MUX = lmk04816_regs_t::STATUS_CLKIN1_MUX_UWIRE_RB;
    _lmk04816_regs.CLKin_Select_MODE = lmk04816_regs_t::CLKIN_SELECT_MODE_CLKIN1_MAN;
    _lmk04816_regs.HOLDOVER_MUX = lmk04816_regs_t::HOLDOVER_MUX_PLL1_R;
    // Register 14
    _lmk04816_regs.Status_CLKin1_TYPE = lmk04816_regs_t::STATUS_CLKIN1_TYPE_OUT_PUSH_PULL;
    _lmk04816_regs.Status_CLKin0_TYPE = lmk04816_regs_t::STATUS_CLKIN0_TYPE_OUT_PUSH_PULL;

    // Register 26
    _lmk04816_regs.PLL2_CP_GAIN_26 =  pll_2_cp_gain;
    _lmk04816_regs.PLL2_CP_POL_26 = lmk04816_regs_t::PLL2_CP_POL_26_NEG_SLOPE;
    _lmk04816_regs.EN_PLL2_REF_2X = lmk04816_regs_t::EN_PLL2_REF_2X_DOUBLED_FREQ_REF;

    // Register 27
    _lmk04816_regs.PLL1_CP_GAIN_27 = pll_1_cp_gain;
    _lmk04816_regs.PLL1_R_27 = pll_1_r_div;
    // Register 28
    _lmk04816_regs.PLL1_N_28 = pll_1_n_div;
    _lmk04816_regs.PLL2_R_28 = pll_2_r_div;
    // Register 29
    _lmk04816_regs.PLL2_N_CAL_29 = pll_2_n_div;
    _lmk04816_regs.OSCin_FREQ_29 = lmk04816_regs_t::OSCIN_FREQ_29_63_TO_127MHZ;
    // Register 30
    _lmk04816_regs.PLL2_P_30 = lmk04816_regs_t::PLL2_P_30_DIV_2A;
    _lmk04816_regs.PLL2_N_30 = pll_2_n_div;

    /* Write the configuration values into the LMK */
    for (size_t i = 1; i <= 16; ++i) {
        this->write_regs(i);
    }
    for (size_t i = 24; i <= 31; ++i) {
        this->write_regs(i);
    }

    this->sync_clocks();
}

double get_sysref_clock_rate(void) {
    // TODO
}

std::string get_sysref_source(void) {
    // TODO
}

double get_extref_clock_rate(void) {
    // TODO
}

double get_refout_clock_rate(void) {
    // TODO
}

void set_rate(const x300_clock_which_t which, double rate) {
    /* TODO */
}

std::vector<double> get_rates(const x300_clock_which_t) {
    std::vector<double> rates;
    rates.push_back(get_master_clock_rate());
    return rates;
}


void set_ref_out(const bool enb) {
    if (enb) _lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVDS; //REF_CLKOUT
    else _lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_P_DOWN; //REF_CLKOUT
    this->write_regs(8);
}


void write_regs(boost::uint8_t addr) {
    boost::uint32_t data = _lmk04816_regs.get_reg(addr);
    if (addr==10) data |= 0x03000000; // jk GIANT HACK enables OSCout LVPECL
        _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, data,32);
        //for testing purposes
        //printf("%u %08x\n", addr, data);
    }

    //uhd::dict<x300_clock_which_t, bool> _enables;
    //uhd::dict<x300_clock_which_t, double> _rates;

    /*
    //read_reg: read a single register to the spi regs.

        void read_reg(boost::uint8_t addr)    {

            boost::unint32_t data = _lmk04816_regs.get_read_reg(addr);
            _spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, data , 32);
        }
    */
    //future implementations for modularity

private:
    const spi_iface::sptr _spiface;
    const size_t _slaveno;
    const size_t _hw_rev;
    const double _master_clock_rate;
    const double _system_ref_rate;
    lmk04816_regs_t _lmk04816_regs;
};

x300_clock_ctrl::sptr x300_clock_ctrl::make(uhd::spi_iface::sptr spiface,
        const size_t slaveno,
        const size_t hw_rev,
        const double master_clock_rate,
        const double system_ref_rate) {
    return sptr(new x300_clock_ctrl_impl(spiface, slaveno, hw_rev,
                master_clock_rate, system_ref_rate));
}
