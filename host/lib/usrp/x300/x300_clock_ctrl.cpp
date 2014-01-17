
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
        const double clock_rate,
        const int &revno,
        const double pll2ref,
        const double refclk_rate):
        _spiface(spiface),
        _slaveno(slaveno),
        _clock_rate(clock_rate),
        _revno(revno),
        _lmkpll2_ref(pll2ref),
        _refclk_rate(refclk_rate)
{
    bool pll2ref96M = (pll2ref == 96e6);
    //bool refclk10M = (refclk_rate == 10e6);
    bool clkis_184_32 = (clock_rate == 184.32e6);

    if (!pll2ref96M && pll2ref != 120e6)
        throw uhd::runtime_error(str(boost::format("x300_clock_ctrl: unsupported pll2 reference %f") % pll2ref));

    /* for non-CPRI clocking, the VCO is run at 2400 MHz, div is the
        * required output divide for the CPRI rate of 184.32 MHz, the VCO is
        * run at 2580.48 MHz and div is 14 */
    int div = 0;

    /*
    if (clock_rate == 120e6) div = 20;
    else if (clock_rate == 150e6) div = 16;
    else if (clock_rate == 200e6) div = 12;
    else throw uhd::runtime_error(str(boost::format("x300_clock_ctrl: can't handle rate %f") % clock_rate));
    */
    std::cerr << "+++++++ jk clock settings debug  " << pll2ref << ' '
        << refclk_rate << ' ' << clock_rate << std::endl;

    enum opmode {  m10M_200M_NOZD,
                    m10M_200M_ZDEL,
                    m30_72M_184_32M_ZDEL,
                    m10M_184_32M_NOZD };

    opmode mode = m10M_200M_ZDEL;

    std::string opmode_str = std::string(getenv("X300_OPMODE"));
    if(opmode_str == "m10M_200M_NOZD") {
        mode = m10M_200M_NOZD;
    } else if(opmode_str == "m10M_200M_ZDEL") {
        mode = m10M_200M_ZDEL;
    } else if(opmode_str == "m30_72M_184_32M_ZDEL") {
        mode = m30_72M_184_32M_ZDEL;
    } else if(opmode_str == "m10M_184_32M_NOZD") {
        mode = m10M_184_32M_NOZD;
    } else {
        std::cerr << "Could not parse OPMODE." << std::endl;
    }

    std::cerr << "+++++++ opmode:  " << opmode_str << ": " << mode << std::endl;

    lmk04816_regs_t::MODE_t lmkmode;

    int pll_1_n_div;
            int pll_1_r_div = 1;
            lmk04816_regs_t::PLL1_CP_GAIN_27_t pll_1_cp_gain;

    int pll_2_n_div;
            int pll_2_r_div;
            lmk04816_regs_t::PLL2_CP_GAIN_26_t pll_2_cp_gain;


    // Note: PLL2 N2 prescaler is enabled for all cases
    //       PLL2 reference doubler is enabled for all cases
    switch (mode) {
        case m10M_200M_NOZD: // 10 MHz reference, 200 MHz out, not zero-delay mode
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT;
            pll_1_n_div = 48;
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 25;
            pll_2_r_div = 4;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            div = 12;
            break;
        case m10M_200M_ZDEL: // 10 MHz reference, 200 MHz out
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
            pll_1_n_div = 100; //int(clock_rate/refclk_rate);
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 25;
            pll_2_r_div = 4;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            div = 12;
            break;
        case m30_72M_184_32M_ZDEL: // CPRI reference, 184.32 MHz out
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
            pll_1_n_div = 90; //int(clock_rate/refclk_rate);
            pll_1_r_div = 15;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 168;
            pll_2_r_div = 25;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            div=14;
            //  TODO: add these for the 184.23 cases only
            _lmk04816_regs.PLL2_R3_LF=lmk04816_regs_t:: PLL2_R3_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_R4_LF=lmk04816_regs_t:: PLL2_R4_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_C3_LF=lmk04816_regs_t:: PLL2_C3_LF_39PF;
            _lmk04816_regs.PLL2_C4_LF=lmk04816_regs_t:: PLL2_C4_LF_34PF;
            break;
        case m10M_184_32M_NOZD: // 10 MHz reference, 184.32 MHz out
            lmkmode  = lmk04816_regs_t::MODE_DUAL_INT;
            pll_1_n_div = 48;
            pll_1_r_div = 5;
            pll_1_cp_gain = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
            pll_2_n_div = 168;
            pll_2_r_div = 25;
            pll_2_cp_gain = lmk04816_regs_t::PLL2_CP_GAIN_26_3200UA;
            div=14;
            // TODO: add these for the 184.23 cases only
            _lmk04816_regs.PLL2_R3_LF=lmk04816_regs_t:: PLL2_R3_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_R4_LF=lmk04816_regs_t:: PLL2_R4_LF_1KILO_OHM;
            _lmk04816_regs.PLL2_C3_LF=lmk04816_regs_t:: PLL2_C3_LF_39PF;
            _lmk04816_regs.PLL2_C4_LF=lmk04816_regs_t:: PLL2_C4_LF_34PF;
            break;
        };

    //calculate N div -- ok as long as integer multiple of 10e6
    //int pll_1_n_div = int(clock_rate/10e6);

    std::cerr << "jk N1=" << pll_1_n_div << " R1=" << pll_1_r_div
        << " N2=" << pll_2_n_div << " R2=" << pll_2_r_div << " div="
        << div << std::endl;

    /* Individual Clock Output Configurations */

    //register 0
    _lmk04816_regs.RESET = lmk04816_regs_t::RESET_RESET;
    this->write_regs(0);
    _lmk04816_regs.RESET = lmk04816_regs_t::RESET_NO_RESET;
    this->write_regs(0);
    _lmk04816_regs.CLKout0_1_PD = lmk04816_regs_t::CLKOUT0_1_PD_POWER_UP;
    this->write_regs(0);
    _lmk04816_regs.CLKout0_1_DIV = div;
    this->write_regs(0);

    //register 1
    _lmk04816_regs.CLKout2_3_PD = lmk04816_regs_t::CLKOUT2_3_PD_POWER_UP;
    _lmk04816_regs.CLKout2_3_DIV = div;
    //this->write_regs(1);
    //register 2
    _lmk04816_regs.CLKout4_5_PD = lmk04816_regs_t::CLKOUT4_5_PD_POWER_UP;
    _lmk04816_regs.CLKout4_5_DIV = div;
    //this->write_regs(2);
    //register 3
    _lmk04816_regs.CLKout6_7_DIV = div;
    _lmk04816_regs.CLKout6_7_OSCin_Sel = lmk04816_regs_t::CLKOUT6_7_OSCIN_SEL_VCO;
    //this->write_regs(3);
    //register 4
    _lmk04816_regs.CLKout8_9_DIV = div;
    //this->write_regs(4);
    //register 5
    _lmk04816_regs.CLKout10_11_PD = lmk04816_regs_t::CLKOUT10_11_PD_NORMAL;
    _lmk04816_regs.CLKout10_11_DIV = div;
    //this->write_regs(5);

    /* Output Clock Type Configurations */

    //register 6
    //sets clock type to LVPECL
    _lmk04816_regs.CLKout0_TYPE = lmk04816_regs_t::CLKOUT0_TYPE_LVDS; //FPGA
    _lmk04816_regs.CLKout2_TYPE = lmk04816_regs_t::CLKOUT2_TYPE_LVPECL_700MVPP; //DB_0_RX
    _lmk04816_regs.CLKout3_TYPE = lmk04816_regs_t::CLKOUT3_TYPE_LVPECL_700MVPP; //DB_1_RX

    //register 7
    //sets clock type to LVPECL
    _lmk04816_regs.CLKout4_TYPE = lmk04816_regs_t::CLKOUT4_TYPE_LVPECL_700MVPP; //DB_1_TX
    _lmk04816_regs.CLKout5_TYPE = lmk04816_regs_t::CLKOUT5_TYPE_LVPECL_700MVPP; //DB_1_TX
    //sets clock type to LVDS
    _lmk04816_regs.CLKout6_TYPE = lmk04816_regs_t::CLKOUT6_TYPE_LVPECL_700MVPP; // DB1_DAC
    _lmk04816_regs.CLKout7_TYPE = lmk04816_regs_t::CLKOUT7_TYPE_LVPECL_700MVPP; // DB1_DAC
    _lmk04816_regs.CLKout8_TYPE = lmk04816_regs_t::CLKOUT8_TYPE_LVPECL_700MVPP; // DB0_ADC
    //this->write_regs(7);
    //register 8
    //sets clock type to LVPECL
    _lmk04816_regs.CLKout9_TYPE = lmk04816_regs_t::CLKOUT9_TYPE_LVPECL_700MVPP; //DB1_ADC
    _lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVDS; //REF_CLKOUT
    //this->write_regs(8);

    /*
    for (size_t i = 1; i <= 8; ++i)    {
        this->write_regs(i);
        }
    */

    /////////////////////////////////////////////////////////////////////////
    //BEGIN CONDITIONAL DIVIDERS AND CLOCKOUT TYPES FOR REVISION DIFFERENCES
    /////////////////////////////////////////////////////////////////////////

    if (_revno < 3) {
        _lmk04816_regs.CLKout5_TYPE = lmk04816_regs_t::CLKOUT5_TYPE_LVDS; //REF_CLKOUT
        _lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVPECL_700MVPP; //DB_0_TX
    } else {
        //_lmk04816_regs.CLKout10_11_DIV = int(((clock_rate*div)/10e6) + 0.5);
        _lmk04816_regs.CLKout10_11_DIV = div; // jk
    }

    /////////////////////////////////////////////////////////////////////////
    //END CONDITIONAL DIVIDERS AND CLOCKOUT TYPES FOR REVISION DIFFERENCES
    /////////////////////////////////////////////////////////////////////////

    //register 10

    //_lmk04816_regs.EN_OSCout0 = lmk04816_regs_t::EN_OSCOUT0_DISABLED;
    _lmk04816_regs.EN_OSCout0 = lmk04816_regs_t::EN_OSCOUT0_ENABLED;
    //_lmk04816_regs.OSCout0_TTYPE = lmk04816_regs_t::OSCout0_TYPE_t::CLKOUT1_TYPE_LVPECL_700MVPP;
    std::cerr << "+++++++ jk CAUTION: OSCOUT is ENABLED!!" << std::endl;

    _lmk04816_regs.FEEDBACK_MUX = 0; //use output 0 (FPGA clock) for feedback
    _lmk04816_regs.EN_FEEDBACK_MUX = lmk04816_regs_t::EN_FEEDBACK_MUX_ENABLED;

    //register 11

    //register 11 sync enabled
    //_lmk04816_regs.MODE = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
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
    //register 12

    //enabling LD_MUX
    _lmk04816_regs.LD_MUX = lmk04816_regs_t::LD_MUX_BOTH;

    /* Input Clock Configurations */

    //register 13

    //disable clockin0 and clockin2 for testing
    _lmk04816_regs.EN_CLKin0 = lmk04816_regs_t::EN_CLKIN0_NO_VALID_USE;
    _lmk04816_regs.EN_CLKin2 = lmk04816_regs_t::EN_CLKIN2_NO_VALID_USE;
    _lmk04816_regs.Status_CLKin1_MUX = lmk04816_regs_t::STATUS_CLKIN1_MUX_UWIRE_RB;
    //this->write_regs(13);
    //manual select for Clk 1
    _lmk04816_regs.CLKin_Select_MODE = lmk04816_regs_t::CLKIN_SELECT_MODE_CLKIN1_MAN;
    //this->write_regs(13);
    _lmk04816_regs.HOLDOVER_MUX = lmk04816_regs_t::HOLDOVER_MUX_PLL1_R; //needed for fpga 10MHz ref in
    //sleep(1000);

    //register 14
    _lmk04816_regs.Status_CLKin1_TYPE = lmk04816_regs_t::STATUS_CLKIN1_TYPE_OUT_PUSH_PULL;
    //this->write_regs(14);
    _lmk04816_regs.Status_CLKin0_TYPE = lmk04816_regs_t::STATUS_CLKIN0_TYPE_OUT_PUSH_PULL;

    /*Loop Filter settings*/

    //register 24 - sets C4, C3, R4

    /* Divider value setting*/

    //Register 26

    //_lmk04816_regs.PLL2_CP_GAIN_26 = pll2ref96M ? lmk04816_regs_t::PLL2_CP_GAIN_26_400UA : lmk04816_regs_t::PLL2_CP_GAIN_26_1600UA;
    _lmk04816_regs.PLL2_CP_GAIN_26 =  pll_2_cp_gain;
    _lmk04816_regs.PLL2_CP_POL_26 = lmk04816_regs_t::PLL2_CP_POL_26_NEG_SLOPE;
    // if (clkis_184_32)
            _lmk04816_regs.EN_PLL2_REF_2X = lmk04816_regs_t::EN_PLL2_REF_2X_DOUBLED_FREQ_REF; // jk
    //else
    //_lmk04816_regs.EN_PLL2_REF_2X = lmk04816_regs_t::EN_PLL2_REF_2X_NORMAL_FREQ_REF; // jk


    //register 27

    //_lmk04816_regs.PLL1_CP_GAIN_27 = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
    _lmk04816_regs.PLL1_CP_GAIN_27 = pll_1_cp_gain;
    //_lmk04816_regs.CLKin1_PreR_DIV = 1;
    //_lmk04816_regs.CLKin1_PreR_DIV = lmk04816_regs_t::CLKIN1_PRER_DIV_DIV2;
    //sets PLL1_R value
    //_lmk04816_regs.PLL1_R_27 = 1;
    _lmk04816_regs.PLL1_R_27 = pll_1_r_div;
    //_lmk04816_regs.CLKin2_PreR_DIV = 1;
    //this->write_regs(27);
    //register 28
    //set PLL_1_N value
    _lmk04816_regs.PLL1_N_28 = pll_1_n_div;
    //set PLL_2_R value
    _lmk04816_regs.PLL2_R_28 = pll_2_r_div;
    //this->write_regs(28);
    //register 29
    //set the PLL_2_N value (calibration divider)
    _lmk04816_regs.PLL2_N_CAL_29 = pll_2_n_div;
            //if (pll2ref96M) _lmk04816_regs.OSCin_FREQ_29 = lmk04816_regs_t::OSCIN_FREQ_29_63_TO_127MHZ;
            _lmk04816_regs.OSCin_FREQ_29 = lmk04816_regs_t::OSCIN_FREQ_29_63_TO_127MHZ;
    //this->write_regs(29);
    //register 30
    //sets PLL_2_N divider prescaler
    _lmk04816_regs.PLL2_P_30 = lmk04816_regs_t::PLL2_P_30_DIV_2A;
    //sets PLL2_N_divider
    _lmk04816_regs.PLL2_N_30 = pll_2_n_div;
    //this->write_regs(30);



    for (size_t i = 1; i <= 16; ++i) {
        if (i==10)
            std::cerr << "        REG 10 is " << std::hex << this->_lmk04816_regs.get_reg(i) << std::endl;
        this->write_regs(i);
    }

    for (size_t i = 24; i <= 31; ++i) {
        this->write_regs(i);
    }

    this->sync_clocks();
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

    return _clock_rate;
}

double get_crystal_clock_rate(void) {

    return _lmkpll2_ref;
}

void enable_clock(const x300_clock_which_t which, const bool enb) {
    /* TODO */
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
    if (_revno < 3) return; //not supported on this hardware
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

    const spi_iface::sptr _spiface;
    const size_t _slaveno;
    const double _clock_rate;
    const int _revno;
    const double _lmkpll2_ref;
    const double _refclk_rate;
    lmk04816_regs_t _lmk04816_regs;
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
};

x300_clock_ctrl::sptr x300_clock_ctrl::make(uhd::spi_iface::sptr spiface,
        const size_t slaveno,
        const double clock_rate,
        const int &revno,
        const double pll2ref,
        const double refclk_freq) {
    return sptr(new x300_clock_ctrl_impl(spiface, slaveno, clock_rate, revno, pll2ref, refclk_freq));
}
