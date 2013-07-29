
#include <cstdio>
#include "b250_clock_ctrl.hpp"
#include <uhd/utils/safe_call.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#include <lmk04816_regs.hpp>
#include <stdexcept>

using namespace uhd;
struct b250_clock_ctrl_impl : b250_clock_ctrl	{

	b250_clock_ctrl_impl(uhd::spi_iface::sptr spiface, const size_t slaveno, const double clock_rate):
		_spiface(spiface), _slaveno(slaveno), _clock_rate(clock_rate)
	{

		int div = 0;
		if (clock_rate == 120e6) div = 20;
		else if (clock_rate == 150e6) div = 16;
		else if (clock_rate == 200e6) div = 12;
		else throw uhd::runtime_error(str(boost::format("b250_clock_ctrl: cant handle rate %f") % clock_rate));

		//calculate N div -- ok as long as integer multiple of 10e6
		const int pll_1_n_div = int(clock_rate/10e6);

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
		//set divide value for ADC
		_lmk04816_regs.CLKout2_3_DIV = div;
		//this->write_regs(1);
//register 2
		_lmk04816_regs.CLKout4_5_PD = lmk04816_regs_t::CLKOUT4_5_PD_POWER_UP;
		//set divide value for ADC
		_lmk04816_regs.CLKout4_5_DIV = div;
		//this->write_regs(2);
//register 3
		//set divide value for FPGA
		_lmk04816_regs.CLKout6_7_DIV = div;
		_lmk04816_regs.CLKout6_7_OSCin_Sel = lmk04816_regs_t::CLKOUT6_7_OSCIN_SEL_VCO;
		//this->write_regs(3);
//register 4
		//set divide value for FPGA
		_lmk04816_regs.CLKout8_9_DIV = div;
		//this->write_regs(4);
//register 5
		_lmk04816_regs.CLKout10_11_PD = lmk04816_regs_t::CLKOUT10_11_PD_NORMAL;
		//set divide value for LVDS low frequency system synch clock
		_lmk04816_regs.CLKout10_11_DIV = div;
		//this->write_regs(5);

/* Output Clock Type Configurations */

//register 6
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout0_TYPE = 1; //FPGA
		_lmk04816_regs.CLKout2_TYPE = lmk04816_regs_t::CLKOUT2_TYPE_LVPECL_700MVPP; //DB_0_RX
		_lmk04816_regs.CLKout3_TYPE = lmk04816_regs_t::CLKOUT3_TYPE_LVPECL_700MVPP; //DB_1_RX
		
//register 7
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout4_TYPE = 2; //DB_1_TX
		_lmk04816_regs.CLKout5_TYPE = lmk04816_regs_t::CLKOUT5_TYPE_LVDS; //REF_CLKOUT
		//sets clock type to LVDS
		_lmk04816_regs.CLKout6_TYPE = lmk04816_regs_t::CLKOUT6_TYPE_LVPECL_700MVPP; // DB1_DAC
		_lmk04816_regs.CLKout7_TYPE = lmk04816_regs_t::CLKOUT7_TYPE_LVPECL_700MVPP; // DB1_DAC
		_lmk04816_regs.CLKout8_TYPE = 2; // DB0_ADC
		//this->write_regs(7);
//register 8 
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout9_TYPE = lmk04816_regs_t::CLKOUT9_TYPE_LVPECL_700MVPP; //DB1_ADC
		_lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVPECL_700MVPP; //DB_0_TX
		//this->write_regs(8);
/*
	for (size_t i = 1; i <= 8; ++i)	{ 
		this->write_regs(i);
		}
*/

//register 10

		_lmk04816_regs.EN_OSCout0 = lmk04816_regs_t::EN_OSCOUT0_DISABLED;
		_lmk04816_regs.FEEDBACK_MUX = 0; //use output 0 (FPGA clock) for feedback
		_lmk04816_regs.EN_FEEDBACK_MUX = lmk04816_regs_t::EN_FEEDBACK_MUX_ENABLED;

//register 11

		//register 11 sync enabled
		_lmk04816_regs.MODE = lmk04816_regs_t::MODE_DUAL_INT_ZER_DELAY;
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

		//

/*Loop Filter settings*/

//register 24 - sets C4, C3, R4		
 
/* Divider value setting*/

//Register 26

		_lmk04816_regs.PLL2_CP_GAIN_26 = lmk04816_regs_t::PLL2_CP_GAIN_26_1600UA;
		_lmk04816_regs.PLL2_CP_POL_26 = lmk04816_regs_t::PLL2_CP_POL_26_NEG_SLOPE;

//register 27

		_lmk04816_regs.PLL1_CP_GAIN_27 = lmk04816_regs_t::PLL1_CP_GAIN_27_100UA;
		//_lmk04816_regs.CLKin1_PreR_DIV = 1;
		//_lmk04816_regs.CLKin1_PreR_DIV = lmk04816_regs_t::CLKIN1_PRER_DIV_DIV2;
		//sets PLL1_R value
		_lmk04816_regs.PLL1_R_27 = 1;
		//_lmk04816_regs.CLKin2_PreR_DIV = 1;
		//this->write_regs(27);
//register 28
		//set PLL_1_N value
		_lmk04816_regs.PLL1_N_28 = pll_1_n_div*_lmk04816_regs.PLL1_R_27;
		//set PLL_2_R value
		_lmk04816_regs.PLL2_R_28 = 1;
		//this->write_regs(28);
//register 29
		//set the PLL_2_N value (calibration divider)
		_lmk04816_regs.PLL2_N_CAL_29 = 10;
		//this->write_regs(29);
//register 30
		//sets PLL_2_N divider prescaler
		_lmk04816_regs.PLL2_P_30 = lmk04816_regs_t::PLL2_P_30_DIV_2A;
		//sets PLL2_N_divider
		_lmk04816_regs.PLL2_N_30 = 10;
		//this->write_regs(30);
		
		for (size_t i = 1; i <= 16; ++i) { 
			this->write_regs(i);
		}
		for (size_t i = 24; i <= 31; ++i) {
                        this->write_regs(i);
                }

		this->sync_clocks();

	}


	void sync_clocks(void)
	{
		//soft sync:
		//put the sync IO into output mode - FPGA must be input
		//write low, then write high - this triggers a soft sync
		_lmk04816_regs.SYNC_POL_INV = lmk04816_regs_t::SYNC_POL_INV_SYNC_LOW;
		this->write_regs(11);
		_lmk04816_regs.SYNC_POL_INV = lmk04816_regs_t::SYNC_POL_INV_SYNC_HIGH;
		this->write_regs(11);
 	}

//empty destructor for testing
	~b250_clock_ctrl_impl(void)	{}
	
//master rate
	double get_master_clock_rate(void) {
		
		return _clock_rate;
	}
//empty
	void enable_clock(const b250_clock_which_t which, const bool enb) {}
	
	void set_rate(const b250_clock_which_t which, double rate) {}

	std::vector<double> get_rates(const b250_clock_which_t) {
		
		std::vector<double> rates;
		rates.push_back(get_master_clock_rate());
		return rates;
	} 


	void set_ref_out(const bool)
	{
		//TODO -- always on until we rev the HW
	}


//write_reg: write a single register to the spi regs.
	void write_regs(boost::uint8_t addr)	{

		boost::uint32_t data = _lmk04816_regs.get_reg(addr);
		_spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, data,32);
		//for testing purposes
		//printf("%u %08x\n", addr, data);
	}

	const spi_iface::sptr _spiface;
	const size_t _slaveno;
	const double _clock_rate;
	lmk04816_regs_t _lmk04816_regs;
	//uhd::dict<b250_clock_which_t, bool> _enables;
	//uhd::dict<b250_clock_which_t, double> _rates;
/*
//read_reg: read a single register to the spi regs.

	void read_reg(boost::uint8_t addr)	{
		
		boost::unint32_t data = _lmk04816_regs.get_read_reg(addr);
		_spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, data , 32);
	}
*/	
//future implementations for modularity


};

b250_clock_ctrl::sptr b250_clock_ctrl::make(uhd::spi_iface::sptr spiface, const size_t slaveno, const double clock_rate)	{
	return sptr(new b250_clock_ctrl_impl(spiface,slaveno,clock_rate));
}




