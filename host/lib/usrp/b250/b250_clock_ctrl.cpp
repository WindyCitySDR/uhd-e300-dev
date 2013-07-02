

#include "b250_clock_ctrl.hpp"
#include "b250_impl.hpp"
#include <uhd/utils/safe_call.hpp>
#include <boost/cstdint.hpp>
#include <lmk04816_regs.hpp>
using namespace uhd;

struct b250_clock_ctrl_impl : b250_clock_ctrl	{

	b250_clock_ctrl_impl(uhd::spi_iface::sptr spiface, const size_t slaveno): _spiface(spiface), _slaveno(slaveno)	{

/* Individual Clock Output Configurations */		
//register 0

		//set divide value for DAC
		_lmk04816_regs.CLKout0_1_DIV = 20;
		this->write_regs(0);
		
//register 1
		//set divide value for ADC
		_lmk04816_regs.CLKout2_3_DIV = 20;
		this->write_regs(1);
//register 2
		//set divide value for ADC
		_lmk04816_regs.CLKout4_5_DIV = 20;
		this->write_regs(3);
//register 3
		//set divide value for FPGA
		_lmk04816_regs.CLKout6_7_DIV = 20;
		this->write_regs(3);
//register 4
		//set divide value for FPGA
		_lmk04816_regs.CLKout8_9_DIV = 20;
		this->write_regs(4);
//register 5
		//set divide value for LVDS low frequency system synch clock
		_lmk04816_regs.CLKout10_11_DIV = 20;
		this->write_regs(5);

/* Output Clock Type Configurations */

//register 6
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout0_TYPE = 1; //FPGA
		_lmk04816_regs.CLKout2_TYPE = lmk04816_regs_t::CLKOUT2_TYPE_LVPECL_700MVPP; //DB_0_RX
		_lmk04816_regs.CLKout3_TYPE = lmk04816_regs_t::CLKOUT3_TYPE_LVPECL_700MVPP; //DB_1_RX
		this->write_regs(6);
//register 7
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout4_TYPE = 2; //DB_1_TX
		_lmk04816_regs.CLKout5_TYPE = lmk04816_regs_t::CLKOUT5_TYPE_LVPECL_700MVPP; //REF_CLKOUT
		//sets clock type to LVDS
		_lmk04816_regs.CLKout6_TYPE = lmk04816_regs_t::CLKOUT6_TYPE_LVPECL_700MVPP; // DB1_DAC
		_lmk04816_regs.CLKout7_TYPE = lmk04816_regs_t::CLKOUT7_TYPE_LVPECL_700MVPP; // DB1_DAC
		_lmk04816_regs.CLKout8_TYPE = 2; // DB0_ADC
		this->write_regs(7);
//register 8 
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout9_TYPE = lmk04816_regs_t::CLKOUT9_TYPE_LVPECL_700MVPP; //DB1_ADC
		_lmk04816_regs.CLKout10_TYPE = lmk04816_regs_t::CLKOUT10_TYPE_LVPECL_700MVPP; //DB_0_TX
		this->write_regs(8);

/* Input Clock Configurations */

//register 13

		//disable clockin0 and clockin2 for testing
		_lmk04816_regs.EN_CLKin0 = lmk04816_regs_t::EN_CLKIN0_NO_VALID_USE;
		_lmk04816_regs.EN_CLKin2 = lmk04816_regs_t::EN_CLKIN2_NO_VALID_USE;
		this->write_regs(13);

/*Loop Filter settings*/

//register 24 - sets C4, C3, R4		
 
/* Divider value setting*/

//register 27

		
		//_lmk04816_regs.CLKin0_PreR_DIV = 1;
		_lmk04816_regs.CLKin1_PreR_DIV = lmk04816_regs_t::CLKIN1_PRER_DIV_DIV2;
		//sets PLL1_R value
		_lmk04816_regs.PLL1_R_27 = 3;
		//_lmk04816_regs.CLKin2_PreR_DIV = 1;
		this->write_regs(27);
//register 28
		//set PLL_1_N value
		_lmk04816_regs.PLL1_N_28 = 3;
		//set PLL_2_R value
		_lmk04816_regs.PLL2_R_28 = 1;
		this->write_regs(28);
//register 29
		//set the PLL_2_N value (calibration divider)
		//_lmk04816_regs.PLL2_N_CAL = 10;
		this->write_regs(29);
//register 30
		//sets PLL_2_N divider prescaler
		_lmk04816_regs.PLL2_P_30 = lmk04816_regs_t::PLL2_P_30_DIV_2A;
		//sets PLL2_N_divider
		_lmk04816_regs.PLL2_N_3 = 10;
		this->write_regs(30);

		

	}

//empty destructor for testing
	~b250_clock_ctrl_impl(void)	{}
	
//master rate
	double get_master_clock_rate(void) {
		
		return B250_RADIO_CLOCK_RATE;
	}
//empty
	void enable_clock(const b250_clock_which_t which, const bool enb) {}
	
	void set_rate(const b250_clock_which_t which, double rate) {}

	std::vector<double> get_rates(const b250_clock_which_t) {
		
		std::vector<double> rates;
		rates.push_back(get_master_clock_rate());
		return rates;
	} 






//write_reg: write a single register to the spi regs.
	void write_regs(boost::uint8_t addr)	{

		boost::uint32_t data = _lmk04816_regs.get_reg(addr);
		_spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, data,32);
	}

	const spi_iface::sptr _spiface;
	const size_t _slaveno;
	lmk04816_regs_t _lmk04816_regs;
	uhd::dict<b250_clock_which_t, bool> _enables;
	uhd::dict<b250_clock_which_t, double> _rates;
/*
//read_reg: read a single register to the spi regs.

	void read_reg(boost::uint8_t addr)	{
		
		boost::unint32_t data = _lmk04816_regs.get_read_reg(addr);
		_spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, data , 32);
	}
*/	
//future implementations for modularity


};

b250_clock_ctrl::sptr b250_clock_ctrl::make(uhd::spi_iface::sptr spiface, const size_t slaveno)	{
	return sptr(new b250_clock_ctrl_impl(spiface,slaveno));
}




