

#include "b250_clock_ctrl.hpp"
#include "b250impl.hpp"
#include <uhd/utils/safe_call.hpp>
#include <boost/cstdint.hpp>
#include <lmk04816_regs.hpp>
using namespace uhd;

struct b250_clock_ctrl_impl : b250_clock_ctrl	{

	b250_clock_ctrl_impl(uhd::spi_iface: sptr spiface, const size_t slaveno): _spiface(spiface), _slaveno(slaveno)	{

		
//register 0

		//set divide value for DAC
		_lmk04816_regs.CLKout0_1_div = 1;
		this->write_reg(0);
		
//register 1
		//set divide value for ADC
		_lmk04816_regs.CLKout2_3_div = 1;
		this->write_regs(1);
//register 2
		//set divide value for ADC
		_lmk04816_regs.CLKout_4_5_div = 1;
		this->write_regs(3);
//register 3
		//set divide value for FPGA
		_lmk04816_regs.CLKout_6_7_div = 1;
		this->write_regs(3);
//register 4
		//set divide value for FPGA
		_lmk04816_regs.CLKout_8_9_div = 1;
		this->write_regs(4);
//register 5
		//set divide value for LVDS low frequency system synch clock
		_lmk04816_regs.CLKout_10_11_div = 1;
		this->write_regs(5);
//register 6
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout0_TYPE = 2; //DAC
		_lmk04816_regs.CLKout1_TYPE = 2; //DAC
		_lmk04816_regs.CLKout2_TYPE = 2; //ADC
		_lmk04816_regs.CLKout3_TYPE = 2; //ADC
		this->write_regs(6);
//register 7
		//sets clock type to LVPECL
		_lmk04816_regs.CLKout4_TYPE = 2; //ADC
		_lmk04816_regs.CLKout5_TYPE = 2; //ADC
		//sets clock type to LVDS
		_lmk04816_regs.CLKout6_TYPE = 1; // FPGA
		_lmk04816_regs.CLKout7_TYPE = 1; // FPGA
		_lmk04816_regs.CLKout8_TYPE = 1; // FPGA
		this->write_regs(7);
//register 8 
		//sets clock type to LVDS
		_lmk04816_regs.CLKout10_TYPE = 1; //Low frequency system synch
		this->write_regs(8);
		
 
//register 27

		//set R values
		_lmk04816_regs.CLKin0_PreR_DIV = 1;
		_lmk04816_regs.CLKin1_PreR_DIV = 1;
		_lmk04816_regs.CLKin2_PreR_DIV = 1;
		this->write_regs(27);
//register 28
		//set PLL_1_N value
		_lmk04816_regs.PLL1_N_28 = 1;
		_lmk04816_regs.PLL2_R_28 = 1;
		this->write_regs(28);
//register 29
		//set the PLL_2_N value (calibration divider)
		_lmk04816_regs.PLL2_N_CAL = 1;
		this->write_regs(29);
//register 30
		//sets LL_2_N divider prescaler
		_lmk04816_regs.PLL2_P_30 = 1;
		_lmk04816_regs.PLL2_N_3 = 1;
		this->write_regs(30);

		

	}

//empty destructor for testing
	~b250_clock_ctrl(void)	{}




//write_reg: write a single register to the spi regs.
	void write_reg(boost::uint8_t addr)	{

		boost::uint32_t data = _lmk04816_regs.get_write_reg(addr);
		_spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, data,32);
	}
//read_reg: read a single register to the spi regs.

	void read_reg(boost::uint8_t addr)	{
		
		boost::unint32_t data = _lmk04816_regs.get_read_reg(addr);
		_spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, data , 32);
	}
	
//future implementations for modularity


};

b250_clock_ctrl::sptr b250_clock_ctrl::make(uhd::spi_iface::sptr spiface, const size_t slaveno)	{
	return sptr(new b250_clock_ctrl_impl(spiface,slaveno));
}




