//
// Copyright 2012 Ettus Research LLC
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

#include "b200_codec_ctrl.hpp"
#include <uhd/exception.hpp>
#include <iostream>

using namespace uhd;
using namespace uhd::transport;

/***********************************************************************
 * The implementation class
 **********************************************************************/
class b200_codec_ctrl_impl : public b200_codec_ctrl{
public:

    b200_codec_ctrl_impl(b200_iface::sptr iface)
    {
        _b200_iface = iface;

        //reset
        _b200_iface->write_reg(0x000,0x01);
        //clear reset
        _b200_iface->write_reg(0x000,0x00);

        //there is not a WAT big enough for this
        _b200_iface->write_reg(0x3df, 0x01);

        //bandgap setup
        _b200_iface->write_reg(0x2a6, 0x0e);
        _b200_iface->write_reg(0x2a8, 0x0e);

        //rfpll refclk to REFCLKx2
        _b200_iface->write_reg(0x2ab, 0x07);
        _b200_iface->write_reg(0x2ac, 0xff);
        _b200_iface->write_reg(0x009, 0b00010111);

        /**set up BBPLL*/
        //64Msps ADC clock means:
        //div 2 in RHB3 = 32Msps
        //div 2 in RHB2 = 16Msps
        //div 2 in RHB1 = 8Msps
        //div 2 in RXFIR = 4Msps output
        //eventually this should be done alongside FIR/HB setup
        set_adcclk(245.76e6); //for 15.36Msps
        //TX1/2 en, THB3 interp x2, THB2 interp x2 fil. en, THB1 en, TX FIR interp 2 en
        _b200_iface->write_reg(0x002, 0b01011110);
        //RX1/2 en, RHB3 decim x2, RHB2 decim x2 fil. en, RHB1 en, RX FIR decim 2 en
        _b200_iface->write_reg(0x003, 0b01011110);
        //select TX1A/TX2A, RX antennas in balanced mode on ch. A
        _b200_iface->write_reg(0x004, 0b00000011);

        //data delay for TX and RX data clocks
        _b200_iface->write_reg(0x006, 0x0F);
        _b200_iface->write_reg(0x007, 0x00);

        /********setup data ports (FDD dual port DDR CMOS)*/
        //FDD dual port DDR CMOS no swap
        _b200_iface->write_reg(0x010, 0b10101000); //FIXME 0b10101000 (swap TX IQ swap TX1/TX2)
        _b200_iface->write_reg(0x011, 0b00000000);
        _b200_iface->write_reg(0x012, 0b00000010); //force TX on one port, RX on the other
        _b200_iface->write_reg(0x013, 0b00000001); //enable ENSM
        _b200_iface->write_reg(0x014, 0b00100001); //use SPI for TXNRX ctrl, to alert, TX on
        _b200_iface->write_reg(0x015, 0b00000100); //dual synth mode, synth en ctrl en

        //setup TX FIR

        //setup RX FIR

        //set baseband filter BW
        set_filter_bw("TX_A", 6.0e6);
        set_filter_bw("RX_A", 6.0e6);

        /**initial VCO setup*/
        _b200_iface->write_reg(0x261,0x00); //RX LO power
        _b200_iface->write_reg(0x2a1,0x00); //TX LO power
        _b200_iface->write_reg(0x248,0x0b); //en RX VCO LDO
        _b200_iface->write_reg(0x288,0x0b); //en TX VCO LDO
        _b200_iface->write_reg(0x246,0x02); //pd RX cal Tcf
        _b200_iface->write_reg(0x286,0x02); //pd TX cal Tcf
        _b200_iface->write_reg(0x243,0x0d); //set rx prescaler bias
        _b200_iface->write_reg(0x283,0x0d); //"" TX
        _b200_iface->write_reg(0x245,0x00); //set RX VCO cal ref Tcf
        _b200_iface->write_reg(0x250,0x70); //set RX VCO varactor ref Tcf
        _b200_iface->write_reg(0x285,0x00); //"" TX
        _b200_iface->write_reg(0x290,0x70); //"" TX
        _b200_iface->write_reg(0x239,0xc1); //init RX ALC
        _b200_iface->write_reg(0x279,0xc1); //"" TX
        _b200_iface->write_reg(0x23b,0x89); //set RX MSB? //FIXME 0x89 magic charge pump current, undocumented
        _b200_iface->write_reg(0x27b,0x88); //"" TX //FIXME 0x88 see above
        _b200_iface->write_reg(0x23d,0x04); //clear RX 1/2 VCO cal clk //FIXME 0x04 enable CP cal, "only change if apps eng. says so"
        _b200_iface->write_reg(0x27d,0x04); //"" TX //FIXME 0x04

        //ATRs configured in b200_impl()        

        //set_clock_rate(40e6); //init ref clk (done above)
        //tune("TX", 850e6);
        //tune("RX", 800e6);

        //ian magic
        _b200_iface->write_reg(0x014, 0b00001111);
        _b200_iface->write_reg(0x014, 0b00101011);

        //output_test_tone();
    }

    std::vector<std::string> get_gain_names(const std::string &which)
    {
        //TODO
        return std::vector<std::string>();
    }

    double set_gain(const std::string &which, const std::string &name, const double value)
    {
        //use Full Gain Table mode, default table
        if(which[0] == 'R') {
            //reg 0x109 bits 6:0 for RX gain
            //gain table index is dB+3 for 200-1300MHz,
            //db+5 for 1300-4000
            //db+14 for 4000-6000
            int gain_offset;
            if(_rx_freq < 1300) gain_offset = 3;
            else if(_rx_freq < 4000) gain_offset = 5;
            else gain_offset = 14;

            int gainreg = value + gain_offset;

            if(gainreg > 76) gainreg = 76;
            if(gainreg < 0) gainreg = 0;
            if(which[3] == 'A') {            
                _b200_iface->write_reg(0x109, gainreg);
            } else {
                _b200_iface->write_reg(0x10c, gainreg);
            }

            return gainreg - gain_offset;
        } else { //TX gain
            //check 0x077 bit 6, make sure it's set
            _b200_iface->write_reg(0x077, 0x40);
            //check 0x07c bit 7, make sure it's set
            _b200_iface->write_reg(0x07c, 0x40);
            //TX_A gain is 0x074[0], 0x073[7:0]
            //TX_B gain is 0x076[0], 0x075[7:0]
            //gain step is -0.25dB, make sure you set for 89.75 - (gain)
            //in order to get the correct direction
            double atten = get_gain_range("TX_A", "").stop() - value;
            int attenreg = atten * 4;
            if(which[3] == 'A') {
                _b200_iface->write_reg(0x074, (attenreg >> 8) & 0x01);
                _b200_iface->write_reg(0x073, attenreg & 0xFF);
            } else {
                _b200_iface->write_reg(0x076, (attenreg >> 8) & 0x01);
                _b200_iface->write_reg(0x075, attenreg & 0xFF);
            }
            return get_gain_range("TX_A", "").stop() - (double(attenreg)/ 4);
        }
    }

    uhd::meta_range_t get_gain_range(const std::string &which, const std::string &name)
    {
        if(which[0] == 'R') {
            return uhd::meta_range_t(0.0, 73.0);
        } else {
            return uhd::meta_range_t(0.0, 89.75);
        }
    }

    double set_clock_rate(const double rate)
    {
        return (61.44e6 / 4); //FIXME
    }

    double set_adcclk(const double rate) {
        //this sets the ADC clock rate -- NOT the sample rate!
        //sample rate also depends on the HB and FIR filter decimation/interpolations
        //modulus is 2088960
        //fo = fref * (Nint + Nfrac/mod)
        const double fref = 40e6;
        const int modulus = 2088960;

        const double vcomax = 1430e6;
        const double vcomin = 715e6;
        double vcorate;
        int vcodiv;

        //iterate over VCO dividers until appropriate divider is found
        int i=0;
        for(i=0; i<=6; i++) {
            vcodiv = 2<<i;
            vcorate = rate * vcodiv;
            if(vcorate >= vcomin && vcorate <= vcomax) break;
        }
        if(i == 7) throw uhd::runtime_error("Can't find valid VCO rate!");
        //TODO this will pick the low rate for threshold values, should eval
        //whether vcomax or vcomin has better performance

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;
        std::cout << "Nint: " << nint << " Nfrac: " << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);
        
        _b200_iface->write_reg(0x044, nint); //Nint
        _b200_iface->write_reg(0x041, (nfrac >> 16) & 0xFF); //Nfrac[23:16]
        _b200_iface->write_reg(0x042, (nfrac >> 8) & 0xFF); //Nfrac[15:8]
        _b200_iface->write_reg(0x043, nfrac & 0xFF); //Nfrac[7:0]

        //use XTALN input, CLKOUT=XTALN (40MHz ref out to FPGA)
        _b200_iface->write_reg(0x00A, 0b00010000 | vcodiv); //set BBPLL divider

        //CP filter recommended coefficients, don't change unless you have a clue
        _b200_iface->write_reg(0x048, 0xe8);
        _b200_iface->write_reg(0x049, 0x5b);
        _b200_iface->write_reg(0x04a, 0x35);

        //scale CP current according to VCO rate
        const double icp_baseline = 150e-6;
        const double freq_baseline = 1280e6;
        double icp = icp_baseline * actual_vcorate / freq_baseline;
        std::cout << "Settled on CP current: " << icp * 1e6 << "uA" << std::endl;
        int icp_reg = icp/25e-6 + 1;

        //CP current
        _b200_iface->write_reg(0x046, icp_reg & 0x3F);

        //calibrate freq (0x04B[7], toggle 0x03F[2] 1 then 0)
        _b200_iface->write_reg(0x04B, 0xC0);
        _b200_iface->write_reg(0x03F, 0x05);
        _b200_iface->write_reg(0x03F, 0x01); //keep bbpll on

        _b200_iface->write_reg(0x04c, 0x86); //increase Kv and phase margin (?)
        _b200_iface->write_reg(0x04d, 0x01);
        _b200_iface->write_reg(0x04d, 0x05);

        //TODO FIXME: check for lock and toss if unlocked

        return actual_vcorate;
    }

    double tune(const std::string &which, const double value)
    {
        //setup charge pump based on horrible PLL lock doc
        //setup VCO/RFPLL based on rx/tx freq
        //VCO cal

        //this is the example 800MHz/850MHz stuff from the tuning document
        //set rx to 800, tx to 850
        if(which[0] == 'R') {
            //set up synth
            _b200_iface->write_reg(0x23a, 0x4a);//vco output level
            _b200_iface->write_reg(0x239, 0xc3);//init ALC value and VCO varactor
            _b200_iface->write_reg(0x242, 0x1f);//vco bias and bias ref
            _b200_iface->write_reg(0x238, 0x78);//vco cal offset
            _b200_iface->write_reg(0x245, 0x00);//vco cal ref tcf
            _b200_iface->write_reg(0x251, 0x0c);//varactor ref
            _b200_iface->write_reg(0x250, 0x70);//vco varactor ref tcf
            _b200_iface->write_reg(0x240, 0x09);//rx synth loop filter r3
            _b200_iface->write_reg(0x23f, 0xdf);//r1 and c3
            _b200_iface->write_reg(0x23e, 0xd4);//c2 and c1
            _b200_iface->write_reg(0x23b, 0x92);//Icp

            //tune that shit
            _b200_iface->write_reg(0x233, 0x00);
            _b200_iface->write_reg(0x234, 0x00);
            _b200_iface->write_reg(0x235, 0x00);
            _b200_iface->write_reg(0x232, 0x00);
            _b200_iface->write_reg(0x231, 0x50);
            _b200_iface->write_reg(0x005, 0x22);
            
            if((_b200_iface->read_reg(0x247) & 0x02) == 0) {
                std::cout << "RX PLL NOT LOCKED" << std::endl;
            }
            _rx_freq = 800.0e6;
            return _rx_freq;
        } else {
            _b200_iface->write_reg(0x27a, 0x4a);//vco output level
            _b200_iface->write_reg(0x279, 0xc1);//init ALC value and VCO varactor
            _b200_iface->write_reg(0x282, 0x17);//vco bias and bias ref
            _b200_iface->write_reg(0x278, 0x70);//vco cal offset //fixme 0x71
            _b200_iface->write_reg(0x285, 0x00);//vco cal ref tcf
            _b200_iface->write_reg(0x291, 0x0e);//varactor ref
            _b200_iface->write_reg(0x290, 0x70);//vco varactor ref tcf
            _b200_iface->write_reg(0x280, 0x09);//rx synth loop filter r3 //fixme 0x0F
            _b200_iface->write_reg(0x27f, 0xdf);//r1 and c3 //fixme e7
            _b200_iface->write_reg(0x27e, 0xd4);//c2 and c1 //fixme f3
            _b200_iface->write_reg(0x27b, 0x98);//Icp //fixme 0x88

            //tuning yo
            _b200_iface->write_reg(0x273, 0x00);
            _b200_iface->write_reg(0x274, 0x00);
            _b200_iface->write_reg(0x275, 0x00);
            _b200_iface->write_reg(0x272, 0x00);
            _b200_iface->write_reg(0x271, 0x55);
            _b200_iface->write_reg(0x005, 0x22);
            
            if((_b200_iface->read_reg(0x287) & 0x02) == 0) {
                std::cout << "TX PLL NOT LOCKED" << std::endl;
            }
            _tx_freq = 850.0e6;
            return _tx_freq;
        }
    }

    virtual double set_sample_rate(const double rate)
    {
        //set up BBPLL
        return 4e6;
    }

    virtual double set_filter_bw(const std::string &which, const double bw)
    {
        //set up BB filter
        //set BBLPF1 to 1.6x bw
        //set BBLPF2 to 2x bw
        if(which == "TX") {
            _b200_iface->write_reg(0x0d6, 0x0c);
            _b200_iface->write_reg(0x0d7, 0x1e);
            _b200_iface->write_reg(0x0d8, 0x06);
            _b200_iface->write_reg(0x0d9, 0x00);
            _b200_iface->write_reg(0x0ca, 0x22);
            _b200_iface->write_reg(0x016, 0x40);
            //while(_b200_iface->read_reg(0x016) & 0x40);
            _b200_iface->write_reg(0x0ca, 0x26);

            //setup TX secondary filter
            _b200_iface->write_reg(0x0d2, 0x29);
            _b200_iface->write_reg(0x0d1, 0x0c);
            _b200_iface->write_reg(0x0d0, 0x56);
            return 6.0e6;
        } else {
            _b200_iface->write_reg(0x1fb, 0x06);
            _b200_iface->write_reg(0x1fc, 0x00);
            _b200_iface->write_reg(0x1f8, 0x0d);
            _b200_iface->write_reg(0x1f9, 0x1e);
            _b200_iface->write_reg(0x1d5, 0x3f);
            _b200_iface->write_reg(0x1c0, 0x03);
            _b200_iface->write_reg(0x1e2, 0x02);
            _b200_iface->write_reg(0x1e3, 0x02);
            _b200_iface->write_reg(0x016, 0x80); //start tune
            //while(_b200_iface->read_reg(0x016) & 0x80);
            _b200_iface->write_reg(0x1e2, 0x03);
            _b200_iface->write_reg(0x1e3, 0x03);

            //setup RX TIA
            _b200_iface->write_reg(0x1db, 0x60);
            _b200_iface->write_reg(0x1dd, 0x08);
            _b200_iface->write_reg(0x1df, 0x08);
            _b200_iface->write_reg(0x1dc, 0x40);
            _b200_iface->write_reg(0x1de, 0x40);
            return 6.0e6;
        }
    }


    void output_test_tone(void)
    {
        /* Output a 480 kHz tone at 800 MHz */
        _b200_iface->write_reg(0x3F4, 0x0B);
        _b200_iface->write_reg(0x3FC, 0xFF);
        _b200_iface->write_reg(0x3FD, 0xFF);
        _b200_iface->write_reg(0x3FE, 0x3F);
    }


private:
    b200_iface::sptr _b200_iface;
    double _rx_freq, _tx_freq;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_codec_ctrl::sptr b200_codec_ctrl::make(b200_iface::sptr iface)
{
    return sptr(new b200_codec_ctrl_impl(iface));
}
