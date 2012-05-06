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

        /********setup basic stuff (chip level setup 0-7)*/
        //enable RX1, TX1 @ 4Msps
        //TX1 en, THB3 interp x2, THB2 interp x2 fil. en, THB1 en, TX FIR interp 4 en
        _b200_iface->write_reg(0x002, 0b11011110); //FIXME 0b11011110 (both xmit, interp 2)
        //RX1 en, RHB3 decim x2, RHB2 decim x2 fil. en, RHB1 en, RX FIR decim 4 en
        _b200_iface->write_reg(0x003, 0b11011110); //FIXME 0b11011110 (both rx, interp 2)
        //select TX1A/TX2A, RX antennas in balanced mode on ch. A
        _b200_iface->write_reg(0x004, 0b00000011);

        /**set up clock interface*/
        //enable BBPLL, clocks, external clk
        _b200_iface->write_reg(0x009, 0b00010111);

        /**set up BBPLL*/
        //set BBPLL to 1024MHz, BBPLL div 16, fref=40MHz for 4Msps
        //modulus is 2088960
        //fo = fref * (Nint + Nfrac/mod)
        //1024/40 = Nint + Nfrac/mod
        //1024/40 = 25.6, so Nint=25, Nfrac/mod = 0.6
        //Nfrac = 1253376 = 0x00132000
        //Nint = 25 = 0x19
        //Cp current 150uA, R1 = 1694, C1 = 522p, R2 = 1563, C2 = 39.2p, C3 = 14.6p
        //R1 = 0x08, C1 = 0x1f, C2 = 0x16, R2 = 0x02, C3 = 0x0c
        _b200_iface->write_reg(0x00A, 0b00010100);
        _b200_iface->write_reg(0x044, 0x19); //Nint
        _b200_iface->write_reg(0x041, 0x13); //Nfrac[23:16]
        _b200_iface->write_reg(0x042, 0x20); //Nfrac[15:8]
        _b200_iface->write_reg(0x043, 0x00); //Nfrac[7:0]

        //CP filter recommended coefficients
        _b200_iface->write_reg(0x048, 0xe8);
        _b200_iface->write_reg(0x049, 0x5b);
        _b200_iface->write_reg(0x04a, 0x35);

        //CP current to 150uA
        _b200_iface->write_reg(0x046, 0x06);

        //calibrate freq (0x04B[7], toggle 0x03F[2] 1 then 0)
        _b200_iface->write_reg(0x04B, 0xC0);
        _b200_iface->write_reg(0x03F, 0x05);
        _b200_iface->write_reg(0x03F, 0x01); //keep bbpll on

        /********setup data ports (FDD dual port DDR CMOS)*/
        //FDD dual port DDR CMOS no swap
        _b200_iface->write_reg(0x010, 0b00001000); //FIXME 0b10101000 (swap TX IQ swap TX1/TX2)
        _b200_iface->write_reg(0x011, 0b00000000);
        _b200_iface->write_reg(0x012, 0b00000010); //force TX on one port, RX on the other, come back to this one
        _b200_iface->write_reg(0x013, 0b00000001); //enable ENSM
        _b200_iface->write_reg(0x014, 0b00100001); //use SPI for TXNRX ctrl, to alert, TX on
        _b200_iface->write_reg(0x015, 0b00000111); //dual synth mode, synth en ctrl en

        //ian magic
        _b200_iface->write_reg(0x014, 0b00001111);
        _b200_iface->write_reg(0x014, 0b00101011);

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
        tune("TX", 850e6);
        tune("RX", 800e6);
    }

    std::vector<std::string> get_gain_names(const std::string &which)
    {
        //TODO
        return std::vector<std::string>();
    }

    double set_gain(const std::string &which, const std::string &name, const double value)
    {
        //TODO
        return 0.0;
    }

    uhd::meta_range_t get_gain_range(const std::string &which, const std::string &name)
    {
        //TODO
        return uhd::meta_range_t(0.0, 0.0);
    }

    double set_clock_rate(const double rate)
    {
        //clock rate is just the output clock rate to the FPGA
        //for Catalina sample rate, see set_sample_rate
        //set ref to refclk in via XTAL_N
        //set ref to 40e6
        //BBPLL input scale keep between 35-70MHz
        //RXPLL input scale keep between 40-150MHz?
        //enable CLK_OUT=REF_CLK_IN (could use div. of ADC clk but that'll happen later)
        
        return 8e6;
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
            return 800.0e6;
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
            return 850.0e6;
        }
    }

    virtual double set_sample_rate(const double rate)
    {
        //set up BBPLL
    }

    virtual double set_filter_bw(const std::string &which, const double bw)
    {
        //set up BB filter
        //set BBLPF1 to 1.6x bw
        //set BBLPF2 to 2x bw
        if(which == "TX") {

        } else {

        }
    }

private:
    b200_iface::sptr _b200_iface;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_codec_ctrl::sptr b200_codec_ctrl::make(b200_iface::sptr iface)
{
    return sptr(new b200_codec_ctrl_impl(iface));
}
