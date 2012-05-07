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

#include "b200_regs.hpp"
#include "b200_codec_ctrl.hpp"
#include <uhd/exception.hpp>
#include <iostream>
#include <boost/thread.hpp>

using namespace uhd;
using namespace uhd::transport;

static const int RX_BANDSEL_C = (1 << 0);
static const int RX_BANDSEL_B = (1 << 1);
static const int RX_BANDSEL_A = (1 << 2);
static const int TX_BANDSEL_B = (1 << 3);
static const int TX_BANDSEL_A = (1 << 4);

/***********************************************************************
 * The implementation class
 **********************************************************************/
class b200_codec_ctrl_impl : public b200_codec_ctrl{
public:

    b200_codec_ctrl_impl(b200_iface::sptr iface, wb_iface::sptr ctrl)
    {
        _b200_iface = iface;
        _ctrl = ctrl;

        //init bandsels (TODO based on settings)
        const int bs_flags = TX_BANDSEL_A | RX_BANDSEL_B;
        _ctrl->poke32(TOREG(SR_MISC+1), bs_flags);

        //reset
        _b200_iface->write_reg(0x000,0x01);
        //clear reset
        _b200_iface->write_reg(0x000,0x00);

        boost::this_thread::sleep(boost::posix_time::milliseconds(20));

        //there is not a WAT big enough for this
        _b200_iface->write_reg(0x3df, 0x01);

        //bandgap setup
        _b200_iface->write_reg(0x2a6, 0x0e);
        _b200_iface->write_reg(0x2a8, 0x0e);

        //rfpll refclk to REFCLKx2
        _b200_iface->write_reg(0x2ab, 0x07);
        _b200_iface->write_reg(0x2ac, 0xff);
        _b200_iface->write_reg(0x009, 0b00010111);

        boost::this_thread::sleep(boost::posix_time::milliseconds(20));

        /**set up BBPLL*/
        //245.76Msps ADC clock means:
        //div 2 in RHB3 = 122.88Msps
        //div 2 in RHB2 = 61.44Msps
        //div 2 in RHB1 = 30.72sps
        //div 2 in RXFIR = 15.36Msps output
        //eventually this should be done alongside FIR/HB setup
        set_adcclk(245.76e6); //for 15.36Msps
        //TX1/2 en, THB3 interp x2, THB2 interp x2 fil. en, THB1 en, TX FIR interp 2 en
        _b200_iface->write_reg(0x002, 0b01011110);
        //RX1/2 en, RHB3 decim x2, RHB2 decim x2 fil. en, RHB1 en, RX FIR decim 2 en
        _b200_iface->write_reg(0x003, 0b01011110);
        //select TX1A/TX2A, RX antennas in balanced mode on ch. C
        _b200_iface->write_reg(0x004, 0b01110000);

        //setup TX FIR
        setup_tx_fir();
        //setup RX FIR
        setup_rx_fir();

        /********setup data ports (FDD dual port DDR CMOS)*/
        //FDD dual port DDR CMOS no swap
        _b200_iface->write_reg(0x010, 0b11001000); //FIXME 0b10101000 (swap TX IQ swap TX1/TX2)
        _b200_iface->write_reg(0x011, 0b00000000);
        _b200_iface->write_reg(0x012, 0b00000010); //force TX on one port, RX on the other

        //data delay for TX and RX data clocks
        _b200_iface->write_reg(0x006, 0x0F);
        _b200_iface->write_reg(0x007, 0x00);

        //set delay register
        _b200_iface->write_reg(0x03a, 0x27);

        /**initial VCO setup*/
        _b200_iface->write_reg(0x261,0x00); //RX LO power
        _b200_iface->write_reg(0x2a1,0x00); //TX LO power
        _b200_iface->write_reg(0x248,0x0b); //en RX VCO LDO
        _b200_iface->write_reg(0x288,0x0b); //en TX VCO LDO
        _b200_iface->write_reg(0x246,0x02); //pd RX cal Tcf
        _b200_iface->write_reg(0x286,0x02); //pd TX cal Tcf
        _b200_iface->write_reg(0x249,0x8e); //rx vco cal length
        _b200_iface->write_reg(0x289,0x8e); //rx vco cal length
        _b200_iface->write_reg(0x23b,0x80); //set RX MSB? //FIXME 0x89 magic charge pump current, undocumented
        _b200_iface->write_reg(0x27b,0x80); //"" TX //FIXME 0x88 see above
        _b200_iface->write_reg(0x243,0x0d); //set rx prescaler bias
        _b200_iface->write_reg(0x283,0x0d); //"" TX
        _b200_iface->write_reg(0x23d,0x00); //clear RX 1/2 VCO cal clk //FIXME 0x04 enable CP cal, "only change if apps eng. says so"
        _b200_iface->write_reg(0x27d,0x00); //"" TX //FIXME 0x04
        
        _b200_iface->write_reg(0x245,0x00); //set RX VCO cal ref Tcf
        _b200_iface->write_reg(0x250,0x70); //set RX VCO varactor ref Tcf
        _b200_iface->write_reg(0x285,0x00); //"" TX
        _b200_iface->write_reg(0x290,0x70); //"" TX
        _b200_iface->write_reg(0x239,0xc1); //init RX ALC
        _b200_iface->write_reg(0x279,0xc1); //"" TX

        _b200_iface->write_reg(0x015, 0b00000111); //dual synth mode, synth en ctrl en
        _b200_iface->write_reg(0x014, 0b00100001); //use SPI for TXNRX ctrl, to alert, TX on
        _b200_iface->write_reg(0x013, 0b00000001); //enable ENSM

        //RX CP CAL
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        _b200_iface->write_reg(0x23d, 0x04);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        if(!(_b200_iface->read_reg(0x244) & 0x80)) {
            std::cout << "RX charge pump cal failure" << std::endl;
        }
        //TX CP CAL
        _b200_iface->write_reg(0x27d, 0x04);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        if(!(_b200_iface->read_reg(0x284) & 0x80)) {
            std::cout << "TX charge pump cal failure" << std::endl;
        }

        tune("RX", 800e6);
        tune("TX", 850e6);

        //gm subtable here

        //gain table here
        
        //set baseband filter BW
        set_filter_bw("RX_A", 6.0e6);
        set_filter_bw("TX_A", 6.0e6);

        //setup RX TIA
        _b200_iface->write_reg(0x1db, 0x60);
        _b200_iface->write_reg(0x1dd, 0x08);
        _b200_iface->write_reg(0x1df, 0x08);
        _b200_iface->write_reg(0x1dc, 0x40);
        _b200_iface->write_reg(0x1de, 0x40);

        //setup TX secondary filter
        _b200_iface->write_reg(0x0d2, 0x29);
        _b200_iface->write_reg(0x0d1, 0x0c);
        _b200_iface->write_reg(0x0d0, 0x56);

        //adc setup
        setup_adc();

        quad_cal();

        //ian magic
        _b200_iface->write_reg(0x014, 0b00001111);
        _b200_iface->write_reg(0x014, 0b00100001); //WAS 0b00101011

        //ATRs configured in b200_impl()

        //set_clock_rate(40e6); //init ref clk (done above)
        
        std::cout << std::endl;
        for(int i=0; i < 64; i++) {
            std::cout << std::hex << std::uppercase << int(i) << ",";
            int j=0;
            for(; j < 15; j++) {
                std::cout << std::hex << std::uppercase << int(_b200_iface->read_reg(i*16+j)) << ",";
            }
            std::cout << std::hex << std::uppercase << int(_b200_iface->read_reg(i*16+j));
            std::cout << std::endl;
        }
        //output_test_tone();
    }

    std::vector<std::string> get_gain_names(const std::string &which)
    {
        return std::vector<std::string>(1, "PGA");
    }

    double set_gain(const std::string &which, const std::string &name, const double value)
    {
        //use Full Gain Table mode, default table
        if(which[0] == 'R') {
            //reg 0x109 bits 6:0 for RX gain
            //gain table index is dB+3 for 200-1300MHz,
            //db+5 for 1300-4000
            //db+14 for 4000-6000

            //set some AGC crap
            _b200_iface->write_reg(0x0fc, 0x23);
            
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
            return uhd::meta_range_t(0.0, 73.0, 1.0);
        } else {
            return uhd::meta_range_t(0.0, 89.75, 0.25);
        }
    }

    uhd::meta_range_t get_rf_freq_range(const std::string &which)
    {
        return uhd::meta_range_t(0.0, 1e12); //FIXME TODO
    }

    uhd::meta_range_t get_bw_filter_range(const std::string &which)
    {
        return uhd::meta_range_t(0.0, 60e6); //FIXME TODO
    }

    double set_bw_filter(const std::string &which, const double bw)
    {
        return 0.0; //TODO
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
            vcodiv = 1<<i;
            vcorate = rate * vcodiv;
            if(vcorate >= vcomin && vcorate <= vcomax) break;
        }
        if(i == 7) throw uhd::runtime_error("BBVCO can't find valid VCO rate!");
        //TODO this will pick the low rate for threshold values, should eval
        //whether vcomax or vcomin has better performance

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;
        std::cout << "BB Nint: " << nint << " Nfrac: " << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);
        
        _b200_iface->write_reg(0x044, nint); //Nint
        _b200_iface->write_reg(0x041, (nfrac >> 16) & 0xFF); //Nfrac[23:16]
        _b200_iface->write_reg(0x042, (nfrac >> 8) & 0xFF); //Nfrac[15:8]
        _b200_iface->write_reg(0x043, nfrac & 0xFF); //Nfrac[7:0]

        //use XTALN input, CLKOUT=XTALN (40MHz ref out to FPGA)
        _b200_iface->write_reg(0x00A, 0b00010000 | i); //set BBPLL divider
        std::cout << "BBPLL divider reg: " << std::hex << int(0b00010000 | i) << std::endl;

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
        _b200_iface->write_reg(0x04e, 0x10); //slow down cal for better accuracy
        _b200_iface->write_reg(0x04B, 0xE0);
        _b200_iface->write_reg(0x03F, 0x05);
        _b200_iface->write_reg(0x03F, 0x01); //keep bbpll on

        _b200_iface->write_reg(0x04c, 0x86); //increase Kv and phase margin (?)
        _b200_iface->write_reg(0x04d, 0x01);
        _b200_iface->write_reg(0x04d, 0x05);

        //check for BBPLL lock
        boost::this_thread::sleep(boost::posix_time::milliseconds(2));
        if(!_b200_iface->read_reg(0x05e) & 0x80) {
            uhd::runtime_error("BBPLL not locked");
        }

        return actual_vcorate;
    }

    double tune(const std::string &which, const double value)
    {
        //setup charge pump
        //setup VCO/RFPLL based on rx/tx freq
        //VCO cal

        //RFPLL runs from 6GHz-12GHz
        const double fref = 80e6; //fixed for now
        const int modulus = 8388593;

        const double vcomax = 12e9;
        const double vcomin = 6e9;
        double vcorate;
        int vcodiv;

        //iterate over VCO dividers until appropriate divider is found
        int i;
        for(i=0; i<=6; i++) {
            vcodiv = 2<<i;
            vcorate = value * vcodiv;
            if(vcorate >= vcomin && vcorate <= vcomax) break;
        }
        if(i == 7) throw uhd::runtime_error("RFVCO can't find valid VCO rate!");
        //TODO this will pick the low rate for threshold values, should eval
        //whether vcomax or vcomin has better performance

        std::cout << "RF VCO rate: " << vcorate << std::endl;

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;
        std::cout << "RF Nint: " << nint << " Nfrac: " << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);
        double actual_lo = actual_vcorate / vcodiv;

        if(which[0] == 'R') {
            //set up synth
            _b200_iface->write_reg(0x23a, 0x4a);//vco output level
            _b200_iface->write_reg(0x239, 0xc1);//init ALC value and VCO varactor
            _b200_iface->write_reg(0x242, 0x17);//vco bias and bias ref
            _b200_iface->write_reg(0x238, 0x70);//vco cal offset
            _b200_iface->write_reg(0x245, 0x00);//vco cal ref tcf
            _b200_iface->write_reg(0x251, 0x0e);//varactor ref
            _b200_iface->write_reg(0x250, 0x70);//vco varactor ref tcf
            _b200_iface->write_reg(0x240, 0x0f);//rx synth loop filter r3
            _b200_iface->write_reg(0x23f, 0xe7);//r1 and c3
            _b200_iface->write_reg(0x23e, 0xf3);//c2 and c1
            _b200_iface->write_reg(0x23b, 0x89);//Icp

            //tune that shit
            _b200_iface->write_reg(0x233, nfrac & 0xFF);
            _b200_iface->write_reg(0x234, (nfrac >> 8) & 0xFF);
            _b200_iface->write_reg(0x235, (nfrac >> 16) & 0xFF);
            _b200_iface->write_reg(0x232, (nint >> 8) & 0xFF);
            _b200_iface->write_reg(0x231, nint & 0xFF);
            _b200_iface->write_reg(0x005, 0x20 | (i & 0x07));

            _b200_iface->write_reg(0x236, 0x6b); //undoc vco settings
            _b200_iface->write_reg(0x237, 0x65);
            _b200_iface->write_reg(0x238, 0x71); //force vco tune <8

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x247) & 0x02) == 0) {
                std::cout << "RX PLL NOT LOCKED" << std::endl;
            }
            _rx_freq = actual_lo;
            return _rx_freq;
        } else {
            _b200_iface->write_reg(0x27a, 0x4a);//vco output level
            _b200_iface->write_reg(0x279, 0xc1);//init ALC value and VCO varactor
            _b200_iface->write_reg(0x282, 0x17);//vco bias and bias ref
            _b200_iface->write_reg(0x278, 0x70);//vco cal offset //fixme 0x71
            _b200_iface->write_reg(0x285, 0x00);//vco cal ref tcf
            _b200_iface->write_reg(0x291, 0x0e);//varactor ref
            _b200_iface->write_reg(0x290, 0x70);//vco varactor ref tcf
            
            _b200_iface->write_reg(0x280, 0x0f);//rx synth loop filter r3 //fixme 0x0F
            _b200_iface->write_reg(0x27f, 0xe7);//r1 and c3 //fixme e7
            _b200_iface->write_reg(0x27e, 0xf3);//c2 and c1 //fixme f3
            _b200_iface->write_reg(0x27b, 0x88);//Icp //fixme 0x88

            //tuning yo
            _b200_iface->write_reg(0x273, nfrac & 0xFF);
            _b200_iface->write_reg(0x274, (nfrac >> 8) & 0xFF);
            _b200_iface->write_reg(0x275, (nfrac >> 16) & 0xFF);
            _b200_iface->write_reg(0x272, (nint >> 8) & 0xFF);
            _b200_iface->write_reg(0x271, nint & 0xFF);
            _b200_iface->write_reg(0x005, 0x20 | (i & 0x07));

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x287) & 0x02) == 0) {
                std::cout << "TX PLL NOT LOCKED" << std::endl;
            }
            _tx_freq = actual_lo;
            return _tx_freq;
        }
    }

    virtual double set_sample_rate(const double rate)
    {
        uhd::runtime_error("don't do that");
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
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            if(_b200_iface->read_reg(0x016) & 0x40) {
                std::cout << "RX baseband filter cal failure" << std::endl;
            }
            _b200_iface->write_reg(0x0ca, 0x26);

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
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            if(_b200_iface->read_reg(0x016) & 0x80) {
                std::cout << "RX baseband filter cal failure" << std::endl;
            }
            _b200_iface->write_reg(0x1e2, 0x03);
            _b200_iface->write_reg(0x1e3, 0x03);

            return 6.0e6;
        }
    }

    void setup_rx_fir()
    {
        //256 tap symmetric FIR filter for 12MHz RX bandwidth
        uint16_t filter_coeffs[] = {
            0x0001, 0xfff1, 0xffcf, 0xffc0, 0xffe8, 0x0020, 0x001a, 0x00e3,
            0xffe1, 0x001f, 0x0028, 0xffdf, 0xffcc, 0x0024, 0x0043, 0xffdb,
            0xffac, 0x0026, 0x0068, 0xffdb, 0xff80, 0x0022, 0x009a, 0xffe2,
            0xff47, 0x0017, 0x00db, 0xfff3, 0xfeff, 0xffff, 0x012b, 0x0013,
            0xfea5, 0xffd7, 0x0190, 0x0046, 0xfe35, 0xff97, 0x020e, 0x0095,
            0xfda7, 0xff36, 0x02ae, 0x010d, 0xfcf0, 0xfea1, 0x0383, 0x01c6,
            0xfbf3, 0xfdb6, 0x04b7, 0x02f8, 0xfa6d, 0xfc1a, 0x06be, 0x0541,
            0xf787, 0xf898, 0x0b60, 0x0b6d, 0xee88, 0xea40, 0x2786, 0x7209
            };

        _b200_iface->write_reg(0x0f5, 0xfa); //enable filter clk
        _b200_iface->write_reg(0x0f6, 0x02); //filter gain
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        int addr = 0;
        for(; addr < 128; addr++) {
            _b200_iface->write_reg(0x0f0, addr);
            _b200_iface->write_reg(0x0f1, (filter_coeffs[addr] >> 8) & 0xff);
            _b200_iface->write_reg(0x0f2, filter_coeffs[addr] & 0xff);
            _b200_iface->write_reg(0x0f5, 0xfe);
            _b200_iface->write_reg(0x0f4, 0x00);
            _b200_iface->write_reg(0x0f4, 0x00);
        }
        //it's symmetric, so we write it out again backwards
        for(; addr < 256; addr++) {
            _b200_iface->write_reg(0x0f0, addr);
            _b200_iface->write_reg(0x0f1, (filter_coeffs[128-addr] >> 8) & 0xff);
            _b200_iface->write_reg(0x0f2, filter_coeffs[128-addr] & 0xff);
            _b200_iface->write_reg(0x0f5, 0xfe);
            _b200_iface->write_reg(0x0f4, 0x00);
            _b200_iface->write_reg(0x0f4, 0x00);
        }
        _b200_iface->write_reg(0x0f5, 0xf8); //disable filter clk
    }

    void setup_tx_fir()
    {

    }

    void setup_adc()
    {
        _b200_iface->write_reg(0x1db, 0x60);
        _b200_iface->write_reg(0x1dd, 0x08);
        _b200_iface->write_reg(0x1df, 0x08);
        _b200_iface->write_reg(0x1dc, 0x40);
        _b200_iface->write_reg(0x1de, 0x40);
        uint8_t adc_regs[] = {
            0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0x00, 0x7c,
            0x52, 0x3c, 0x4c, 0x34, 0x4f, 0x32, 0x00, 0x7f,
            0x7e, 0x7f, 0x4a, 0x49, 0x4a, 0x4d, 0x4c, 0x4d,
            0x2e, 0x98, 0x1b, 0x13, 0x98, 0x1b, 0x13, 0x98,
            0x1b, 0x27, 0x27, 0x40, 0x40, 0x2c, 0x00, 0x00};
        for(int i=0; i<40; i++) {
            _b200_iface->write_reg(0x200+i, adc_regs[i]);
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

    void quad_cal(void)
    {
        //tx quad cal
        _b200_iface->write_reg(0x0a0, 0x3a);
        _b200_iface->write_reg(0x0a3, 0x40);
        _b200_iface->write_reg(0x0a1, 0x79);
        _b200_iface->write_reg(0x0a9, 0xff);
        _b200_iface->write_reg(0x0a2, 0x4f);
        _b200_iface->write_reg(0x0a5, 0x03);
        _b200_iface->write_reg(0x0a6, 0x03);
        _b200_iface->write_reg(0x0aa, 0x33);
        _b200_iface->write_reg(0x0a4, 0x20);

        //rx quad cal
        _b200_iface->write_reg(0x193, 0x3f);
        _b200_iface->write_reg(0x190, 0x0f);
        _b200_iface->write_reg(0x194, 0x01);
        _b200_iface->write_reg(0x016, 0x01);
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        _b200_iface->write_reg(0x185, 0x20);
        _b200_iface->write_reg(0x186, 0x10);
        _b200_iface->write_reg(0x187, 0x14);
        _b200_iface->write_reg(0x18b, 0x83);
        _b200_iface->write_reg(0x188, 0x64);
        _b200_iface->write_reg(0x189, 0x30);
        _b200_iface->write_reg(0x016, 0x02);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        _b200_iface->write_reg(0x016, 0x10);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        _b200_iface->write_reg(0x168, 0x03);
        _b200_iface->write_reg(0x16e, 0x25);
        _b200_iface->write_reg(0x16a, 0x75);
        _b200_iface->write_reg(0x16b, 0x15);
        _b200_iface->write_reg(0x169, 0xcf);
        _b200_iface->write_reg(0x18b, 0xad);
    }

private:
    b200_iface::sptr _b200_iface;
    wb_iface::sptr _ctrl;
    double _rx_freq, _tx_freq;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_codec_ctrl::sptr b200_codec_ctrl::make(b200_iface::sptr iface, wb_iface::sptr ctrl)
{
    return sptr(new b200_codec_ctrl_impl(iface, ctrl));
}
