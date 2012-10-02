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
#include "ad9361_synth_lut.hpp"
#include "ad9361_gain_tables.hpp"
#include <uhd/exception.hpp>
#include <uhd/utils/msg.hpp>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <vector>

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

    /***********************************************************************
     * Information retrieval functions
     **********************************************************************/

    uhd::meta_range_t get_gain_range(const std::string &which, \
            const std::string &name) {
        if(which[0] == 'R') {
            return uhd::meta_range_t(0.0, 73.0, 1.0);
        } else {
            return uhd::meta_range_t(0.0, 89.75, 0.25);
        }
    }

    uhd::meta_range_t get_rf_freq_range(const std::string &which) {
        return uhd::meta_range_t(100.0e6, 6.0e9);
    }

    uhd::meta_range_t get_bw_filter_range(const std::string &which) {
        return uhd::meta_range_t(0.0, 60e6); //TODO
    }

    double set_bw_filter(const std::string &which, const double bw) {
        return 0.0; //TODO
    }

    std::vector<std::string> get_gain_names(const std::string &which) {
        return std::vector<std::string>(1, "PGA");
    }


    /***********************************************************************
     * Placeholders, unused, or test functions
     **********************************************************************/

    double set_sample_rate(const double rate) {
        uhd::runtime_error("don't do that");
        return 0.0;
    }

    void output_test_tone(void) {
        /* Output a 480 kHz tone at 800 MHz */
        _b200_iface->write_reg(0x3F4, 0x0B);
        _b200_iface->write_reg(0x3FC, 0xFF);
        _b200_iface->write_reg(0x3FD, 0xFF);
        _b200_iface->write_reg(0x3FE, 0x3F);
    }



    /***********************************************************************
     * Filter functions
     **********************************************************************/

    void setup_fir(const std::string &which, int num_taps, uint16_t *coeffs) {
        uint16_t base;
        if(which == "RX") {
            base = 0x0f0;
            _b200_iface->write_reg(base+6, 0x02); //filter gain
        } else {
            base = 0x060;
        }

        /* Write the filter configuration. */
        uint8_t reg_numtaps = (((num_taps / 16) - 1) & 0x07) << 5;

        _b200_iface->write_reg(base+5, reg_numtaps | 0x1a); //enable filter clk
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        int num_unique_coeffs = (num_taps / 2);

        for(int addr=0; addr < num_unique_coeffs; addr++) {
            _b200_iface->write_reg(base+0, addr);
            _b200_iface->write_reg(base+1, (coeffs[addr]) & 0xff);
            _b200_iface->write_reg(base+2, (coeffs[addr] >> 8) & 0xff);
            _b200_iface->write_reg(base+5, 0xfe);
            _b200_iface->write_reg(base+4, 0x00);
            _b200_iface->write_reg(base+4, 0x00);
        }
        //it's symmetric, so we write it out again backwards
        for(int addr=0; addr < num_unique_coeffs; addr++) {
            _b200_iface->write_reg(base+0, addr+num_unique_coeffs);
            _b200_iface->write_reg(base+1, (coeffs[num_unique_coeffs-1-addr]) & 0xff);
            _b200_iface->write_reg(base+2, (coeffs[num_unique_coeffs-1-addr] >> 8) & 0xff);
            _b200_iface->write_reg(base+5, 0xfe);
            _b200_iface->write_reg(base+4, 0x00);
            _b200_iface->write_reg(base+4, 0x00);
        }
        _b200_iface->write_reg(base+5, 0xf8); //disable filter clk
    }

    void setup_rx_fir(int total_num_taps) {
        uint16_t master_coeffs[] = {
            0xffe2,0x0042,0x0024,0x0095,0x0056,0x004d,0xffcf,0xffb7,
            0xffb1,0x0019,0x0059,0x006a,0x0004,0xff9d,0xff72,0xffd4,
            0x0063,0x00b7,0x0062,0xffac,0xff21,0xff59,0x0032,0x0101,
            0x00f8,0x0008,0xfeea,0xfeac,0xffa3,0x0117,0x01b5,0x00d0,
            0xff05,0xfdea,0xfe9e,0x00ba,0x026f,0x0215,0xffb5,0xfd4a,
            0xfd18,0xffa0,0x02de,0x03dc,0x0155,0xfd2a,0xfb0d,0xfd54,
            0x0287,0x062f,0x048a,0xfe37,0xf862,0xf8c1,0x004d,0x0963,
            0x0b88,0x02a4,0xf3e7,0xebdd,0xf5f8,0x1366,0x3830,0x518b
        };

        int num_taps = total_num_taps / 2;
        std::vector<uint16_t> coeffs(num_taps);
        for(int i = 0; i < num_taps; i++) {
            coeffs[num_taps - 1 - i] = master_coeffs[63 - i];
        }

        setup_fir("RX", total_num_taps, &coeffs[0]);
    }

    void setup_tx_fir(int total_num_taps) {
        //LTE 6MHz
        uint16_t master_coeffs[] = {
            0xfffb,0x0000,0x0004,0x0017,0x0024,0x0028,0x0013,0xfff3,
            0xffdc,0xffe5,0x000b,0x0030,0x002e,0xfffe,0xffc4,0xffb8,
            0xfff0,0x0045,0x0068,0x002b,0xffb6,0xff72,0xffad,0x0047,
            0x00b8,0x0088,0xffc8,0xff1c,0xff33,0x001a,0x0110,0x0124,
            0x0019,0xfec8,0xfe74,0xff9a,0x0156,0x0208,0x00d3,0xfe9b,
            0xfd68,0xfe96,0x015d,0x033f,0x0236,0xfecd,0xfc00,0xfcb5,
            0x00d7,0x04e5,0x04cc,0xffd5,0xf9fe,0xf8fb,0xfef2,0x078c,
            0x0aae,0x036d,0xf5c0,0xed89,0xf685,0x12af,0x36a4,0x4faa
        };

        int num_taps = total_num_taps / 2;
        std::vector<uint16_t> coeffs(num_taps);
        for(int i = 0; i < num_taps; i++) {
            coeffs[num_taps - 1 - i] = master_coeffs[63 - i];
        }

        setup_fir("TX", total_num_taps, &coeffs[0]);
    }


    /***********************************************************************
     * Calibration functions
     ***********************************************************************/

    void calibrate_lock_bbpll() {
        _b200_iface->write_reg(0x03F, 0x05);
        _b200_iface->write_reg(0x03F, 0x01); //keep bbpll on

        _b200_iface->write_reg(0x04c, 0x86); //increase Kv and phase margin (?)
        _b200_iface->write_reg(0x04d, 0x01);
        _b200_iface->write_reg(0x04d, 0x05);

        /* Wait for BBPLL lock. */
        int count = 0;
        while(!(_b200_iface->read_reg(0x05e) & 0x80)) {
            if(count > 1000) {
                uhd::runtime_error("BBPLL not locked");
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
        }
    }

    void calibrate_synth_charge_pumps() {
        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            std::cout << "ERROR! Catalina not in ALERT during cal!" << std::endl;
        }

        /* Should only be done the first time the device enters ALERT. */
        //RX CP CAL
        int count = 0;
        _b200_iface->write_reg(0x23d, 0x04);
        while(!(_b200_iface->read_reg(0x244) & 0x80)) {
            if(count > 5) {
                std::cout << "RX charge pump cal failure!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        _b200_iface->write_reg(0x23d, 0x00);

        //TX CP CAL
        count = 0;
        _b200_iface->write_reg(0x27d, 0x04);
        while(!(_b200_iface->read_reg(0x284) & 0x80)) {
            if(count > 5) {
                std::cout << "TX charge pump cal failure" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        _b200_iface->write_reg(0x27d, 0x00);
    }

    double calibrate_baseband_rx_analog_filter() {
        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 28e6 and 0.2e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 28e6) {
            bbbw = 28e6;
        } else if (bbbw < 0.20e6) {
            bbbw = 0.20e6;
        }

        double rxtune_clk = ((1.4 * bbbw * 2 *
                boost::math::constants::pi<double>()) / std::log(2.0));

        _rx_bbf_tunediv = std::min(511, int(std::ceil(_bbpll_freq / rxtune_clk)));

        reg_bbftune_config = (reg_bbftune_config & 0xFE) \
                             | ((_rx_bbf_tunediv >> 8) & 0x0001);

        double bbbw_mhz = bbbw / 1e6;

        double temp = ((bbbw_mhz - std::floor(bbbw_mhz)) * 1000) / 7.8125;
        uint8_t bbbw_khz = std::min(127, int(std::floor(temp + 0.5)));

        _b200_iface->write_reg(0x1fb, uint8_t(bbbw_mhz));
        _b200_iface->write_reg(0x1fc, bbbw_khz);
        _b200_iface->write_reg(0x1f8, (_rx_bbf_tunediv & 0x00FF));
        _b200_iface->write_reg(0x1f9, reg_bbftune_config);

        /* FIXME remove this after debug complete
        std::cout << std::hex << "0x1fb: " << (int) uint8_t(bbbw_mhz) << std::endl;
        std::cout << std::hex << "0x1fc: " << (int) bbbw_khz << std::endl;
        std::cout << std::hex << "0x1f8: " << (int) (_rx_bbf_tunediv & 0x00FF) << std::endl;
        std::cout << std::hex << "0x1f9: " << (int) reg_bbftune_config << std::endl;
        */

        /* RX Mix Voltage settings - only change with apps engineer help. */
        _b200_iface->write_reg(0x1d5, 0x3f);
        _b200_iface->write_reg(0x1c0, 0x03);

        _b200_iface->write_reg(0x1e2, 0x02);
        _b200_iface->write_reg(0x1e3, 0x02);

        int count = 0;
        _b200_iface->write_reg(0x016, 0x80);
        while(_b200_iface->read_reg(0x016) & 0x80) {
            if(count > 5) {
                std::cout << "RX baseband filter cal FAILURE!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }

        _b200_iface->write_reg(0x1e2, 0x03);
        _b200_iface->write_reg(0x1e3, 0x03);

        return bbbw;
    }

    double calibrate_baseband_tx_analog_filter() {
        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 28e6 and 0.2e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 20e6) {
            bbbw = 20e6;
        } else if (bbbw < 0.625e6) {
            bbbw = 0.625e6;
        }

        double txtune_clk = ((1.6 * bbbw * 2 *
                boost::math::constants::pi<double>()) / std::log(2.0));

        uint16_t txbbfdiv = std::min(511, int(std::ceil(_bbpll_freq / txtune_clk)));

        reg_bbftune_mode = (reg_bbftune_mode & 0xFE) \
                             | ((txbbfdiv >> 8) & 0x0001);

        _b200_iface->write_reg(0x0d6, (txbbfdiv & 0x00FF));
        _b200_iface->write_reg(0x0d7, reg_bbftune_mode);

        _b200_iface->write_reg(0x0ca, 0x22);

        int count = 0;
        _b200_iface->write_reg(0x016, 0x40);
        while(_b200_iface->read_reg(0x016) & 0x40) {
            if(count > 5) {
                std::cout << "TX baseband filter cal FAILURE!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }

        _b200_iface->write_reg(0x0ca, 0x26);

        return bbbw;
    }

    void calibrate_secondary_tx_filter() {
        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 20e6 and 0.53e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 20e6) {
            bbbw = 20e6;
        } else if (bbbw < 0.53e6) {
            bbbw = 0.53e6;
        }

        double bbbw_mhz = bbbw / 1e6;

        /* Start with a resistor value of 100 Ohms. */
        int res = 100;

        /* Calculate target corner frequency. */
        double corner_freq = 5 * bbbw_mhz * 2 * boost::math::constants::pi<double>();

        /* Iterate through RC values to determine correct combination. */
        int cap = 0;
        for(int i = 0; i <= 3; i++) {
            cap = int(std::floor(0.5 + (( 1 / ((corner_freq * res) * 1e6)) * 1e12))) - 12;

            if(cap <= 63) {
                break;
            }

            res = res * 2;
        }
        if(cap > 63) {
            cap = 63;
        }

        uint8_t reg0d0, reg0d1, reg0d2;

        if((bbbw_mhz * 2) <= 9) {
            reg0d0 = 0x59;
        } else if(((bbbw_mhz * 2) > 9) && ((bbbw_mhz * 2) <= 24)) {
            reg0d0 = 0x56;
        } else if((bbbw_mhz * 2) > 24) {
            reg0d0 = 0x57;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        if(res == 100) {
            reg0d1 = 0x0c;
        } else if(res == 200) {
            reg0d1 = 0x04;
        } else if(res == 400) {
            reg0d1 = 0x03;
        } else if(res == 800) {
            reg0d1 = 0x01;
        } else {
            reg0d1 = 0x0c;
        }

        reg0d2 = cap;

        _b200_iface->write_reg(0x0d3, 0x60);
        _b200_iface->write_reg(0x0d2, reg0d2);
        _b200_iface->write_reg(0x0d1, reg0d1);
        _b200_iface->write_reg(0x0d0, reg0d0);
    }

    void calibrate_rx_TIAs() {
        uint8_t reg1eb = _b200_iface->read_reg(0x1eb) & 0x3F;
        uint8_t reg1ec = _b200_iface->read_reg(0x1ec) & 0x7F;
        uint8_t reg1e6 = _b200_iface->read_reg(0x1e6) & 0x07;
        uint8_t reg1db = 0x00;
        uint8_t reg1dc = 0x00;
        uint8_t reg1dd = 0x00;
        uint8_t reg1de = 0x00;
        uint8_t reg1df = 0x00;

        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 28e6 and 0.2e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 20e6) {
            bbbw = 20e6;
        } else if (bbbw < 0.20e6) {
            bbbw = 0.20e6;
        }
        double ceil_bbbw_mhz = std::ceil(bbbw / 1e6);

        int Cbbf = (reg1eb * 160) + (reg1ec * 10) + 140;
        int R2346 = 18300 * (reg1e6 & 0x07);
        double CTIA_fF = (Cbbf * R2346 * 0.56) / 3500;

        if(ceil_bbbw_mhz <= 3) {
            reg1db = 0xe0;
        } else if((ceil_bbbw_mhz > 3) && (ceil_bbbw_mhz <= 10)) {
            reg1db = 0x60;
        } else if(ceil_bbbw_mhz > 10) {
            reg1db = 0x20;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        if(CTIA_fF > 2920) {
            reg1dc = 0x40;
            reg1de = 0x40;

            uint8_t temp = std::min(127, int(std::floor(0.5 + ((CTIA_fF - 400.0) / 320.0))));
            reg1dd = temp;
            reg1df = temp;
        } else {
            uint8_t temp = std::floor(0.5 + ((CTIA_fF - 400.0) / 40.0)) + 0x40;
            reg1dc = temp;
            reg1de = temp;
            reg1dd = 0;
            reg1df = 0;
        }

        std::cout << std::hex << "reg1db: " << (int) reg1db << std::endl;
        std::cout << std::hex << "reg1dc: " << (int) reg1dc << std::endl;
        std::cout << std::hex << "reg1de: " << (int) reg1de << std::endl;
        std::cout << std::hex << "reg1dd: " << (int) reg1dd << std::endl;
        std::cout << std::hex << "reg1df: " << (int) reg1df << std::endl;

        _b200_iface->write_reg(0x1db, reg1db);
        _b200_iface->write_reg(0x1dd, reg1dd);
        _b200_iface->write_reg(0x1df, reg1df);
        _b200_iface->write_reg(0x1dc, reg1dc);
        _b200_iface->write_reg(0x1de, reg1de);

        std::cout << std::hex << "read reg1db: " << (int) _b200_iface->read_reg(0x1db) << std::endl;
        std::cout << std::hex << "read reg1dc: " << (int) _b200_iface->read_reg(0x1dc) << std::endl;
        std::cout << std::hex << "read reg1de: " << (int) _b200_iface->read_reg(0x1de) << std::endl;
        std::cout << std::hex << "read reg1dd: " << (int) _b200_iface->read_reg(0x1dd) << std::endl;
        std::cout << std::hex << "read reg1df: " << (int) _b200_iface->read_reg(0x1df) << std::endl;
    }

    void setup_adc() {
        double bbbw_mhz = (((_bbpll_freq / 1e6) / _rx_bbf_tunediv) * std::log(2.0)) \
                      / (1.4 * 2 * boost::math::constants::pi<double>());

        if(bbbw_mhz > 28) {
            bbbw_mhz = 28;
        } else if (bbbw_mhz < 0.20) {
            bbbw_mhz = 0.20;
        }

        uint8_t rxbbf_c3_msb = _b200_iface->read_reg(0x1eb) & 0x3F;
        uint8_t rxbbf_c3_lsb = _b200_iface->read_reg(0x1ec) & 0x7F;
        uint8_t rxbbf_r2346 = _b200_iface->read_reg(0x1e6) & 0x07;

        double scale_snr_dB = (_adcclock_freq < 80e6) ? 0 : 2;

        double fsadc = _adcclock_freq / 1e6;

        double rc_timeconst = 0.0;
        if(bbbw_mhz < 18) {
            rc_timeconst = (1 / ((1.4 * 2 * boost::math::constants::pi<double>()) \
                                * (18300 * rxbbf_r2346)
                                * ((160e-15 * rxbbf_c3_msb)
                                    + (10e-15 * rxbbf_c3_lsb) + 140e-15)
                                * (bbbw_mhz * 1e6)));
        } else {
            rc_timeconst = (1 / ((1.4 * 2 * boost::math::constants::pi<double>()) \
                                * (18300 * rxbbf_r2346)
                                * ((160e-15 * rxbbf_c3_msb)
                                    + (10e-15 * rxbbf_c3_lsb) + 140e-15)
                                * (bbbw_mhz * 1e6) * (1 + (0.01 * (bbbw_mhz - 18)))));
        }

        double scale_res = std::sqrt(1 / rc_timeconst);
        double scale_cap = std::sqrt(1 / rc_timeconst);
        double scale_snr = std::pow(10,(scale_snr_dB / 10));
        double maxsnr = 640 / 160;

        uint8_t data[40];
        data[0] = 0;    data[1] = 0; data[2] = 0; data[3] = 0x24;
        data[4] = 0x24; data[5] = 0; data[6] = 0;
        data[7] = std::min(uint8_t(124), uint8_t(std::floor(-0.5
                        + (80.0 * scale_snr * scale_res
                        * std::min(1.0, std::sqrt(maxsnr * fsadc / 640.0))))));
        double data007 = data[7];
        data[8] = std::min(uint8_t(255), uint8_t(std::floor(0.5
                        + ((20.0 * (640.0 / fsadc) * ((data007 / 80.0))
                        / (scale_res * scale_cap))))));
        data[10] = std::min(uint8_t(127), uint8_t(std::floor(-0.5 + (77.0 * scale_res
                        * std::min(1.0, std::sqrt(maxsnr * fsadc / 640.0))))));
        double data010 = data[10];
        data[9] = std::min(uint8_t(127), uint8_t(std::floor(0.8 * data010)));
        data[11] = std::min(uint8_t(255), uint8_t(std::floor(0.5
                        + (20.0 * (640.0 / fsadc) * ((data010 / 77.0)
                        / (scale_res * scale_cap))))));
        data[12] = std::min(uint8_t(127), uint8_t(std::floor(-0.5
                        + (80.0 * scale_res * std::min(1.0,
                        std::sqrt(maxsnr * fsadc / 640.0))))));
        double data012 = data[12];
        data[13] = std::min(uint8_t(255), uint8_t(std::floor(-1.5
                        + (20.0 * (640.0 / fsadc) * ((data012 / 80.0)
                        / (scale_res * scale_cap))))));
        data[14] = 21 * uint8_t(std::floor(0.1 * 640.0 / fsadc));
        data[15] = std::min(uint8_t(127), uint8_t(1.025 * data007));
        double data015 = data[15];
        data[16] = std::min(uint8_t(127), uint8_t(std::floor((data015
                        * (0.98 + (0.02 * std::max(1.0,
                        (640.0 / fsadc) / maxsnr)))))));
        data[17] = data[15];
        data[18] = std::min(uint8_t(127), uint8_t(0.975 * (data010)));
        double data018 = data[18];
        data[19] = std::min(uint8_t(127), uint8_t(std::floor((data018
                        * (0.98 + (0.02 * std::max(1.0,
                        (640.0 / fsadc) / maxsnr)))))));
        data[20] = data[18];
        data[21] = std::min(uint8_t(127), uint8_t(0.975 * data012));
        double data021 = data[21];
        data[22] = std::min(uint8_t(127), uint8_t(std::floor((data021
                        * (0.98 + (0.02 * std::max(1.0,
                        (640.0 / fsadc) / maxsnr)))))));
        data[23] = data[21];
        data[24] = 0x2e;
        data[25] = uint8_t(std::floor(128.0 + std::min(63.0,
                        63.0 * (fsadc / 640.0))));
        data[26] = uint8_t(std::floor(std::min(63.0, 63.0 * (fsadc / 640.0)
                        * (0.92 + (0.08 * (640.0 / fsadc))))));
        data[27] = uint8_t(std::floor(std::min(63.0,
                        32.0 * std::sqrt(fsadc / 640.0))));
        data[28] = uint8_t(std::floor(128.0 + std::min(63.0,
                        63.0 * (fsadc / 640.0))));
        data[29] = uint8_t(std::floor(std::min(63.0,
                        63.0 * (fsadc / 640.0)
                        * (0.92 + (0.08 * (640.0 / fsadc))))));
        data[30] = uint8_t(std::floor(std::min(63.0,
                        32.0 * std::sqrt(fsadc / 640.0))));
        data[31] = uint8_t(std::floor(128.0 + std::min(63.0,
                        63.0 * (fsadc / 640.0))));
        data[32] = uint8_t(std::floor(std::min(63.0,
                        63.0 * (fsadc / 640.0) * (0.92
                        + (0.08 * (640.0 / fsadc))))));
        data[33] = uint8_t(std::floor(std::min(63.0,
                        63.0 * std::sqrt(fsadc / 640.0))));
        data[34] = std::min(uint8_t(127), uint8_t(std::floor(64.0
                        * std::sqrt(fsadc / 640.0))));
        data[35] = 0x40;
        data[36] = 0x40;
        data[37] = 0x2c;
        data[38] = 0x00;
        data[39] = 0x00;

        for(int i=0; i<40; i++) {
            _b200_iface->write_reg(0x200+i, data[i]);
        }
    }

    void calibrate_baseband_dc_offset() {
        _b200_iface->write_reg(0x193, 0x3f);
        _b200_iface->write_reg(0x190, 0x0f);
        _b200_iface->write_reg(0x194, 0x01);

        int count = 0;
        _b200_iface->write_reg(0x016, 0x01);
        while(_b200_iface->read_reg(0x016) & 0x01) {
            if(count > 5) {
                std::cout << "Baseband DC Offset Calibration Failure!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        }
    }

    void calibrate_rf_dc_offset() {
        _b200_iface->write_reg(0x185, 0x20);    // RF DC Offset wait count
        _b200_iface->write_reg(0x186, 0x32);    // RF DC Offset count
        _b200_iface->write_reg(0x187, 0x24);
        _b200_iface->write_reg(0x18b, 0x83);
        _b200_iface->write_reg(0x188, 0x05);
        _b200_iface->write_reg(0x189, 0x30);

        int count = 0;
        _b200_iface->write_reg(0x016, 0x02);
        while(_b200_iface->read_reg(0x016) & 0x02) {
            if(count > 5) {
                std::cout << "RF DC Offset Calibration Failure!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }
    }

    void calibrate_rx_quadrature(void) {
        /* Configure RX Quadrature calibration settings. */
        _b200_iface->write_reg(0x168, 0x03);    // Set tone level for cal
        _b200_iface->write_reg(0x16e, 0x25);    // RX Gain index to use for cal
        _b200_iface->write_reg(0x16a, 0x75);
        _b200_iface->write_reg(0x16b, 0x15);
        _b200_iface->write_reg(0x169, 0xcf);
        _b200_iface->write_reg(0x18b, 0xad);
    }

    void calibrate_tx_quadrature(void) {
        /* TX Quad Cal: write settings, cal. */
        UHD_HERE();

        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            throw uhd::runtime_error("TX Quad Cal started, but not in ALERT!");
        }

        uint8_t maskbits = _b200_iface->read_reg(0x0a3) & 0x3F;
        _b200_iface->write_reg(0x0a0, 0x15);
        _b200_iface->write_reg(0x0a3, 0x00 | maskbits);
        _b200_iface->write_reg(0x0a1, 0x7B);
        _b200_iface->write_reg(0x0a9, 0xff);
        _b200_iface->write_reg(0x0a2, 0x7f);
        _b200_iface->write_reg(0x0a5, 0x01);
        _b200_iface->write_reg(0x0a6, 0x01);
        _b200_iface->write_reg(0x0aa, 0x25);
        _b200_iface->write_reg(0x0a4, 0xf0);
        _b200_iface->write_reg(0x0ae, 0x00);


        /* First, calibrate the baseband DC offset. */
        calibrate_baseband_dc_offset();

        /* Second, calibrate the RF DC offset. */
        calibrate_rf_dc_offset();

        /* Now, calibrate the TX quadrature! */
        int count = 0;
        _b200_iface->write_reg(0x016, 0x10);
        while(_b200_iface->read_reg(0x016) & 0x10) {
            if(count > 5) {
                std::cout << "TX Quadrature Calibration Failure!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }

        std::cout << "0x08E: " << (int) _b200_iface->read_reg(0x08E) << std::endl;
        std::cout << "0x08F: " << (int) _b200_iface->read_reg(0x08F) << std::endl;
    }


    /***********************************************************************
     * Other Misc Setup Functions
     ***********************************************************************/

    void program_mixer_gm_subtable() {
        uint8_t gain[] = {0x78, 0x74, 0x70, 0x6C, 0x68, 0x64, 0x60, 0x5C, 0x58,
                          0x54, 0x50, 0x4C, 0x48, 0x30, 0x18, 0x00};
        uint8_t gm[] = {0x00, 0x0D, 0x15, 0x1B, 0x21, 0x25, 0x29, 0x2C, 0x2F,
                        0x31, 0x33, 0x34, 0x35, 0x3A, 0x3D, 0x3E};

        /* Start the clock. */
        _b200_iface->write_reg(0x13f, 0x02);

        /* Program the GM Sub-table. */
        for(int i = 15; i >= 0; i--) {
            _b200_iface->write_reg(0x138, i);
            _b200_iface->write_reg(0x139, gain[(15 - i)]);
            _b200_iface->write_reg(0x13A, 0x00);
            _b200_iface->write_reg(0x13B, gm[(15 - i)]);
            _b200_iface->write_reg(0x13F, 0x06);
            _b200_iface->write_reg(0x13C, 0x00);
            _b200_iface->write_reg(0x13C, 0x00);
        }

        /* Clear write bit and stop clock. */
        _b200_iface->write_reg(0x13f, 0x02);
        _b200_iface->write_reg(0x13C, 0x00);
        _b200_iface->write_reg(0x13C, 0x00);
        _b200_iface->write_reg(0x13f, 0x00);
    }


    void program_gain_table() {

        uint8_t (*gain_table)[5] = NULL;

        uint8_t new_gain_table;

        if(_rx_freq  < 1300e6) {
            gain_table = gain_table_sub_1300mhz;
            new_gain_table = 1;
        } else if(_rx_freq < 4e9) {
            gain_table = gain_table_1300mhz_to_4000mhz;
            new_gain_table = 2;
        } else if(_rx_freq <= 6e9) {
            gain_table = gain_table_4000mhz_to_6000mhz;
            new_gain_table = 3;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        /* Only re-program the gain table if there has been a band change. */
        if(_curr_gain_table == new_gain_table) {
            return;
        } else {
            _curr_gain_table = new_gain_table;
        }

        /* Start the gain table clock. */
        _b200_iface->write_reg(0x137, 0x1A);

        uint8_t index = 0;
        for(; index < 77; index++) {
            _b200_iface->write_reg(0x130, index);
            _b200_iface->write_reg(0x131, gain_table[index][1]);
            _b200_iface->write_reg(0x132, gain_table[index][2]);
            _b200_iface->write_reg(0x133, gain_table[index][3]);
            _b200_iface->write_reg(0x137, 0x1E);
            _b200_iface->write_reg(0x134, 0x00);
            _b200_iface->write_reg(0x134, 0x00);

            /* FIXME debug
            std::cout.unsetf(std::ios::hex);
            std::cout << "Index: " << (int) index << std::endl;
            std::cout << "Word 1: " << std::hex << (int) gain_table[index][1] << std::endl;
            std::cout << "Word 2: " << std::hex << (int) gain_table[index][2] << std::endl;
            std::cout << "Word 3: " << std::hex << (int) gain_table[index][3] << std::endl;
            */
        }

        for(; index < 91; index++) {
            _b200_iface->write_reg(0x130, index);
            _b200_iface->write_reg(0x131, 0x00);
            _b200_iface->write_reg(0x132, 0x00);
            _b200_iface->write_reg(0x133, 0x00);
            _b200_iface->write_reg(0x137, 0x1E);
            _b200_iface->write_reg(0x134, 0x00);
            _b200_iface->write_reg(0x134, 0x00);
        }

        /* Clear the write bit and stop the gain clock. */
        _b200_iface->write_reg(0x137, 0x1A);
        _b200_iface->write_reg(0x134, 0x00);
        _b200_iface->write_reg(0x134, 0x00);
        _b200_iface->write_reg(0x137, 0x00);

        /* FIXME debug
        index = 0;
        uint8_t word1, word2, word3;
        for(; index < 77; index++) {
            _b200_iface->write_reg(0x130, index);
            word1 = _b200_iface->read_reg(0x134);
            word2 = _b200_iface->read_reg(0x135);
            word3 = _b200_iface->read_reg(0x136);
            _b200_iface->write_reg(0x134, 0x00);
            _b200_iface->write_reg(0x134, 0x00);

            std::cout.unsetf(std::ios::hex);
            std::cout << "Index: " << (int) index << std::endl;
            std::cout << "Word 1: " << std::hex << (int) word1 << std::endl;
            std::cout << "Word 2: " << std::hex << (int) word2 << std::endl;
            std::cout << "Word 3: " << std::hex << (int) word3 << std::endl;
        }
        */
    }

    void setup_gain_control() {
        _b200_iface->write_reg(0x0FA,0xE0);// Gain Control Mode Select
        _b200_iface->write_reg(0x0FB,0x08);// Table, Digital Gain, Man Gain Ctrl
        _b200_iface->write_reg(0x0FC,0x23);// Incr Step Size, ADC Overrange Size
        _b200_iface->write_reg(0x0FD,0x4C);// Max Full/LMT Gain Table Index
        _b200_iface->write_reg(0x0FE,0x44);// Decr Step Size, Peak Overload Time
        _b200_iface->write_reg(0x100,0x6F);// Max Digital Gain
        _b200_iface->write_reg(0x104,0x2F);// ADC Small Overload Threshold
        _b200_iface->write_reg(0x105,0x3A);// ADC Large Overload Threshold
        _b200_iface->write_reg(0x107,0x31);// Large LMT Overload Threshold
        _b200_iface->write_reg(0x108,0x39);// Small LMT Overload Threshold
        _b200_iface->write_reg(0x109,0x23);// Rx1 Full/LMT Gain Index
        _b200_iface->write_reg(0x10A,0x58);// Rx1 LPF Gain Index
        _b200_iface->write_reg(0x10B,0x00);// Rx1 Digital Gain Index
        _b200_iface->write_reg(0x10C,0x23);// Rx2 Full/LMT Gain Index
        _b200_iface->write_reg(0x10D,0x18);// Rx2 LPF Gain Index
        _b200_iface->write_reg(0x10E,0x00);// Rx2 Digital Gain Index
        _b200_iface->write_reg(0x114,0x30);// Low Power Threshold
        _b200_iface->write_reg(0x11A,0x27);// Initial LMT Gain Limit
        _b200_iface->write_reg(0x081,0x00);// Tx Symbol Gain Control
    }

    void setup_synth(std::string which, double vcorate) {
        /* The vcorates in the vco_index array represent lower boundaries for
         * rates. Once we find a match, we use that index to look-up the rest of
         * the register values in the LUT. */
        int vcoindex = 0;
        for(int i = 0; i < 53; i++) {
            vcoindex = i;
            if(vcorate > vco_index[i]) {
                break;
            }
        }

        UHD_ASSERT_THROW(vcoindex < 53);

        uint8_t vco_output_level = synth_cal_lut[vcoindex][0];
        uint8_t vco_varactor = synth_cal_lut[vcoindex][1];
        uint8_t vco_bias_ref = synth_cal_lut[vcoindex][2];
        uint8_t vco_bias_tcf = synth_cal_lut[vcoindex][3];
        uint8_t vco_cal_offset = synth_cal_lut[vcoindex][4];
        uint8_t vco_varactor_ref = synth_cal_lut[vcoindex][5];
        uint8_t charge_pump_curr = synth_cal_lut[vcoindex][6];
        uint8_t loop_filter_c2 = synth_cal_lut[vcoindex][7];
        uint8_t loop_filter_c1 = synth_cal_lut[vcoindex][8];
        uint8_t loop_filter_r1 = synth_cal_lut[vcoindex][9];
        uint8_t loop_filter_c3 = synth_cal_lut[vcoindex][10];
        uint8_t loop_filter_r3 = synth_cal_lut[vcoindex][11];

        if(which == "RX") {
            _b200_iface->write_reg(0x23a, 0x40 | vco_output_level);
            _b200_iface->write_reg(0x239, 0xC0 | vco_varactor);
            _b200_iface->write_reg(0x242, vco_bias_ref | (vco_bias_tcf << 3));
            _b200_iface->write_reg(0x238, (vco_cal_offset << 3));
            _b200_iface->write_reg(0x245, 0x00);
            _b200_iface->write_reg(0x251, vco_varactor_ref);
            _b200_iface->write_reg(0x250, 0x70);
            _b200_iface->write_reg(0x23b, 0x80 | charge_pump_curr);
            _b200_iface->write_reg(0x23e, loop_filter_c1 | (loop_filter_c2 << 4));
            _b200_iface->write_reg(0x23f, loop_filter_c3 | (loop_filter_r1 << 4));
            _b200_iface->write_reg(0x240, loop_filter_r3);
        } else if(which == "TX") {
            _b200_iface->write_reg(0x27a, 0x40 | vco_output_level);
            _b200_iface->write_reg(0x279, 0xC0 | vco_varactor);
            _b200_iface->write_reg(0x282, vco_bias_ref | (vco_bias_tcf << 3));
            _b200_iface->write_reg(0x278, (vco_cal_offset << 3));
            _b200_iface->write_reg(0x285, 0x00);
            _b200_iface->write_reg(0x291, vco_varactor_ref);
            _b200_iface->write_reg(0x290, 0x70);
            _b200_iface->write_reg(0x27b, 0x80 | charge_pump_curr);
            _b200_iface->write_reg(0x27e, loop_filter_c1 | (loop_filter_c2 << 4));
            _b200_iface->write_reg(0x27f, loop_filter_c3 | (loop_filter_r1 << 4));
            _b200_iface->write_reg(0x280, loop_filter_r3);
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    }


    /***********************************************************************
     * Implementation
     ***********************************************************************/

    b200_codec_ctrl_impl(b200_iface::sptr iface, wb_iface::sptr ctrl) {
        /* Initialize shadow registers. */
        fpga_bandsel = 0x00;
        reg_vcodivs = 0x00;
        reg_inputsel = 0x70;
        reg_rxfilt = 0x00;
        reg_txfilt = 0x00;
        //reg_bbpll = 0x02; // no clock Set by default in _impl for 40e6 reference
        reg_bbpll = 0x12; // yes clock Set by default in _impl for 40e6 reference
        reg_bbftune_config = 0x1e;
        reg_bbftune_mode = 0x1e;

        /* Initialize internal fields. */
        _rx_freq = 0.0;
        _tx_freq = 0.0;
        _baseband_bw = 0.0;
        _req_clock_rate = 0.0;
        _req_coreclk = 0.0;
        _bbpll_freq = 0.0;
        _adcclock_freq = 0.0;
        _rx_bbf_tunediv = 0;
        _curr_gain_table = 0;

        /* Initialize control interfaces. */
        _b200_iface = iface;
        _ctrl = ctrl;

        /* Reset the device. */
        _b200_iface->write_reg(0x000,0x01);
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

        //enable clocks
        _b200_iface->write_reg(0x009, BOOST_BINARY( 00010111 ) );
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));

        /* Tune the BBPLL, write TX and RX FIRS. */
        setup_rates(30.72e6);

        /* Setup data ports (FDD dual port DDR CMOS):
         *      FDD dual port DDR CMOS no swap.
         *      Force TX on one port, RX on the other. */
        _b200_iface->write_reg(0x010, 0xc8);
        _b200_iface->write_reg(0x011, 0x00);
        _b200_iface->write_reg(0x012, 0x02);

        /* Data delay for TX and RX data clocks */
        _b200_iface->write_reg(0x006, 0x0F);
        _b200_iface->write_reg(0x007, 0x00);

        /* Setup AuxDAC */
        _b200_iface->write_reg(0x018, 0x00); // AuxDAC1 Word[9:2]
        _b200_iface->write_reg(0x019, 0x00); // AuxDAC2 Word[9:2]
        _b200_iface->write_reg(0x01A, 0x00); // AuxDAC1 Config and Word[1:0]
        _b200_iface->write_reg(0x01B, 0x00); // AuxDAC2 Config and Word[1:0]
        _b200_iface->write_reg(0x023, 0xFF); // AuxDAC Manaul/Auto Control
        _b200_iface->write_reg(0x026, 0x00); // AuxDAC Manual Select Bit/GPO Manual Select
        _b200_iface->write_reg(0x030, 0x00); // AuxDAC1 Rx Delay
        _b200_iface->write_reg(0x031, 0x00); // AuxDAC1 Tx Delay
        _b200_iface->write_reg(0x032, 0x00); // AuxDAC2 Rx Delay
        _b200_iface->write_reg(0x033, 0x00); // AuxDAC2 Tx Delay

        /* Setup AuxADC */
        _b200_iface->write_reg(0x00B, 0x00); // Temp Sensor Setup (Offset)
        _b200_iface->write_reg(0x00C, 0x00); // Temp Sensor Setup (Temp Window)
        _b200_iface->write_reg(0x00D, 0x03); // Temp Sensor Setup (Periodic Measure)
        _b200_iface->write_reg(0x00F, 0x04); // Temp Sensor Setup (Decimation)
        _b200_iface->write_reg(0x01C, 0x10); // AuxADC Setup (Clock Div)
        _b200_iface->write_reg(0x01D, 0x01); // AuxADC Setup (Decimation/Enable)

        /* Setup control outputs. */
        _b200_iface->write_reg(0x035, 0x07);
        _b200_iface->write_reg(0x036, 0xFF);

        /* Setup GPO */
        _b200_iface->write_reg(0x03a, 0x27); //set delay register
        _b200_iface->write_reg(0x020, 0x00); // GPO Auto Enable Setup in RX and TX
        _b200_iface->write_reg(0x027, 0x03); // GPO Manual and GPO auto value in ALERT
        _b200_iface->write_reg(0x028, 0x00); // GPO_0 RX Delay
        _b200_iface->write_reg(0x029, 0x00); // GPO_1 RX Delay
        _b200_iface->write_reg(0x02A, 0x00); // GPO_2 RX Delay
        _b200_iface->write_reg(0x02B, 0x00); // GPO_3 RX Delay
        _b200_iface->write_reg(0x02C, 0x00); // GPO_0 TX Delay
        _b200_iface->write_reg(0x02D, 0x00); // GPO_1 TX Delay
        _b200_iface->write_reg(0x02E, 0x00); // GPO_2 TX Delay
        _b200_iface->write_reg(0x02F, 0x00); // GPO_3 TX Delay

        _b200_iface->write_reg(0x261, 0x00); //RX LO power
        _b200_iface->write_reg(0x2a1, 0x00); //TX LO power
        _b200_iface->write_reg(0x248, 0x0b); //en RX VCO LDO
        _b200_iface->write_reg(0x288, 0x0b); //en TX VCO LDO
        _b200_iface->write_reg(0x246, 0x02); //pd RX cal Tcf
        _b200_iface->write_reg(0x286, 0x02); //pd TX cal Tcf
        _b200_iface->write_reg(0x249, 0x8e); //rx vco cal length
        _b200_iface->write_reg(0x289, 0x8e); //rx vco cal length
        _b200_iface->write_reg(0x23b, 0x80); //set RX MSB?, FIXME 0x89 magic cp
        _b200_iface->write_reg(0x27b, 0x80); //"" TX //FIXME 0x88 see above
        _b200_iface->write_reg(0x243, 0x0d); //set rx prescaler bias
        _b200_iface->write_reg(0x283, 0x0d); //"" TX

        _b200_iface->write_reg(0x23d, 0x00); // Clear half VCO cal clock setting
        _b200_iface->write_reg(0x27d, 0x00); // Clear half VCO cal clock setting

        /* The order of the following process is EXTREMELY important. If the
         * below functions are modified at all, device initialization and
         * calibration might be broken in the process! */

        _b200_iface->write_reg(0x015, 0x04); //dual synth mode, synth en ctrl en
        _b200_iface->write_reg(0x014, 0x05); //use SPI for TXNRX ctrl, to ALERT, TX on
        _b200_iface->write_reg(0x013, 0x01); //enable ENSM
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        calibrate_synth_charge_pumps();

        tune_helper("RX", 800e6, false);
        tune_helper("TX", 850e6, false);

        program_mixer_gm_subtable();
        program_gain_table();
        setup_gain_control();

        calibrate_baseband_rx_analog_filter();
        calibrate_baseband_tx_analog_filter();
        calibrate_rx_TIAs();
        calibrate_secondary_tx_filter();

        setup_adc();

        calibrate_tx_quadrature();
        calibrate_rx_quadrature();

        _b200_iface->write_reg(0x012, 0x02); // cals done, set PPORT config
        _b200_iface->write_reg(0x013, 0x01); // Set ENSM FDD bit
        _b200_iface->write_reg(0x015, 0x04); // dual synth mode, synth en ctrl en

        /* Default TX attentuation to 10dB on both TX1 and TX2 */
        _b200_iface->write_reg(0x073, 0x28);
        _b200_iface->write_reg(0x074, 0x00);
        _b200_iface->write_reg(0x075, 0x28);
        _b200_iface->write_reg(0x076, 0x00);

        /* Setup RSSI Measurements */
        _b200_iface->write_reg(0x150, 0x0E); // RSSI Measurement Duration 0, 1
        _b200_iface->write_reg(0x151, 0x00); // RSSI Measurement Duration 2, 3
        _b200_iface->write_reg(0x152, 0xFF); // RSSI Weighted Multiplier 0
        _b200_iface->write_reg(0x153, 0x00); // RSSI Weighted Multiplier 1
        _b200_iface->write_reg(0x154, 0x00); // RSSI Weighted Multiplier 2
        _b200_iface->write_reg(0x155, 0x00); // RSSI Weighted Multiplier 3
        _b200_iface->write_reg(0x156, 0x00); // RSSI Delay
        _b200_iface->write_reg(0x157, 0x00); // RSSI Wait
        _b200_iface->write_reg(0x158, 0x0D); // RSSI Mode Select
        _b200_iface->write_reg(0x15C, 0x67); // Power Measurement Duration

        /* Not sure why we do this, but it's what ADI does. */
        _b200_iface->write_reg(0x002, reg_txfilt);
        _b200_iface->write_reg(0x003, reg_rxfilt);

        /* Set TXers & RXers on (only works in FDD mode) */
        _b200_iface->write_reg(0x014, 0x21);
    }

    double set_gain(const std::string &which, const std::string &name, \
            const double value) {
        if(which[0] == 'R') {
            /* Indexing the gain tables requires an offset from the requested
             * amount of total gain in dB:
             *      < 1300MHz: dB + 5
             *      >= 1300MHz and < 4000MHz: dB + 3
             *      >= 4000MHz and <= 6000MHz: dB + 14
             */
            int gain_offset = 0;
            if(_rx_freq < 1300e6) {
                gain_offset = 5;
            } else if(_rx_freq < 4000e6) {
                gain_offset = 3;
            } else {
                gain_offset = 14;
            }

            int gain_index = value + gain_offset;

            UHD_VAR(gain_index);

            /* Clip the gain values to the proper min/max gain values. */
            if(gain_index > 76) gain_index = 76;
            if(gain_index < 0) gain_index = 0;

            if(which[3] == 'A') {
                _b200_iface->write_reg(0x109, gain_index);
            } else {
                _b200_iface->write_reg(0x10c, gain_index);
            }

            return gain_index - gain_offset;
        } else { //TX gain
            /* Setting the below bits causes a change in the TX attenuation word
             * to immediately take effect. */
            _b200_iface->write_reg(0x077, 0x40);
            _b200_iface->write_reg(0x07c, 0x40);

            /* Each gain step is -0.25dB. Calculate the attenuation necessary
             * for the requested gain, convert it into gain steps, then write
             * the attenuation word. Max gain (so zero attenuation) is 89.75. */
            double atten = get_gain_range("TX_A", "").stop() - value;
            int attenreg = atten * 4;
            if(which[3] == 'A') {
                _b200_iface->write_reg(0x073, attenreg & 0xFF);
                _b200_iface->write_reg(0x074, (attenreg >> 8) & 0x01);
            } else {
                _b200_iface->write_reg(0x075, attenreg & 0xFF);
                _b200_iface->write_reg(0x076, (attenreg >> 8) & 0x01);
            }
            return get_gain_range("TX_A", "").stop() - (double(attenreg)/ 4);
        }
    }

    int get_num_taps(int max_num_taps) {

        int num_taps = 0;
        int num_taps_list[] = {16, 32, 48, 64, 80, 96, 112, 128};
        for(int i = 1; i < 8; i++) {
            if(max_num_taps >= num_taps_list[i]) {
                continue;
            } else {
                num_taps = num_taps_list[i - 1];
                break;
            }
        } if(num_taps == 0) { num_taps = 128; }

        return num_taps;
    }


    double set_clock_rate(const double req_rate) {

        /* We must be in the SLEEP / WAIT state to do this. If we aren't already
         * there, transition the ENSM to State 0. */
        uint8_t current_state = _b200_iface->read_reg(0x017) & 0x0F;
        switch(current_state) {
            case 0x05:
                /* We are in the ALERT state. */
                _b200_iface->write_reg(0x014, 0x21);
                boost::this_thread::sleep(boost::posix_time::milliseconds(5));
                _b200_iface->write_reg(0x014, 0x00);
                break;

            case 0x0A:
                /* We are in the FDD state. */
                _b200_iface->write_reg(0x014, 0x00);
                break;

            default:
                throw uhd::runtime_error("AD9361 is in unknown state!");
                break;
        };

        double rate = setup_rates(req_rate);

        /* Transition to the ALERT state and calibrate everything. */
        _b200_iface->write_reg(0x015, 0x04); //dual synth mode, synth en ctrl en
        _b200_iface->write_reg(0x014, 0x05); //use SPI for TXNRX ctrl, to ALERT, TX on
        _b200_iface->write_reg(0x013, 0x01); //enable ENSM
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        calibrate_synth_charge_pumps();

        tune_helper("RX", _rx_freq, false);
        tune_helper("TX", _tx_freq, false);

        program_mixer_gm_subtable();
        program_gain_table();
        setup_gain_control();

        calibrate_baseband_rx_analog_filter();
        calibrate_baseband_tx_analog_filter();
        calibrate_rx_TIAs();
        calibrate_secondary_tx_filter();

        setup_adc();

        calibrate_tx_quadrature();
        calibrate_rx_quadrature();

        _b200_iface->write_reg(0x012, 0x02); // cals done, set PPORT config
        _b200_iface->write_reg(0x013, 0x01); // Set ENSM FDD bit
        _b200_iface->write_reg(0x015, 0x04); // dual synth mode, synth en ctrl en

        /* End the function in the same state as the entry state. */
        switch(current_state) {
            case 0x05:
                /* We are already in ALERT. */
                break;

            case 0x0A:
                /* Transition back to FDD. */
                _b200_iface->write_reg(0x002, reg_txfilt);
                _b200_iface->write_reg(0x003, reg_rxfilt);
                _b200_iface->write_reg(0x014, 0x21);
                break;

            default:
                throw uhd::runtime_error("AD9361 is in unknown state!");
                break;
        };

        return rate;
    }

    double setup_rates(const double rate) {
        if(rate > 61.44e6) {
            throw uhd::runtime_error("Requested master clock rate outside range!");
        }

        if(rate == _req_clock_rate) {
            return _req_clock_rate;
        }

        _req_clock_rate = rate;

        UHD_VAR(rate);

        /* Set the decimation values based on clock rate. We are setting default
         * values for the interpolation filters, as well, although they may
         * change after the ADC clock has been set. */
        int divfactor = 0;
        int tfir = 0;
        if(rate <= 20e6) {
            // RX1 enabled, 2, 2, 2, 2
            reg_rxfilt = BOOST_BINARY( 01011110 ) ;

            // TX1 enabled, 2, 2, 2, 2
            reg_txfilt = BOOST_BINARY( 01011110 ) ;

            divfactor = 16;
            tfir = 2;
        } else if((rate > 20e6) && (rate < 23e6)) {
           // RX1 enabled, 3, 2, 2, 2
            reg_rxfilt = BOOST_BINARY( 01101110 ) ;

            // TX1 enabled, 3, 1, 2, 2
            reg_txfilt = BOOST_BINARY( 01100110 ) ;

            divfactor = 24;
            tfir = 2;
        } else if((rate >= 23e6) && (rate < 41e6)) {
            // RX1 enabled, 2, 2, 2, 2
            reg_rxfilt = BOOST_BINARY( 01011110 ) ;

            // TX1 enabled, 1, 2, 2, 2
            reg_txfilt = BOOST_BINARY( 01001110 ) ;

            divfactor = 16;
            tfir = 2;
        } else if((rate >= 41e6) && (rate <= 56e6)) {
            // RX1 enabled, 3, 1, 2, 2
            reg_rxfilt = BOOST_BINARY( 01100110 ) ;

            // TX1 enabled, 3, 1, 1, 2
            reg_txfilt = BOOST_BINARY( 01100010 ) ;

            divfactor = 12;
            tfir = 2;
        } else if((rate > 56e6) && (rate <= 61.44e6)) {
            // RX1 enabled, 3, 1, 1, 2
            reg_rxfilt = BOOST_BINARY( 01100010 ) ;

            // TX1 enabled, 3, 1, 1, 1
            reg_txfilt = BOOST_BINARY( 01100001 ) ;

            divfactor = 6;
            tfir = 1;
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        /* Tune the BBPLL to get the ADC and DAC clocks. */
        double adcclk = set_coreclk(rate * divfactor);
        double dacclk = adcclk;

        /* The DAC clock must be <= 336e6, and is either the ADC clock or 2x the
         * ADC clock.*/
        if(adcclk > 336e6) {
            /* Make the DAC clock = ADC/2, and bypass the TXFIR. */
            reg_bbpll = reg_bbpll | 0x08;
            // TODO: I do not believe the below line is still necessary after
            // the complete rework of the above txfilt logic. Leaving it here in
            // case we need to revisit later.
            //reg_txfilt = (reg_txfilt & 0xFC);//FIXME WTF? | 0x01;

            dacclk = adcclk / 2.0;
        } else {
            reg_bbpll = reg_bbpll & 0xF7;
        }

        std::cout << std::hex << "reg_txfilt: " << (int) reg_txfilt << std::endl;
        std::cout << std::hex << "reg_rxfilt: " << (int) reg_rxfilt << std::endl;

        /* Set the dividers / interpolators in Catalina. */
        _b200_iface->write_reg(0x002, reg_txfilt);
        _b200_iface->write_reg(0x003, reg_rxfilt);
        _b200_iface->write_reg(0x004, reg_inputsel);
        _b200_iface->write_reg(0x00A, reg_bbpll);

        _baseband_bw = (adcclk / divfactor);

        /* Setup the RX and TX FIR filters. Scale the number of taps based on
         * the clock speed. */
        int max_tx_taps = 16 * std::min(int((dacclk / rate) + 0.5), \
                std::min(4 * (1 << tfir), 8));
        int max_rx_taps = std::min((16 * int(adcclk / rate)), 128);

        int num_tx_taps = get_num_taps(max_tx_taps);
        int num_rx_taps = get_num_taps(max_rx_taps);

        setup_tx_fir(num_tx_taps);
        setup_rx_fir(num_rx_taps);

        return _baseband_bw;
    }

    double set_coreclk(const double rate) {
        if(rate == _req_coreclk) {
            return _req_coreclk;
        }

        _req_coreclk = rate;

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
        int i=1;
        for(; i<=6; i++) {
            vcodiv = 1<<i;
            vcorate = rate * vcodiv;
            if(vcorate >= vcomin && vcorate <= vcomax) break;
        }
        if(i == 7) throw uhd::runtime_error("BBVCO can't find valid VCO rate!");
        //TODO this will pick the low rate for threshold values, should eval
        //whether vcomax or vcomin has better performance

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;
        //std::cout << "BB Nint: " << nint << " Nfrac: " << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);

        //scale CP current according to VCO rate
        const double icp_baseline = 150e-6;
        const double freq_baseline = 1280e6;
        double icp = icp_baseline * (actual_vcorate / freq_baseline);
        int icp_reg = (icp/25e-6) - 1;

        _b200_iface->write_reg(0x045, 0x00);            //REFCLK / 1 to BBPLL
        _b200_iface->write_reg(0x046, icp_reg & 0x3F);  //CP current
        _b200_iface->write_reg(0x048, 0xe8);            //BBPLL loop filters
        _b200_iface->write_reg(0x049, 0x5b);            //BBPLL loop filters
        _b200_iface->write_reg(0x04a, 0x35);            //BBPLL loop filters

        _b200_iface->write_reg(0x04b, 0xe0);
        _b200_iface->write_reg(0x04e, 0x10);            //max accuracy

        _b200_iface->write_reg(0x043, nfrac & 0xFF);        //Nfrac[7:0]
        _b200_iface->write_reg(0x042, (nfrac >> 8) & 0xFF); //Nfrac[15:8]
        _b200_iface->write_reg(0x041, (nfrac >> 16) & 0xFF);//Nfrac[23:16]
        _b200_iface->write_reg(0x044, nint);                //Nint

        calibrate_lock_bbpll();

        //use XTALN input, CLKOUT=XTALN (40MHz ref out to FPGA)
        reg_bbpll = (reg_bbpll & 0xF8) | i;

        _bbpll_freq = actual_vcorate;
        _adcclock_freq = (actual_vcorate / vcodiv);

        return _adcclock_freq;
    }

    double tune(const std::string &which, const double value) {
        return tune_helper(which, value, true);
    }

    double tune_helper(const std::string &which, const double value, bool do_cal) {
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

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);
        double actual_lo = actual_vcorate / vcodiv;

        double return_freq = 0.0;
        if(which[0] == 'R') {
            if(value < 2.2e9) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_B;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x30;
                //std::cout << "FPGA BANDSEL_B; CAT BANDSEL C" << std::endl;
            } else if((value >= 2.2e9) && (value < 4e9)) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_A;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x0C;
                //std::cout << "FPGA BANDSEL_A; CAT BANDSEL B" << std::endl;
            } else if((value >= 4e9) && (value <= 6e9)) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_C;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x03;
                //std::cout << "FPGA BANDSEL_C; CAT BANDSEL A" << std::endl;
            } else {
                UHD_THROW_INVALID_CODE_PATH();
            }
            _ctrl->poke32(TOREG(SR_MISC+1), fpga_bandsel);
            _b200_iface->write_reg(0x004, reg_inputsel);

            /* Store vcodiv setting. */
            reg_vcodivs = (reg_vcodivs & 0xF0) | (i & 0x0F);

            /* Setup the synthesizer. */
            setup_synth("RX", actual_vcorate);

            //tune that shit
            _b200_iface->write_reg(0x233, nfrac & 0xFF);
            _b200_iface->write_reg(0x234, (nfrac >> 8) & 0xFF);
            _b200_iface->write_reg(0x235, (nfrac >> 16) & 0xFF);
            _b200_iface->write_reg(0x232, (nint >> 8) & 0xFF);
            _b200_iface->write_reg(0x231, nint & 0xFF);
            _b200_iface->write_reg(0x005, reg_vcodivs);

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x247) & 0x02) == 0) {
                std::cout << "RX PLL NOT LOCKED" << std::endl;
            }

            _rx_freq = actual_lo;

            if(do_cal) {
                program_gain_table();
            }

            return_freq = actual_lo;

        } else {
            if(value < 3e9) {
                fpga_bandsel = (fpga_bandsel & 0xE7) | TX_BANDSEL_A;
                reg_inputsel = reg_inputsel | 0x40;
                //std::cout << "FPGA BANDSEL_A; CAT BANDSEL B" << std::endl;
            } else if((value >= 3e9) && (value <= 6e9)) {
                fpga_bandsel = (fpga_bandsel & 0xE7) | TX_BANDSEL_B;
                reg_inputsel = reg_inputsel & 0xBF;
                //std::cout << "FPGA BANDSEL_B; CAT BANDSEL A" << std::endl;
            } else {
                UHD_THROW_INVALID_CODE_PATH();
            }
            _ctrl->poke32(TOREG(SR_MISC+1), fpga_bandsel);
            _b200_iface->write_reg(0x004, reg_inputsel);

            /* Store vcodiv setting. */
            reg_vcodivs = (reg_vcodivs & 0x0F) | ((i & 0x0F) << 4);

            /* Setup the synthesizer. */
            setup_synth("TX", actual_vcorate);

            //tuning yo
            _b200_iface->write_reg(0x273, nfrac & 0xFF);
            _b200_iface->write_reg(0x274, (nfrac >> 8) & 0xFF);
            _b200_iface->write_reg(0x275, (nfrac >> 16) & 0xFF);
            _b200_iface->write_reg(0x272, (nint >> 8) & 0xFF);
            _b200_iface->write_reg(0x271, nint & 0xFF);
            _b200_iface->write_reg(0x005, reg_vcodivs);

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x287) & 0x02) == 0) {
                std::cout << "TX PLL NOT LOCKED" << std::endl;
            }

            _tx_freq = actual_lo;

            return_freq = actual_lo;
        }

        if(do_cal) {
            /* If we aren't already in the ALERT state, we will need to return to
             * the FDD state after calibration. */
            bool not_in_alert = false;
            if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
                /* Force the device into the ALERT state. */
                not_in_alert = true;
                _b200_iface->write_reg(0x014, 0x01);
            }

            calibrate_tx_quadrature();
            calibrate_rx_quadrature();

            /* If we were in the FDD state, return it now. */
            if(not_in_alert) {
                _b200_iface->write_reg(0x014, 0x21);
            }
        }

        return return_freq;
    }

    double set_filter_bw(const std::string &which) {
        return 0.0;
    }


private:
    b200_iface::sptr _b200_iface;
    wb_iface::sptr _ctrl;
    double _rx_freq, _tx_freq;
    double _baseband_bw, _bbpll_freq, _adcclock_freq;
    double _req_clock_rate, _req_coreclk;
    uint16_t _rx_bbf_tunediv;
    uint8_t _curr_gain_table;

    /* Shadow register fields.*/
    boost::uint32_t fpga_bandsel;
    boost::uint8_t reg_vcodivs;
    boost::uint8_t reg_inputsel;
    boost::uint8_t reg_rxfilt;
    boost::uint8_t reg_txfilt;
    boost::uint8_t reg_bbpll;
    boost::uint8_t reg_bbftune_config;
    boost::uint8_t reg_bbftune_mode;
};

/***********************************************************************
 * Make an instance of the implementation
 **********************************************************************/
b200_codec_ctrl::sptr b200_codec_ctrl::make(b200_iface::sptr iface, wb_iface::sptr ctrl)
{
    return sptr(new b200_codec_ctrl_impl(iface, ctrl));
}
