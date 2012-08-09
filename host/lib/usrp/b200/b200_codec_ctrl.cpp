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
        return uhd::meta_range_t(0.0, 60e6); //FIXME TODO
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

    void setup_fir(const std::string &which, uint8_t num_taps, uint16_t *coeffs) {
        uint16_t base;
        if(which == "RX") base = 0x0f0;
        else base = 0x060;

        /* Write the filter configuration. */
        uint8_t reg_numtaps = (((num_taps / 16) - 1) & 0x07) << 5;

        _b200_iface->write_reg(base+5, reg_numtaps | 0x1a); //enable filter clk
        _b200_iface->write_reg(base+6, 0x02); //filter gain
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        for(int addr=0; addr < (num_taps / 2); addr++) {
            _b200_iface->write_reg(base+0, addr);
            _b200_iface->write_reg(base+1, (coeffs[addr]) & 0xff);
            _b200_iface->write_reg(base+2, (coeffs[addr] >> 8) & 0xff);
            _b200_iface->write_reg(base+5, 0xfe);
            _b200_iface->write_reg(base+4, 0x00);
            _b200_iface->write_reg(base+4, 0x00);
        }
        //it's symmetric, so we write it out again backwards
        for(int addr=0; addr < (num_taps / 2); addr++) {
            _b200_iface->write_reg(base+0, addr+64);
            _b200_iface->write_reg(base+1, (coeffs[64-addr]) & 0xff);
            _b200_iface->write_reg(base+2, (coeffs[64-addr] >> 8) & 0xff);
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
        //calibrate freq (0x04B[7], toggle 0x03F[2] 1 then 0)
        _b200_iface->write_reg(0x04e, 0x10); //slow down cal for better accuracy
        _b200_iface->write_reg(0x04B, 0xE0);
        _b200_iface->write_reg(0x03F, 0x05);
        _b200_iface->write_reg(0x03F, 0x01); //keep bbpll on

        _b200_iface->write_reg(0x04c, 0x86); //increase Kv and phase margin (?)
        _b200_iface->write_reg(0x04d, 0x01);
        _b200_iface->write_reg(0x04d, 0x05);

        /* Wait for BBPLL lock. */
        int count = 0;
        while(!(_b200_iface->read_reg(0x05e) & 0x80)) {
            if(count > 5) {
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
        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        bool not_in_alert = false;
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            /* Force the device into the ALERT state. */
            not_in_alert = true;
            _b200_iface->write_reg(0x014, 0x0f);
        }

        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 28e6 and 0.2e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 28e6) {
            bbbw = 28e6;
        } else if (bbbw < 0.20e6) {
            bbbw = 0.20e6;
        }

        double rxtune_clk = ((1.4 * bbbw *
                boost::math::constants::pi<double>()) / std::log(2));

        _rx_bbf_tunediv = std::min(511, int(std::ceil(_bbpll_freq / rxtune_clk)));

        reg_bbftune_config = (reg_bbftune_config & 0xFE) \
                             | ((_rx_bbf_tunediv >> 8) & 0x0001);

        _b200_iface->write_reg(0x1f8, (_rx_bbf_tunediv & 0x00FF));
        _b200_iface->write_reg(0x1f9, reg_bbftune_config);

        double bbbw_mhz = bbbw / 1e6;

        double temp = ((bbbw_mhz - std::floor(bbbw_mhz)) * 1000) / 7.8125;
        uint8_t bbbw_khz = std::min(127, int(std::floor(temp + 0.5)));

        _b200_iface->write_reg(0x1fb, uint8_t(bbbw_mhz));
        _b200_iface->write_reg(0x1fc, bbbw_khz);

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

        /* If we were in the FDD state, return it now. */
        if(not_in_alert) {
            _b200_iface->write_reg(0x014, 0x21);
        }

        return bbbw;
    }

    double calibrate_baseband_tx_analog_filter() {
        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        bool not_in_alert = false;
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            /* Force the device into the ALERT state. */
            not_in_alert = true;
            _b200_iface->write_reg(0x014, 0x0f);
        }

        /* For filter tuning, baseband BW is half the complex BW, and must be
         * between 28e6 and 0.2e6. */
        double bbbw = _baseband_bw / 2.0;
        if(bbbw > 20e6) {
            bbbw = 20e6;
        } else if (bbbw < 0.625e6) {
            bbbw = 0.625e6;
        }

        double txtune_clk = ((1.6 * bbbw *
                boost::math::constants::pi<double>()) / std::log(2));

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

        /* If we were in the FDD state, return it now. */
        if(not_in_alert) {
            _b200_iface->write_reg(0x014, 0x21);
        }

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

        int bbbw_mhz = int((bbbw / 1e6));

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
            reg0d0 = 0x0c;
        } else if(((bbbw_mhz * 2) > 9) && ((bbbw_mhz * 2) <= 24)) {
            reg0d0 = 0x056;
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

        _b200_iface->write_reg(0x0d0, reg0d0);
        _b200_iface->write_reg(0x0d1, reg0d1);
        _b200_iface->write_reg(0x0d2, reg0d2);
        _b200_iface->write_reg(0x0d3, 0x60);
    }

    void calibrate_rx_TIAs() {

        uint8_t reg1eb = _b200_iface->read_reg(0x1eb) & 0x3F;
        uint8_t reg1ec = _b200_iface->read_reg(0x1ec) & 0x7F;
        uint8_t reg1e6 = _b200_iface->read_reg(0x1e6) & 0x07;
        uint8_t reg1db, reg1dc, reg1dd, reg1de, reg1df;

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

            uint8_t temp = std::min(127, int(std::floor(0.5 + ((CTIA_fF - 400) / 320))));
            reg1dd = temp;
            reg1df = temp;
        } else {
            uint8_t temp = std::floor(0.5 + ((CTIA_fF - 400) / 40)) + 0x40;
            reg1dc = temp;
            reg1de = temp;
            reg1dd = 0;
            reg1df = 0;
        }

        _b200_iface->write_reg(0x1db, reg1db);
        _b200_iface->write_reg(0x1dc, reg1dc);
        _b200_iface->write_reg(0x1dd, reg1dd);
        _b200_iface->write_reg(0x1de, reg1de);
        _b200_iface->write_reg(0x1df, reg1df);
    }

    void setup_adc() {

        double bbbw = ((_bbpll_freq / _rx_bbf_tunediv) * std::log(2)) \
                      / (1.4 * 2 * boost::math::constants::pi<double>());

        if(bbbw > 28e6) {
            bbbw = 28e6;
        } else if (bbbw < 0.20e6) {
            bbbw = 0.20e6;
        }

        uint8_t rxbbf_c3_msb = _b200_iface->read_reg(0x1eb) & 0x3F;
        uint8_t rxbbf_c3_lsb = _b200_iface->read_reg(0x1ec) & 0x7F;
        uint8_t rxbbf_r2346 = _b200_iface->read_reg(0x1e6) & 0x07;

        double scale_snr_dB = (_bbpll_freq < 80e6) ? 0 : 2;

        double rc_timeconst = 0.0;
        if(bbbw < 18e6) {
            rc_timeconst = (1 / ((1.4 * 2 * boost::math::constants::pi<double>()) \
                                * (18300 * rxbbf_r2346)
                                * ((160e-15 * rxbbf_c3_msb)
                                    + (10e-15 * rxbbf_c3_lsb) + 140e-15)
                                * (bbbw)));
        } else {
            rc_timeconst = (1 / ((1.4 * 2 * boost::math::constants::pi<double>()) \
                                * (18300 * rxbbf_r2346)
                                * ((160e-15 * rxbbf_c3_msb)
                                    + (10e-15 * rxbbf_c3_lsb) + 140e-15)
                                * (bbbw) * (1 + (0.01 * (bbbw - 18e6)))));
        }

        double scale_res = std::sqrt(1 / rc_timeconst);
        double scale_cap = std::sqrt(1 / rc_timeconst);
        double scale_snr = std::pow(10,(scale_snr_dB / 10));
        double maxsnr = 640 / 160;

        uint8_t data[40];
        data[0] = 0;    data[1] = 0; data[2] = 0; data[3] = 0x24;
        data[4] = 0x24; data[5] = 0; data[6] = 0;
        data[7] = std::min(uint8_t(124), uint8_t(std::floor(-0.5
                        + (80 * scale_snr * scale_res
                        * std::min(1.0, std::sqrt(maxsnr * _bbpll_freq / 640))))));
        data[8] = std::min(uint8_t(255), uint8_t(std::floor(0.5
                        + (20 * (640 / _bbpll_freq) * (data[7] / 80)
                        / (scale_res * scale_cap)))));
        data[10] = std::min(uint8_t(127), uint8_t(std::floor(-0.5 + (77 * scale_res
                        * std::min(1.0, std::sqrt(maxsnr * _bbpll_freq / 640))))));
        data[9] = std::min(uint8_t(127), uint8_t(std::floor(0.8 * data[10])));
        data[11] = std::min(uint8_t(255), uint8_t(std::floor(0.5
                        + (20 * (640 / _bbpll_freq) * (data[10] / 77)
                        / (scale_res * scale_cap)))));
        data[12] = std::min(uint8_t(127), uint8_t(std::floor(-0.5
                        + (80 * scale_res * std::min(1.0,
                        std::sqrt(maxsnr * _bbpll_freq / 640))))));
        data[13] = std::min(uint8_t(255), uint8_t(std::floor(-1.5
                        + (20 * (640 / _bbpll_freq) * (data[12] / 80)
                        / (scale_res * scale_cap)))));
        data[14] = 21 * uint8_t(std::floor(0.1 * 640 / _bbpll_freq));
        data[15] = std::min(uint8_t(127), uint8_t(1.025 * data[7]));
        data[16] = std::min(uint8_t(127), uint8_t(std::floor(data[15]
                        * (0.98 + (0.02 * std::max(1.0,
                        (640 / _bbpll_freq) / maxsnr))))));
        data[17] = data[15];
        data[18] = std::min(uint8_t(127), uint8_t(0.975 * data[10]));
        data[19] = std::min(uint8_t(127), uint8_t(std::floor(data[18]
                        * (0.98 + (0.02 * std::max(1.0,
                        (640 / _bbpll_freq) / maxsnr))))));
        data[20] = data[18];
        data[21] = std::min(uint8_t(127), uint8_t(0.975 * data[12]));
        data[22] = std::min(uint8_t(127), uint8_t(std::floor(data[21]
                        * (0.98 + (0.02 * std::max(1.0,
                        (640 / _bbpll_freq) / maxsnr))))));
        data[23] = data[21];
        data[24] = 0x2e;
        data[25] = uint8_t(std::floor(128 + std::min(63.0,
                        63 * (_bbpll_freq / 640))));
        data[26] = uint8_t(std::floor(std::min(63.0, 63 * (_bbpll_freq / 640)
                        * (0.92 + (0.08 * (640 / _bbpll_freq))))));
        data[27] = uint8_t(std::floor(std::min(63.0,
                        32 * std::sqrt(_bbpll_freq / 640))));
        data[28] = uint8_t(std::floor(128 + std::min(63.0,
                        63 * (_bbpll_freq / 640))));
        data[29] = uint8_t(std::floor(std::min(63.0,
                        63 * (_bbpll_freq / 640)
                        * (0.92 + (0.08 * (640 / _bbpll_freq))))));
        data[30] = uint8_t(std::floor(std::min(63.0,
                        32 * std::sqrt(_bbpll_freq / 640))));
        data[31] = uint8_t(std::floor(128 + std::min(63.0,
                        63 * (_bbpll_freq / 640))));
        data[32] = uint8_t(std::floor(std::min(63.0,
                        63 * (_bbpll_freq / 640) * (0.92
                        + (0.08 * (640 / _bbpll_freq))))));
        data[33] = uint8_t(std::floor(std::min(63.0,
                        63 * std::sqrt(_bbpll_freq))));
        data[34] = std::min(uint8_t(127), uint8_t(std::floor(64
                        * std::sqrt(_bbpll_freq / 640))));
        data[35] = 0x40;
        data[36] = 0x40;
        data[37] = 0x2c;
        data[38] = 0x00;
        data[39] = 0x00;

        for(int i=0; i<40; i++) {
            _b200_iface->write_reg(0x200+i, data[i]);
//            UHD_VAR(((int) data[i]));
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
        //FIXME Needs to be run for all bands, too???

        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        bool not_in_alert = false;
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            /* Force the device into the ALERT state. */
            not_in_alert = true;
            _b200_iface->write_reg(0x014, 0x0f);
        }

        _b200_iface->write_reg(0x185, 0x20);
        _b200_iface->write_reg(0x186, 0x32);
        _b200_iface->write_reg(0x187, 0x24);
        _b200_iface->write_reg(0x188, 0x05);
        _b200_iface->write_reg(0x189, 0x30);
        _b200_iface->write_reg(0x18b, 0xad);

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

        /* If we were in the FDD state, return it now. */
        if(not_in_alert) {
            _b200_iface->write_reg(0x014, 0x21);
        }
    }

    void calibrate_rx_quadrature(void) {
        _b200_iface->write_reg(0x186, 0x32);
        _b200_iface->write_reg(0x187, 0x24);
        _b200_iface->write_reg(0x188, 0x05);
        _b200_iface->write_reg(0x189, 0x30);
        _b200_iface->write_reg(0x18b, 0xad);

        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        bool not_in_alert = false;
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            /* Force the device into the ALERT state. */
            not_in_alert = true;
            _b200_iface->write_reg(0x014, 0x0f);
        }

         /* Disable RX cal free run, set other cal settings. */
        _b200_iface->write_reg(0x168, 0x03);
        _b200_iface->write_reg(0x169, 0xC0);
        _b200_iface->write_reg(0x16a, 0x75);
        _b200_iface->write_reg(0x16b, 0x15);
        _b200_iface->write_reg(0x16e, 0x25);

        /* RX Quad Cal: power down TX mixer, tune TX LO to passband of RX
         * spectrum, run the calibration, retune the TX LO, re-enable the TX
         * mixer. */
        _b200_iface->write_reg(0x057, 0x33);

        double old_tx_freq = _tx_freq;
        tune("CAL", (_rx_freq + (_baseband_bw / 4)));

        int count = 0;
        _b200_iface->write_reg(0x016, 0x20);
        while(_b200_iface->read_reg(0x016) & 0x20) {
            if(count > 5) {
                std::cout << "RX Quadrature Calibration Failure!" << std::endl;
                break;
            }

            count++;
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }

        /* Re-enable TX mixer and re-tune TX LO. */
        if((old_tx_freq >= 100e6) && (old_tx_freq <= 6e9)) {
            tune("CAL", old_tx_freq);
        }
        _b200_iface->write_reg(0x057, 0x30);

        /* Enable Quad Cal Tracking. */
        _b200_iface->write_reg(0x169, 0xcf);

        /* If we were in the FDD state, return it now. */
        if(not_in_alert) {
            _b200_iface->write_reg(0x014, 0x21);
        }
    }

    void calibrate_tx_quadrature(void) {
        /* If we aren't already in the ALERT state, we will need to return to
         * the FDD state after calibration. */
        bool not_in_alert = false;
        if((_b200_iface->read_reg(0x017) & 0x0F) != 5) {
            /* Force the device into the ALERT state. */
            not_in_alert = true;
            _b200_iface->write_reg(0x014, 0x0f);
        }

        /* TX Quad Cal: write settings, cal. */
        uint8_t maskbits = _b200_iface->read_reg(0x0a3) & 0x3F;
        _b200_iface->write_reg(0x0a0, 0x15);
        _b200_iface->write_reg(0x0a3, 0x00 | maskbits);
        _b200_iface->write_reg(0x0a1, 0x7B);
        _b200_iface->write_reg(0x0a9, 0xff);
        _b200_iface->write_reg(0x0a2, 0x7f);
        _b200_iface->write_reg(0x0a5, 0x01);
        _b200_iface->write_reg(0x0a6, 0x01);
        _b200_iface->write_reg(0x0aa, 0x22);
        _b200_iface->write_reg(0x0a4, 0xf0);
        _b200_iface->write_reg(0x0ae, 0x00);

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

        /* If we were in the FDD state, return it now. */
        if(not_in_alert) {
            _b200_iface->write_reg(0x014, 0x21);
        }
    }


    /***********************************************************************
     * Implementation
     ***********************************************************************/

    b200_codec_ctrl_impl(b200_iface::sptr iface, wb_iface::sptr ctrl) {
        /* Initialize shadow registers. */
        fpga_bandsel = 0x00;
        reg_vcodivs = 0x00;
        reg_inputsel = 0x00;
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
        _clock_rate = 0.0;
        _bbpll_freq = 0.0;
        _rx_bbf_tunediv = 0;

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
        _b200_iface->write_reg(0x009, BOOST_BINARY( 00010111 ) );
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));

        /* Disable all calibration bits. */
        _b200_iface->write_reg(0x1e2, 0x03);
        _b200_iface->write_reg(0x1e3, 0x03);
        _b200_iface->write_reg(0x23d, 0x00);
        _b200_iface->write_reg(0x27d, 0x00);

        /* Setup data ports (FDD dual port DDR CMOS) */
        //FDD dual port DDR CMOS no swap
        _b200_iface->write_reg(0x010, BOOST_BINARY( 11001000 ) );
        _b200_iface->write_reg(0x011, BOOST_BINARY( 00000000 ) );
         //force TX on one port, RX on the other
        _b200_iface->write_reg(0x012, BOOST_BINARY( 00000010 ) );

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
        _b200_iface->write_reg(0x23b,0x80); //set RX MSB?, FIXME 0x89 magic cp
        _b200_iface->write_reg(0x27b,0x80); //"" TX //FIXME 0x88 see above
        _b200_iface->write_reg(0x243,0x0d); //set rx prescaler bias
        _b200_iface->write_reg(0x283,0x0d); //"" TX

        _b200_iface->write_reg(0x245,0x00); //set RX VCO cal ref Tcf
        _b200_iface->write_reg(0x250,0x70); //set RX VCO varactor ref Tcf
        _b200_iface->write_reg(0x285,0x00); //"" TX
        _b200_iface->write_reg(0x290,0x70); //"" TX
        _b200_iface->write_reg(0x239,0xc1); //init RX ALC
        _b200_iface->write_reg(0x279,0xc1); //"" TX

         //enable ENSM
        _b200_iface->write_reg(0x013, BOOST_BINARY( 00000001 ) );
        //dual synth mode, synth en ctrl en
        _b200_iface->write_reg(0x015, BOOST_BINARY( 00000111 ) );
        //use SPI for TXNRX ctrl, to ALERT, TX on
        _b200_iface->write_reg(0x014, BOOST_BINARY( 00100101 ) );

        /* RX Mix Voltage settings - only change with apps engineer help. */
        _b200_iface->write_reg(0x1d5, 0x3f);
        _b200_iface->write_reg(0x1c0, 0x03);

        calibrate_synth_charge_pumps();

        set_clock_rate(30.72e6);

        calibrate_baseband_dc_offset();

        tune("RX", 800e6);
        tune("TX", 850e6);

        setup_adc();

        calibrate_baseband_rx_analog_filter();
        calibrate_baseband_tx_analog_filter();
        calibrate_secondary_tx_filter();
        calibrate_rx_TIAs();

        //gm subtable here

        //gain table here

        //ian magic
        _b200_iface->write_reg(0x014, 0x0f);
        _b200_iface->write_reg(0x014, 0x21);
    }

    double set_gain(const std::string &which, const std::string &name, \
            const double value) {
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

    double set_clock_rate(const double rate) {
        if(rate > 61.44e6) {
            throw uhd::runtime_error("Requested master clock rate outside range!");
        }

        if(rate == _clock_rate) {
            return _clock_rate;
        }

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
            reg_txfilt = BOOST_BINARY( 01010110 ) ;

            divfactor = 16;
            tfir = 2;
        } else if((rate > 20e6) && (rate <= 22e6)) {
           // RX1 enabled, 3, 2, 2, 2
            reg_rxfilt = BOOST_BINARY( 01101110 ) ;

            // TX1 enabled, 3, 1, 2, 2
            reg_txfilt = BOOST_BINARY( 01100110 ) ;

            divfactor = 24;
            tfir = 2;
        } else if((rate > 22e6) && (rate <= 40e6)) {
            // RX1 enabled, 2, 2, 2, 2
            reg_rxfilt = BOOST_BINARY( 01011110 ) ;

            // TX1 enabled, 1, 2, 2, 2
            reg_txfilt = BOOST_BINARY( 01001110 ) ;

            divfactor = 16;
            tfir = 2;
        } else if((rate > 40e6) && (rate <= 56e6)) {
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
            tfir = 2;
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
            reg_txfilt = (reg_txfilt & 0xFC);//FIXME WTF? | 0x01;
            tfir = 1;

            dacclk = adcclk / 2.0;
        }

        /* Set the dividers / interpolators in Catalina. */
        _b200_iface->write_reg(0x00A, reg_bbpll);
        _b200_iface->write_reg(0x002, reg_txfilt);
        _b200_iface->write_reg(0x003, reg_rxfilt);

        _baseband_bw = (adcclk / divfactor);
        _clock_rate = rate;

        /* Setup the RX and TX FIR filters. Scale the number of taps based on
         * the clock speed. */
        int max_rx_taps = std::min((16 * int(adcclk / rate)), 128);
        int max_tx_taps = 16 * std::min(int(std::floor(dacclk / rate)), \
                std::min(4 * (1 << tfir), 8));

        int num_rx_taps = get_num_taps(max_rx_taps);
        int num_tx_taps = get_num_taps(max_tx_taps);

        setup_rx_fir(num_rx_taps);
        setup_tx_fir(num_tx_taps);

        /* Run through other necessary calibrations after a BBPLL tune. */
        if((_rx_freq != 0.0) && (_tx_freq != 0.0)) {
            calibrate_baseband_rx_analog_filter();
            calibrate_baseband_tx_analog_filter();
            calibrate_secondary_tx_filter();
            calibrate_rx_TIAs();

            setup_adc();
        }

        return _baseband_bw;
    }

    double set_coreclk(const double rate) {
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
//        std::cout << "BB Nint: " << nint << " Nfrac: " << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);

        _b200_iface->write_reg(0x044, nint); //Nint
        _b200_iface->write_reg(0x041, (nfrac >> 16) & 0xFF); //Nfrac[23:16]
        _b200_iface->write_reg(0x042, (nfrac >> 8) & 0xFF); //Nfrac[15:8]
        _b200_iface->write_reg(0x043, nfrac & 0xFF); //Nfrac[7:0]

        //use XTALN input, CLKOUT=XTALN (40MHz ref out to FPGA)
        reg_bbpll = (reg_bbpll & 0xF8) | i;
        _b200_iface->write_reg(0x00A, reg_bbpll);   //set BBPLL divider
        _b200_iface->write_reg(0x045, 0x00);        //REFCLK / 1 to BBPLL

        //CP filter recommended coefficients, don't change unless you have a clue
        _b200_iface->write_reg(0x048, 0xe8);
        _b200_iface->write_reg(0x049, 0x5b);
        _b200_iface->write_reg(0x04a, 0x35);

        //scale CP current according to VCO rate
        const double icp_baseline = 150e-6;
        const double freq_baseline = 1280e6;
        double icp = icp_baseline * actual_vcorate / freq_baseline;
//        std::cout << "Settled on CP current: " << icp * 1e6 << "uA" << std::endl;
        int icp_reg = icp/25e-6 + 1;

        //CP current
        _b200_iface->write_reg(0x046, icp_reg & 0x3F);

        calibrate_lock_bbpll();

        _bbpll_freq = (actual_vcorate / vcodiv);
        return _bbpll_freq;
    }

    double tune(const std::string &which, const double value) {
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

//        std::cout << "RF VCO rate: " << vcorate << std::endl;

        int nint = vcorate / fref;
        int nfrac = ((vcorate / fref) - nint) * modulus;
//        std::cout << std::dec << "RF Nint: " << nint << " Nfrac: "
//            << nfrac << " vcodiv: " << vcodiv << std::endl;

        double actual_vcorate = fref * (nint + double(nfrac)/modulus);
        double actual_lo = actual_vcorate / vcodiv;

        double return_freq = 0.0;
        if(which[0] == 'R') {
            if(value < 2.2e9) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_B;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x30;
//                std::cout << "FPGA BANDSEL_B; CAT BANDSEL C" << std::endl;
            } else if((value >= 2.2e9) && (value < 4e9)) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_A;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x0C;
//                std::cout << "FPGA BANDSEL_A; CAT BANDSEL B" << std::endl;
            } else if((value >= 4e9) && (value <= 6e9)) {
                fpga_bandsel = (fpga_bandsel & 0xF8) | RX_BANDSEL_C;
                reg_inputsel = (reg_inputsel & 0xC0) | 0x03;
//                std::cout << "FPGA BANDSEL_C; CAT BANDSEL A" << std::endl;
            } else {
                UHD_THROW_INVALID_CODE_PATH();
            }
            _ctrl->poke32(TOREG(SR_MISC+1), fpga_bandsel);
            _b200_iface->write_reg(0x004, reg_inputsel);

            /* Store vcodiv setting. */
            reg_vcodivs = (reg_vcodivs & 0xF0) | (i & 0x0F);

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
            _b200_iface->write_reg(0x005, reg_vcodivs);

            _b200_iface->write_reg(0x236, 0x6b); //undoc vco settings
            _b200_iface->write_reg(0x237, 0x65);
            _b200_iface->write_reg(0x238, 0x71); //force vco tune <8

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x247) & 0x02) == 0) {
                std::cout << "RX PLL NOT LOCKED" << std::endl;
            }

            _rx_freq = actual_lo;

            calibrate_rx_quadrature();

            return_freq = actual_lo;

        } else {
            if(value < 3e9) {
                fpga_bandsel = (fpga_bandsel & 0xE7) | TX_BANDSEL_A;
                reg_inputsel = reg_inputsel | 0x40;
//                std::cout << "FPGA BANDSEL_A; CAT BANDSEL B" << std::endl;
            } else if((value >= 3e9) && (value <= 6e9)) {
                fpga_bandsel = (fpga_bandsel & 0xE7) | TX_BANDSEL_B;
                reg_inputsel = reg_inputsel & 0xBF;
//                std::cout << "FPGA BANDSEL_B; CAT BANDSEL A" << std::endl;
            } else {
                UHD_THROW_INVALID_CODE_PATH();
            }
            _ctrl->poke32(TOREG(SR_MISC+1), fpga_bandsel);
            _b200_iface->write_reg(0x004, reg_inputsel);

            /* Store vcodiv setting. */
            reg_vcodivs = (reg_vcodivs & 0x0F) | ((i & 0x0F) << 4);

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
            _b200_iface->write_reg(0x005, reg_vcodivs);

            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            if((_b200_iface->read_reg(0x287) & 0x02) == 0) {
                std::cout << "TX PLL NOT LOCKED" << std::endl;
            }

            _tx_freq = actual_lo;

            /* If the tune call wasn't invoked during a calibration routine, run the
             * quadrature calibration. */
            if(which[0] != 'C') {
                calibrate_tx_quadrature();
            }

            return_freq = actual_lo;
        }

        calibrate_rf_dc_offset();

        return return_freq;
    }

    double set_filter_bw(const std::string &which) {
        /* If the BBPLL hasn't been tuned & calibrated yet, return. */
        if(_bbpll_freq == 0) {
            return 0.0;
        }

        //set up BB filter
        if(which == "TX") {
            double bw = calibrate_baseband_tx_analog_filter();
            calibrate_secondary_tx_filter();

            return bw;
        } else {
            double bw = calibrate_baseband_rx_analog_filter();
            calibrate_rx_TIAs();

            return bw;
        }
    }


private:
    b200_iface::sptr _b200_iface;
    wb_iface::sptr _ctrl;
    double _rx_freq, _tx_freq;
    double _baseband_bw, _bbpll_freq;
    double _clock_rate;
    uint16_t _rx_bbf_tunediv;

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
