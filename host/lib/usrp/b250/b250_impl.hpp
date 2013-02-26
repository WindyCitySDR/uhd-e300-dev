//
// Copyright 2013 Ettus Research LLC
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

#ifndef INCLUDED_B250_IMPL_HPP
#define INCLUDED_B250_IMPL_HPP

#include <uhd/property_tree.hpp>
#include <uhd/device.hpp>
#include "wb_iface.hpp"
#include "b250_fw_common.h"
#include "b250_ctrl.hpp"
#include "b250_fw_ctrl.hpp"
#include <uhd/transport/udp_zero_copy.hpp>
#include "spi_core_3000.hpp"
#include "b250_adc_ctrl.hpp"

static const std::string B250_FW_FILE_NAME = "b250_fw.bin";


#define XB_DST_E0 0
#define XB_DST_E1 1
#define XB_DST_R0 2
#define XB_DST_R1 3

#define HOST_ADDR 0
#define FPGA_ADDR 1

#define RADIO0_CTRL_DST 1
#define RADIO0_CTRL_SRC 1 //on the host...

//static const boost::uint32_t B200_R0_CTRL_SID = 0x00000001; //needs to end in a 1 for mux
static const boost::uint32_t B200_R0_CTRL_SID = (HOST_ADDR << 24) | (RADIO0_CTRL_SRC << 16) | (FPGA_ADDR << 8) | (RADIO0_CTRL_DST << 0);

class b250_impl : public uhd::device
{
public:
    b250_impl(const uhd::device_addr_t &);
    ~b250_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &){return uhd::rx_streamer::sptr();}
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &){return uhd::tx_streamer::sptr();}
    bool recv_async_msg(uhd::async_metadata_t &, double){return false;}

private:
    uhd::property_tree::sptr _tree;
    //device properties interface
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

    uhd::transport::udp_zero_copy::sptr make_transport(const std::string &addr, const boost::uint32_t sid);
    void register_loopback_self_test(void);

    //perifs in the zpu
    wb_iface::sptr _zpu_ctrl;
    spi_core_3000::sptr _zpu_spi;
    void setup_ad9510_clock(spi_core_3000::sptr);

    //perifs in the radio core
    b250_ctrl::sptr _radio_ctrl0;
    b250_ctrl::sptr _radio_ctrl1;
    spi_core_3000::sptr _radio_spi0;
    spi_core_3000::sptr _radio_spi1;
    b250_adc_ctrl::sptr _adc_ctrl0;
    b250_adc_ctrl::sptr _adc_ctrl1;

    void set_ad9146_dac(spi_core_3000::sptr);
    void setup_crossbar_router(void);

};

#endif /* INCLUDED_B250_IMPL_HPP */
