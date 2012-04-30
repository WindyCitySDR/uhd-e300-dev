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

#ifndef INCLUDED_B200_IMPL_HPP
#define INCLUDED_B200_IMPL_HPP

#include "b200_iface.hpp"
#include "b200_ctrl.hpp"
#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/utils/pimpl.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/transport/usb_zero_copy.hpp>
#include <boost/weak_ptr.hpp>

static const std::string     B200_FW_FILE_NAME = "usrp_b200_fw.bin";
static const std::string     B200_FPGA_FILE_NAME = "usrp_b200_fpga.bin";
static const boost::uint16_t B200_FW_COMPAT_NUM = 0x03;
static const boost::uint16_t B200_FPGA_COMPAT_NUM = 0x09;
static const size_t          B200_MAX_PKT_BYTE_LIMIT = 2048;
static const boost::uint32_t B200_ASYNC_MSG_SID0 = 1;
static const boost::uint32_t B200_ASYNC_MSG_SID1 = 2;
static const boost::uint32_t B200_CTRL_MSG_SID = 3;

//! Implementation guts
class b200_impl : public uhd::device {
public:
    //structors
    b200_impl(const uhd::device_addr_t &);
    ~b200_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
    bool recv_async_msg(uhd::async_metadata_t &, double);

private:
    uhd::property_tree::sptr _tree;

    //controllers
    b200_iface::sptr _iface;
    b200_ctrl::sptr _ctrl;

    //transports
    uhd::transport::zero_copy_if::sptr _data_transport, _ctrl_transport;

    //handle io stuff
    UHD_PIMPL_DECL(io_impl) _io_impl;
    void io_init(void);

    //device properties interface
    uhd::property_tree::sptr get_tree(void) const{
        return _tree;
    }

    std::vector<boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
    std::vector<boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;

    void set_mb_eeprom(const uhd::usrp::mboard_eeprom_t &);
    void check_fw_compat(void);
    void check_fpga_compat(void);
};

#endif /* INCLUDED_B200_IMPL_HPP */
