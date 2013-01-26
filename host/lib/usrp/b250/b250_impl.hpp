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
#include <uhd/transport/udp_simple.hpp>
#include <uhd/utils/byteswap.hpp>

static const std::string B250_FW_FILE_NAME = "b250_fw.bin";
static const size_t B250_FW_NUM_BYTES = (1 << 14);

struct b250_ctrl_iface : wb_iface
{
    b250_ctrl_iface(uhd::transport::udp_simple::sptr udp):
        udp(udp), poke_count(0){}

    uhd::transport::udp_simple::sptr udp;
    size_t poke_count;

    void poke32(wb_addr_type addr, boost::uint32_t data)
    {
        boost::uint32_t request[2] = {};
        request[0] = uhd::htonx(addr);
        request[1] = uhd::htonx(data);
        boost::uint32_t reply[2] = {};
        while (udp->recv(boost::asio::buffer(reply), 0.0)){} //flush
        udp->send(boost::asio::buffer(request));
        //const size_t nbytes = udp->recv(boost::asio::buffer(reply));
        //UHD_ASSERT_THROW(nbytes == 8);
        //UHD_ASSERT_THROW(uhd::ntohx(reply[0]) == addr);
        if ((poke_count++ & 0xf) == 0) this->peek32(0); //stop poke from overflowing
    }

    boost::uint32_t peek32(wb_addr_type addr)
    {
        boost::uint32_t request[1] = {};
        request[0] = uhd::htonx(addr);
        boost::uint32_t reply[2] = {};
        while (udp->recv(boost::asio::buffer(reply), 0.0)){} //flush
        udp->send(boost::asio::buffer(request));
        const size_t nbytes = udp->recv(boost::asio::buffer(reply));
        UHD_ASSERT_THROW(nbytes == 8);
        UHD_ASSERT_THROW(uhd::ntohx(reply[0]) == addr);
        return uhd::ntohx(reply[1]);
    }

    void poke16(wb_addr_type, boost::uint16_t)
    {
        throw uhd::not_implemented_error("b250_ctrl_iface::poke16");
    }

    boost::uint16_t peek16(wb_addr_type)
    {
        throw uhd::not_implemented_error("b250_ctrl_iface::peek16");
    }
};

class b250_impl : public uhd::device
{
public:
    b250_impl(const uhd::device_addr_t &);
    ~b250_impl(void);

    //the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args){}
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args){}
    bool recv_async_msg(uhd::async_metadata_t &, double){}

private:
    uhd::property_tree::sptr _tree;
    //device properties interface
    uhd::property_tree::sptr get_tree(void) const
    {
        return _tree;
    }

    void load_fw(const std::string &file_name);

    wb_iface::sptr ctrl;

};

#endif /* INCLUDED_B250_IMPL_HPP */
