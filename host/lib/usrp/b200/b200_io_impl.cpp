//
// Copyright 2012-2013 Ettus Research LLC
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
#include "b200_impl.hpp"
#include "validate_subdev_spec.hpp"
#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"
#include "async_packet_handler.hpp"
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

void b200_impl::update_streamer_rates(void)
{
    //update the tick rate on all existing streamers -> thread safe
    {
        boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(_rx_streamer.lock());
        if (my_streamer) my_streamer->set_tick_rate(this->get_tick_rate());
        if (my_streamer) my_streamer->set_samp_rate(this->get_rx_sample_rate());
    }
    {
        boost::shared_ptr<sph::send_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamer.lock());
        if (my_streamer) my_streamer->set_tick_rate(this->get_tick_rate());
        if (my_streamer) my_streamer->set_samp_rate(this->get_tx_sample_rate());
    }
}

void b200_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec)
{
    //sanity checking
    if (spec.size()) validate_subdev_spec(_tree, spec, "rx");
    UHD_ASSERT_THROW(spec.size() <= 2);

    _fe_enb_map["RX1"] = false;
    _fe_enb_map["RX2"] = false;

    if (spec.size() == 1)
    {
        UHD_ASSERT_THROW(spec[0].db_name == "A");
        _fe_enb_map["RX1"] = spec[0].sd_name == "RX1";
        _fe_enb_map["RX2"] = spec[0].sd_name == "RX2";
    }
    if (spec.size() == 2)
    {
        //TODO we can support swapping at a later date, only this combo is supported
        UHD_ASSERT_THROW(spec[0].db_name == "A");
        UHD_ASSERT_THROW(spec[0].sd_name == "RX1");
        UHD_ASSERT_THROW(spec[1].db_name == "A");
        UHD_ASSERT_THROW(spec[1].sd_name == "RX2");
        _fe_enb_map["RX1"] = true;
        _fe_enb_map["RX2"] = true;
    }

    this->update_enables();
}

void b200_impl::update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &spec)
{
    //sanity checking
    if (spec.size()) validate_subdev_spec(_tree, spec, "tx");
    UHD_ASSERT_THROW(spec.size() <= 2);

    _fe_enb_map["TX1"] = false;
    _fe_enb_map["TX2"] = false;

    if (spec.size() == 1)
    {
        UHD_ASSERT_THROW(spec[0].db_name == "A");
        _fe_enb_map["TX1"] = spec[0].sd_name == "TX1";
        _fe_enb_map["TX2"] = spec[0].sd_name == "TX2";
    }
    if (spec.size() == 2)
    {
        //TODO we can support swapping at a later date, only this combo is supported
        UHD_ASSERT_THROW(spec[0].db_name == "A");
        UHD_ASSERT_THROW(spec[0].sd_name == "TX1");
        UHD_ASSERT_THROW(spec[1].db_name == "A");
        UHD_ASSERT_THROW(spec[1].sd_name == "TX2");
        _fe_enb_map["TX1"] = true;
        _fe_enb_map["TX2"] = true;
    }

    this->update_enables();
}

void b200_impl::issue_stream_cmd(const size_t dspno, const uhd::stream_cmd_t &cmd)
{
    if (dspno == 0)
    {
        uhd::stream_cmd_t new_cmd = cmd;
        if (_gpio_state.mimo_rx) new_cmd.num_samps *= 2;
        _rx_framer->issue_stream_command(new_cmd);
    }
}

static void b200_if_hdr_unpack_le(
    const boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_unpack_le(packet_buff, if_packet_info);
}

static void b200_if_hdr_pack_le(
    boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_pack_le(packet_buff, if_packet_info);
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool b200_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    return _async_md->pop_with_timed_wait(async_metadata, timeout);
}

void b200_impl::handle_async_task(
    uhd::transport::zero_copy_if::sptr xport,
    boost::shared_ptr<async_md_type> async_md,
    b200_uart::sptr gpsdo_uart
)
{
    managed_recv_buffer::sptr buff = xport->get_recv_buff();
    if (not buff or buff->size() < 8) return;
    const boost::uint32_t sid = uhd::wtohx(buff->cast<const boost::uint32_t *>()[1]);

    //if the packet is a control response
    if (sid == B200_RESP_MSG_SID)
    {
        _ctrl->push_response(buff->cast<const boost::uint32_t *>());
        return;
    }

    //if the packet is a local control response
    if (sid == B200_LOCAL_RESP_SID)
    {
        _local_ctrl->push_response(buff->cast<const boost::uint32_t *>());
        return;
    }

    //if the packet is a uart message
    if (sid == B200_RX_GPS_UART_SID)
    {
        gpsdo_uart->handle_uart_packet(buff);
        return;
    }

    //or maybe the packet is a TX async message
    if (sid == B200_TX_MSG_SID_BASE)
    {
        //extract packet info
        vrt::if_packet_info_t if_packet_info;
        if_packet_info.num_packet_words32 = buff->size()/sizeof(boost::uint32_t);
        const boost::uint32_t *packet_buff = buff->cast<const boost::uint32_t *>();

        //unpacking can fail
        try
        {
            b200_if_hdr_unpack_le(packet_buff, if_packet_info);
        }
        catch(const std::exception &ex)
        {
            UHD_MSG(error) << "Error parsing ctrl packet: " << ex.what() << std::endl;
            return;
        }

        //fill in the async metadata
        async_metadata_t metadata;
        load_metadata_from_buff(uhd::wtohx<boost::uint32_t>, metadata, if_packet_info, packet_buff, _tick_rate);
        async_md->push_with_pop_on_full(metadata);
        standard_async_msg_prints(metadata);
        return;
    }

    //doh!
    UHD_MSG(error) << "Got a ctrl packet with unknown SID " << sid << std::endl;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr b200_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b200_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
    const size_t nchans = args.channels.size();
    UHD_ASSERT_THROW(nchans == 1 or nchans == 2);

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = _data_transport->get_recv_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp/nchans);

    //init some streamer stuff
    my_streamer->set_vrt_unpacker(&b200_if_hdr_unpack_le);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_le";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = nchans;
    my_streamer->set_converter(id);

    _rx_framer->clear();
    _rx_framer->set_nsamps_per_packet(spp & ~1); //seems to be a good place to set this
    _rx_framer->set_sid(B200_RX_DATA_SID_BASE);
    _rx_framer->setup(args);
    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &zero_copy_if::get_recv_buff, _data_transport, _1
    ), true /*flush*/);
    my_streamer->set_overflow_handler(0, boost::bind(
        &rx_vita_core_3000::handle_overflow, _rx_framer
    ));
    my_streamer->set_issue_stream_cmd(0, boost::bind(
        &b200_impl::issue_stream_cmd, this, 0, _1
    ));
    _rx_streamer = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    this->update_streamer_rates();

    //set the mimo bit as per number of channels
    _gpio_state.mimo_rx = (nchans == 2)? 1 : 0;
    update_gpio_state();

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr b200_impl::get_tx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b200_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
    const size_t nchans = args.channels.size();
    UHD_ASSERT_THROW(nchans == 1 or nchans == 2);

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        //+ sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    static const size_t bpp = _data_transport->get_send_frame_size() - hdr_size;
    const size_t spp = bpp/convert::get_bytes_per_item(args.otw_format);

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::send_packet_streamer> my_streamer = boost::make_shared<sph::send_packet_streamer>(spp/nchans);

    //init some streamer stuff
    my_streamer->set_vrt_packer(&b200_if_hdr_pack_le);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = nchans;
    id.output_format = args.otw_format + "_item32_le";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    _tx_deframer->clear();
    _tx_deframer->setup(args);

    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &zero_copy_if::get_send_buff, _data_transport, _1
    ));
    my_streamer->set_async_receiver(boost::bind(
        &bounded_buffer<async_metadata_t>::pop_with_timed_wait, _async_md, _1, _2
    ));
    my_streamer->set_xport_chan_sid(0, true, B200_TX_DATA_SID_BASE);
    my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet
    _tx_streamer = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    this->update_streamer_rates();

    //set the mimo bit as per number of channels
    _gpio_state.mimo_tx = (nchans == 2)? 1 : 0;
    update_gpio_state();

    return my_streamer;
}
