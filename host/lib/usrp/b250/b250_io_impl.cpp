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

#include "b250_regs.hpp"
#include "b250_impl.hpp"
#include "validate_subdev_spec.hpp"
#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"
#include "async_packet_handler.hpp"
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;


/***********************************************************************
 * Setup dboard muxing for IQ
 **********************************************************************/
void b250_impl::update_rx_subdev_spec(const subdev_spec_t &spec)
{
    fs_path root = "/mboards/0/dboards";

    //sanity checking
    validate_subdev_spec(_tree, spec, "rx");

    //setup mux for this spec
    bool fe_swapped = false;
    for (size_t i = 0; i < spec.size(); i++)
    {
        const std::string conn = _tree->access<std::string>(root / spec[i].db_name / "rx_frontends" / spec[i].sd_name / "connection").get();
        if (i == 0 and (conn == "QI" or conn == "Q")) fe_swapped = true;
        _rx_dsp->set_mux(conn, fe_swapped); //TODO which frontend?
    }
    //TODO _rx_fe->set_mux(fe_swapped);

    /*
    //compute the new occupancy and resize
    _mbc[which_mb].rx_chan_occ = spec.size();
    size_t nchan = 0;
    BOOST_FOREACH(const std::string &mb, _mbc.keys()) nchan += _mbc[mb].rx_chan_occ;
    */
}

void b250_impl::update_tx_subdev_spec(const subdev_spec_t &spec)
{
    /*
    fs_path root = "/mboards/" + which_mb + "/dboards";

    //sanity checking
    validate_subdev_spec(_tree, spec, "tx", which_mb);

    //set the mux for this spec
    const std::string conn = _tree->access<std::string>(root / spec[0].db_name / "tx_frontends" / spec[0].sd_name / "connection").get();
    _mbc[which_mb].tx_fe->set_mux(conn);

    //compute the new occupancy and resize
    _mbc[which_mb].tx_chan_occ = spec.size();
    size_t nchan = 0;
    BOOST_FOREACH(const std::string &mb, _mbc.keys()) nchan += _mbc[mb].tx_chan_occ;
    */
}

/***********************************************************************
 * VITA stuff
 **********************************************************************/
static void b250_if_hdr_unpack_be(
    const boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_VRLP;
    return vrt::if_hdr_unpack_be(packet_buff, if_packet_info);
}

static void b250_if_hdr_pack_be(
    boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_VRLP;
    return vrt::if_hdr_pack_be(packet_buff, if_packet_info);
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool b250_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    return false;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr b250_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b250_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    //const size_t bpp = 1400/*TOD0*/ - hdr_size;
    //const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = 360;//unsigned(args.args.cast<double>("spp", bpp/bpi));

    //allocate sid and create transport
    sid_config_t data_config;
    data_config.router_addr_there = B250_DEVICE_THERE;
    data_config.dst_prefix = B250_RADIO_DEST_PREFIX_RX;
    data_config.router_dst_there = B250_XB_DST_R0;
    data_config.router_dst_here = B250_XB_DST_E0;
    const boost::uint32_t data_sid = this->allocate_sid(data_config);
    udp_zero_copy::sptr data_xport = this->make_transport(_addr, data_sid);
    sleep(1);

    UHD_MSG(status) << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

    //send a fc packet to enable some stuff
    {
        managed_send_buffer::sptr buff = data_xport->get_send_buff(0.0);
        if (not buff){
            throw uhd::runtime_error("fifo ctrl timed out getting a send buffer");
        }
        boost::uint32_t *pkt = buff->cast<boost::uint32_t *>();

        //load packet info
        vrt::if_packet_info_t packet_info;
        packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_VRLP;
        packet_info.packet_type = vrt::if_packet_info_t::PACKET_TYPE_CONTEXT;
        packet_info.num_payload_words32 = 2;
        packet_info.num_payload_bytes = packet_info.num_payload_words32*sizeof(boost::uint32_t);
        packet_info.packet_count = 0;
        packet_info.sob = false;
        packet_info.eob = false;
        packet_info.sid = data_sid;
        packet_info.has_sid = true;
        packet_info.has_cid = false;
        packet_info.has_tsi = false;
        packet_info.has_tsf = false;
        packet_info.has_tlr = false;

        //load header
        vrt::if_hdr_pack_be(pkt, packet_info);

        //load payload
        pkt[packet_info.num_header_words32+0] = uhd::htonx<boost::uint32_t>(~0);
        pkt[packet_info.num_header_words32+1] = uhd::htonx<boost::uint32_t>(~0);

        //send the buffer over the interface
        buff->commit(sizeof(boost::uint32_t)*(packet_info.num_packet_words32));
        UHD_HERE();
        buff.reset();
    }

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->set_vrt_unpacker(&b250_if_hdr_unpack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_be";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    _rx_framer->set_nsamps_per_packet(spp); //seems to be a good place to set this
    _rx_framer->set_sid((data_sid << 16) | (data_sid >> 16));
    _rx_framer->setup(args);
    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &zero_copy_if::get_recv_buff, data_xport, _1
    ), true /*flush*/);
    my_streamer->set_overflow_handler(0, boost::bind(
        &rx_vita_core_3000::handle_overflow, _rx_framer
    ));
    _rx_streamer = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    my_streamer->set_tick_rate(B250_RADIO_CLOCK_RATE);
    my_streamer->set_samp_rate(B250_RADIO_CLOCK_RATE);

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr b250_impl::get_tx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;
/*
    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b250_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

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
    my_streamer->set_vrt_packer(&b250_if_hdr_pack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = nchans;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    _tx_deframer->setup(args);
    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &zero_copy_if::get_send_buff, _data_transport, _1
    ));
    my_streamer->set_xport_chan_sid(0, true, B200_TX_DATA_SID_BASE);
    my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet
    _tx_streamer = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    this->update_streamer_rates(_tick_rate);

    return my_streamer;
    */
}