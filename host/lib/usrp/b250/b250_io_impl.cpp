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
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/bind.hpp>
#include <uhd/utils/tasks.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

/***********************************************************************
 * update streamer rates
 **********************************************************************/
void b250_impl::update_tick_rate(const double rate)
{
    BOOST_FOREACH(const size_t &dspno, _rx_streamers.keys())
    {
        boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(_rx_streamers[dspno].lock());
        if (my_streamer) my_streamer->set_tick_rate(rate);
    }
    BOOST_FOREACH(const size_t &dspno, _tx_streamers.keys())
    {
        boost::shared_ptr<sph::send_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[dspno].lock());
        if (my_streamer) my_streamer->set_tick_rate(rate);
    }
}

void b250_impl::update_rx_samp_rate(const size_t dspno, const double rate)
{
    if (not _rx_streamers.has_key(dspno)) return;
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::recv_packet_streamer>(_rx_streamers[dspno].lock());
    if (my_streamer) my_streamer->set_samp_rate(rate);
}

void b250_impl::update_tx_samp_rate(const size_t dspno, const double rate)
{
    if (not _tx_streamers.has_key(dspno)) return;
    boost::shared_ptr<sph::send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[dspno].lock());
    if (my_streamer) my_streamer->set_samp_rate(rate);
}

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
        _radio_perifs[(spec[i].db_name == "A")? 0: 1].ddc->set_mux(conn, fe_swapped); //TODO which frontend?
    }
    //TODO _rx_fe->set_mux(fe_swapped);

    /*
    //compute the new occupancy and resize
    _mbc[which_mb].rx_chan_occ = spec.size();
    size_t nchan = 0;
    BOOST_FOREACH(const std::string &mb, _mbc.keys()) nchan += _mbc[mb].rx_chan_occ;
    */
    _rx_fe_map = spec;
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
    _tx_fe_map = spec;
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
 * RX flow control handler
 **********************************************************************/
static void handle_rx_flowctrl(const boost::uint32_t sid, zero_copy_if::sptr xport, const size_t last_seq)
{
    managed_send_buffer::sptr buff = xport->get_send_buff(0.0);
    if (not buff)
    {
        throw uhd::runtime_error("handle_rx_flowctrl timed out getting a send buffer");
    }
    boost::uint32_t *pkt = buff->cast<boost::uint32_t *>();

    //load packet info
    vrt::if_packet_info_t packet_info;
    packet_info.packet_type = vrt::if_packet_info_t::PACKET_TYPE_CONTEXT;
    packet_info.num_payload_words32 = 2;
    packet_info.num_payload_bytes = packet_info.num_payload_words32*sizeof(boost::uint32_t);
    packet_info.packet_count = 0;
    packet_info.sob = false;
    packet_info.eob = false;
    packet_info.sid = sid;
    packet_info.has_sid = true;
    packet_info.has_cid = false;
    packet_info.has_tsi = false;
    packet_info.has_tsf = false;
    packet_info.has_tlr = false;

    //load header
    b250_if_hdr_pack_be(pkt, packet_info);

    //load payload
    pkt[packet_info.num_header_words32+0] = uhd::htonx<boost::uint32_t>(B250_ENABLE_RX_FC? 0 : ~0);
    pkt[packet_info.num_header_words32+1] = uhd::htonx<boost::uint32_t>(last_seq + B250_RX_FC_PKT_WINDOW);

    //send the buffer over the interface
    buff->commit(sizeof(boost::uint32_t)*(packet_info.num_packet_words32));
}


/***********************************************************************
 * TX flow control handler
 **********************************************************************/
struct tx_fc_guts_t
{
    tx_fc_guts_t(void):
        last_seq_out(0),
        last_seq_ack(0),
        seq_queue(1),
        async_queue(1000){}
    size_t last_seq_out;
    size_t last_seq_ack;
    bounded_buffer<size_t> seq_queue;
    bounded_buffer<async_metadata_t> async_queue;
};

static void handle_tx_async_msgs(boost::shared_ptr<tx_fc_guts_t> guts, zero_copy_if::sptr xport)
{
    managed_recv_buffer::sptr buff = xport->get_recv_buff();
    if (not buff) return;

    //extract packet info
    vrt::if_packet_info_t if_packet_info;
    if_packet_info.num_packet_words32 = buff->size()/sizeof(boost::uint32_t);
    const boost::uint32_t *packet_buff = buff->cast<const boost::uint32_t *>();

    //unpacking can fail
    try
    {
        b250_if_hdr_unpack_be(packet_buff, if_packet_info);
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "Error parsing async message packet: " << ex.what() << std::endl;
        return;
    }

    //catch the flow control packets and react
    if (uhd::ntohx(packet_buff[if_packet_info.num_header_words32+0]) == 0)
    {
        const size_t seq = uhd::ntohx(packet_buff[if_packet_info.num_header_words32+1]);
        guts->seq_queue.push_with_haste(seq);
        return;
    }

    //fill in the async metadata
    async_metadata_t metadata;
    load_metadata_from_buff(uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, packet_buff, B250_RADIO_CLOCK_RATE/*FIXME set from rate update*/);
    guts->async_queue.push_with_pop_on_full(metadata);
    standard_async_msg_prints(metadata);
}

static managed_send_buffer::sptr get_tx_buff_with_flowctrl(
    task::sptr /*holds ref*/,
    boost::shared_ptr<tx_fc_guts_t> guts,
    zero_copy_if::sptr xport,
    const double timeout
){
    while (true)
    {
        const size_t delta = (guts->last_seq_out & 0xfff) - (guts->last_seq_ack & 0xfff);
        if ((delta & 0xfff) <= B250_TX_FC_PKT_WINDOW) break;

        const bool ok = guts->seq_queue.pop_with_timed_wait(guts->last_seq_ack, timeout);
        if (not ok) return managed_send_buffer::sptr(); //timeout waiting for flow control
    }

    managed_send_buffer::sptr buff = xport->get_send_buff(timeout);
    if (buff) guts->last_seq_out++; //update seq, this will actually be a send
    return buff;
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool b250_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    boost::shared_ptr<sph::send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[0].lock());
    if (my_streamer) return my_streamer->recv_async_msg(async_metadata, timeout);
    return false;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr b250_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    boost::mutex::scoped_lock lock(_transport_setup_mutex);
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b250_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
    const std::string db_name = _rx_fe_map[args.channels[0]].db_name;
    const size_t i = (db_name == "A")? 0 : 1;
    radio_perifs_t &perif = _radio_perifs[i];

    //allocate sid and create transport
    sid_config_t data_config;
    data_config.router_addr_there = B250_DEVICE_THERE;
    data_config.dst_prefix = B250_RADIO_DEST_PREFIX_RX;
    data_config.router_dst_there = (i == 0)? B250_XB_DST_R0 : B250_XB_DST_R1;
    data_config.router_dst_here = B250_XB_DST_E0;
    const boost::uint32_t data_sid = this->allocate_sid(data_config);
    zero_copy_if::sptr data_xport = this->make_transport(_addr, data_sid);
    UHD_MSG(status) << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::num_vrl_words32*sizeof(boost::uint32_t)
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = data_xport->get_recv_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

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

    perif.framer->clear();
    perif.framer->set_nsamps_per_packet(spp); //seems to be a good place to set this
    perif.framer->set_sid((data_sid << 16) | (data_sid >> 16));
    perif.framer->setup(args);
    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &zero_copy_if::get_recv_buff, data_xport, _1
    ), true /*flush*/);
    my_streamer->set_overflow_handler(0, boost::bind(
        &rx_vita_core_3000::handle_overflow, perif.framer
    ));
    my_streamer->set_xport_handle_flowctrl(0, boost::bind(
        &handle_rx_flowctrl, data_sid, data_xport, _1
    ), B250_RX_FC_PKT_WINDOW/8, true/*init*/);
    my_streamer->set_issue_stream_cmd(0, boost::bind(
        &rx_vita_core_3000::issue_stream_command, perif.framer, _1
    ));
    _rx_streamers[i] = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    _tree->access<double>("/mboards/0/tick_rate").update();
    _tree->access<double>(str(boost::format("/mboards/0/rx_dsps/%u/rate/value") % i)).update();

    return my_streamer;
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr b250_impl::get_tx_stream(const uhd::stream_args_t &args_)
{
    boost::mutex::scoped_lock lock(_transport_setup_mutex);
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("b250_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;
    const std::string db_name = _tx_fe_map[args.channels[0]].db_name;
    const size_t i = (db_name == "A")? 0 : 1;
    UHD_VAR(i);
    radio_perifs_t &perif = _radio_perifs[i];

    //allocate sid and create transport
    sid_config_t data_config;
    data_config.router_addr_there = B250_DEVICE_THERE;
    data_config.dst_prefix = B250_RADIO_DEST_PREFIX_TX;
    data_config.router_dst_there = (i == 0)? B250_XB_DST_R0 : B250_XB_DST_R1;
    data_config.router_dst_here = B250_XB_DST_E0;
    const boost::uint32_t data_sid = this->allocate_sid(data_config);
    zero_copy_if::sptr data_xport = this->make_transport(_addr, data_sid);
    UHD_MSG(status) << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::num_vrl_words32*sizeof(boost::uint32_t)
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        //+ sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = data_xport->get_send_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::send_packet_streamer> my_streamer = boost::make_shared<sph::send_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->set_vrt_packer(&b250_if_hdr_pack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_be";
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    perif.deframer->clear();
    perif.deframer->setup(args);

    //flow control setup
    perif.deframer->configure_flow_control(0/*cycs off*/, B250_TX_FC_PKT_WINDOW/8/*pkts*/);
    boost::shared_ptr<tx_fc_guts_t> guts(new tx_fc_guts_t());
    task::sptr task = task::make(boost::bind(&handle_tx_async_msgs, guts, data_xport));

    my_streamer->set_xport_chan_get_buff(0, boost::bind(
        &get_tx_buff_with_flowctrl, task, guts, data_xport, _1
    ));
    my_streamer->set_async_receiver(boost::bind(
        &bounded_buffer<async_metadata_t>::pop_with_timed_wait, &(guts->async_queue), _1, _2
    ));
    my_streamer->set_xport_chan_sid(0, true, data_sid);
    my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet
    _tx_streamers[i] = my_streamer; //store weak pointer

    //sets all tick and samp rates on this streamer
    _tree->access<double>("/mboards/0/tick_rate").update();
    _tree->access<double>(str(boost::format("/mboards/0/tx_dsps/%u/rate/value") % i)).update();

    return my_streamer;
}
