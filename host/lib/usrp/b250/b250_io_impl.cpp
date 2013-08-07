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
#include <uhd/utils/log.hpp>
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
    if (not my_streamer) return;
    my_streamer->set_samp_rate(rate);
    const double adj = _radio_perifs[dspno].ddc->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void b250_impl::update_tx_samp_rate(const size_t dspno, const double rate)
{
    if (not _tx_streamers.has_key(dspno)) return;
    boost::shared_ptr<sph::send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[dspno].lock());
    if (not my_streamer) return;
    my_streamer->set_samp_rate(rate);
    const double adj = _radio_perifs[dspno].duc->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

/***********************************************************************
 * Setup dboard muxing for IQ
 **********************************************************************/
void b250_impl::update_rx_subdev_spec(const subdev_spec_t &spec)
{
    fs_path root = "/mboards/0/dboards";

    //sanity checking
    validate_subdev_spec(_tree, spec, "rx");
    UHD_ASSERT_THROW(spec.size() <= 2);
    if (spec.size() > 0) UHD_ASSERT_THROW(spec[0].db_name == "A");
    if (spec.size() > 1) UHD_ASSERT_THROW(spec[1].db_name == "B");

    //setup mux for this spec
    for (size_t i = 0; i < 2; i++)
    {
        //extract db name
        const std::string db_name = (i == 0)? "A" : "B";
        if (i < spec.size()) UHD_ASSERT_THROW(spec[i].db_name == db_name);

        //extract fe name
        std::string fe_name;
        if (i < spec.size()) fe_name = spec[i].sd_name;
        else fe_name = _tree->list(root / db_name / "rx_frontends").front();

        //extract connection
        const std::string conn = _tree->access<std::string>(root / db_name / "rx_frontends" / fe_name / "connection").get();

        //swap condition
        _radio_perifs[i].ddc->set_mux(conn, false);
    }

    _rx_fe_map = spec;
}

void b250_impl::update_tx_subdev_spec(const subdev_spec_t &spec)
{
    fs_path root = "/mboards/0/dboards";

    //sanity checking
    validate_subdev_spec(_tree, spec, "tx");
    UHD_ASSERT_THROW(spec.size() <= 2);
    if (spec.size() > 0) UHD_ASSERT_THROW(spec[0].db_name == "A");
    if (spec.size() > 1) UHD_ASSERT_THROW(spec[1].db_name == "B");

    //set the mux for this spec
    for (size_t i = 0; i < 2; i++)
    {
        //extract db name
        const std::string db_name = (i == 0)? "A" : "B";
        if (i < spec.size()) UHD_ASSERT_THROW(spec[i].db_name == db_name);

        //extract fe name
        std::string fe_name;
        if (i < spec.size()) fe_name = spec[i].sd_name;
        else fe_name = _tree->list(root / db_name / "tx_frontends").front();

        //extract connection
        const std::string conn = _tree->access<std::string>(root / db_name / "tx_frontends" / fe_name / "connection").get();

        //swap condition
        const bool swap = (conn[0] == 'Q');
        _radio_perifs[i].dac->set_iq_swap(swap);
    }

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
static void handle_rx_flowctrl(const boost::uint32_t sid, zero_copy_if::sptr xport, boost::shared_ptr<boost::uint32_t> seq32_state, const size_t last_seq)
{
    managed_send_buffer::sptr buff = xport->get_send_buff(0.0);
    if (not buff)
    {
        throw uhd::runtime_error("handle_rx_flowctrl timed out getting a send buffer");
    }
    boost::uint32_t *pkt = buff->cast<boost::uint32_t *>();

    //recover seq32
    boost::uint32_t &seq32 = *seq32_state;
    const size_t seq12 = seq32 & 0xfff;
    if (last_seq < seq12) seq32 += (1 << 12);
    seq32 &= ~0xfff;
    seq32 |= last_seq;

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
    pkt[packet_info.num_header_words32+0] = uhd::htonx<boost::uint32_t>(0);
    pkt[packet_info.num_header_words32+1] = uhd::htonx<boost::uint32_t>(seq32);

    //send the buffer over the interface
    buff->commit(sizeof(boost::uint32_t)*(packet_info.num_packet_words32));
}


/***********************************************************************
 * TX flow control handler
 **********************************************************************/
struct b250_tx_fc_guts_t
{
    b250_tx_fc_guts_t(void):
        stream_channel(0),
        device_channel(0),
        last_seq_out(0),
        last_seq_ack(0),
        seq_queue(1){}
    size_t stream_channel;
    size_t device_channel;
    size_t last_seq_out;
    size_t last_seq_ack;
    bounded_buffer<size_t> seq_queue;
    boost::shared_ptr<b250_impl::async_md_type> async_queue;
    boost::shared_ptr<b250_impl::async_md_type> old_async_queue;
};

static void handle_tx_async_msgs(boost::shared_ptr<b250_tx_fc_guts_t> guts, zero_copy_if::sptr xport, b250_clock_ctrl::sptr clock)
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
    load_metadata_from_buff(
        uhd::ntohx<boost::uint32_t>, metadata, if_packet_info, packet_buff,
        clock->get_master_clock_rate(), guts->stream_channel);
    guts->async_queue->push_with_pop_on_full(metadata);
    metadata.channel = guts->device_channel;
    guts->old_async_queue->push_with_pop_on_full(metadata);
    standard_async_msg_prints(metadata);
}

static managed_send_buffer::sptr get_tx_buff_with_flowctrl(
    task::sptr /*holds ref*/,
    boost::shared_ptr<b250_tx_fc_guts_t> guts,
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
    return _async_md->pop_with_timed_wait(async_metadata, timeout);
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

    boost::shared_ptr<sph::recv_packet_streamer> my_streamer;
    for (size_t stream_i = 0; stream_i < args.channels.size(); stream_i++)
    {
        const size_t chan = args.channels[stream_i];
        radio_perifs_t &perif = _radio_perifs[chan];

        //setup the dsp transport hints (default to a large recv buff)
        device_addr_t device_addr = _recv_args;
        if (not device_addr.has_key("recv_buff_size"))
        {
            #if defined(UHD_PLATFORM_MACOS) || defined(UHD_PLATFORM_BSD)
                //limit buffer resize on macos or it will error
                device_addr["recv_buff_size"] = "1e6";
            #elif defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)
                //set to half-a-second of buffering at max rate
                device_addr["recv_buff_size"] = "50e6";
            #endif
        }

        //allocate sid and create transport
        sid_config_t data_config;
        data_config.router_addr_there = B250_DEVICE_THERE;
        data_config.dst_prefix = B250_RADIO_DEST_PREFIX_RX;
        data_config.router_dst_there = (chan == 0)? B250_XB_DST_R0 : B250_XB_DST_R1;
        data_config.router_dst_here = _router_dst_here;
        const boost::uint32_t data_sid = this->allocate_sid(data_config);
        UHD_LOG << "creating rx stream " << device_addr.to_string() << std::endl;
        zero_copy_if::sptr data_xport = this->make_transport(_addr, data_sid, device_addr);
        UHD_LOG << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

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
        if (not my_streamer) my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);
        my_streamer->resize(args.channels.size());

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
        perif.ddc->setup(args);

        //flow control setup
        const size_t max_buffering = size_t(device_addr.cast<double>("recv_buff_size", 1e6));
        const size_t fc_window = max_buffering/data_xport->get_recv_frame_size();
        perif.framer->configure_flow_control(fc_window);

        boost::shared_ptr<boost::uint32_t> seq32(new boost::uint32_t(0));
        my_streamer->set_xport_chan_get_buff(stream_i, boost::bind(
            &zero_copy_if::get_recv_buff, data_xport, _1
        ), true /*flush*/);
        my_streamer->set_overflow_handler(stream_i, boost::bind(
            &rx_vita_core_3000::handle_overflow, perif.framer
        ));
        my_streamer->set_xport_handle_flowctrl(stream_i, boost::bind(
            &handle_rx_flowctrl, data_sid, data_xport, seq32, _1
        ), fc_window, true/*init*/);
        my_streamer->set_issue_stream_cmd(stream_i, boost::bind(
            &rx_vita_core_3000::issue_stream_command, perif.framer, _1
        ));
        _rx_streamers[chan] = my_streamer; //store weak pointer

        //sets all tick and samp rates on this streamer
        _tree->access<double>("/mboards/0/tick_rate").update();
        _tree->access<double>(str(boost::format("/mboards/0/rx_dsps/%u/rate/value") % chan)).update();
    }

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

    //shared async queue for all channels in streamer
    boost::shared_ptr<async_md_type> async_md(new async_md_type(1000/*messages deep*/));

    boost::shared_ptr<sph::send_packet_streamer> my_streamer;
    for (size_t stream_i = 0; stream_i < args.channels.size(); stream_i++)
    {
        const size_t chan = args.channels[stream_i];
        radio_perifs_t &perif = _radio_perifs[chan];

        //allocate sid and create transport
        sid_config_t data_config;
        data_config.router_addr_there = B250_DEVICE_THERE;
        data_config.dst_prefix = B250_RADIO_DEST_PREFIX_TX;
        data_config.router_dst_there = (chan == 0)? B250_XB_DST_R0 : B250_XB_DST_R1;
        data_config.router_dst_here = _router_dst_here;
        const boost::uint32_t data_sid = this->allocate_sid(data_config);
        UHD_LOG << "creating tx stream " << _send_args.to_string() << std::endl;
        zero_copy_if::sptr data_xport = this->make_transport(_addr, data_sid, _send_args);
        UHD_LOG << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

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
        if (not my_streamer) my_streamer = boost::make_shared<sph::send_packet_streamer>(spp);
        my_streamer->resize(args.channels.size());

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
        perif.duc->setup(args);

        //flow control setup
        perif.deframer->configure_flow_control(0/*cycs off*/, B250_TX_FC_PKT_WINDOW/8/*pkts*/);
        boost::shared_ptr<b250_tx_fc_guts_t> guts(new b250_tx_fc_guts_t());
        guts->stream_channel = stream_i;
        guts->device_channel = chan;
        guts->async_queue = async_md;
        guts->old_async_queue = _async_md;
        task::sptr task = task::make(boost::bind(&handle_tx_async_msgs, guts, data_xport, _clock));

        my_streamer->set_xport_chan_get_buff(stream_i, boost::bind(
            &get_tx_buff_with_flowctrl, task, guts, data_xport, _1
        ));
        my_streamer->set_async_receiver(boost::bind(
            &async_md_type::pop_with_timed_wait, async_md, _1, _2
        ));
        my_streamer->set_xport_chan_sid(stream_i, true, data_sid);
        my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet
        _tx_streamers[chan] = my_streamer; //store weak pointer

        //sets all tick and samp rates on this streamer
        _tree->access<double>("/mboards/0/tick_rate").update();
        _tree->access<double>(str(boost::format("/mboards/0/tx_dsps/%u/rate/value") % chan)).update();
    }

    return my_streamer;
}
