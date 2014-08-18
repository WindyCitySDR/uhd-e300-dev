//
// Copyright 2013-2014 Ettus Research LLC
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

#include "x300_regs.hpp"
#include "x300_impl.hpp"
#include "validate_subdev_spec.hpp"
#include "../../transport/super_recv_packet_handler.hpp"
#include "../../transport/super_send_packet_handler.hpp"
#include <uhd/transport/nirio_zero_copy.hpp>
#include "async_packet_handler.hpp"
#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/usrp/rfnoc/rx_block_ctrl_base.hpp>
#include <boost/bind.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/log.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

////////////// RFNOC /////////////////
#include <uhd/utils/cast.hpp>
////////////// RFNOC /////////////////

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

/***********************************************************************
 * update streamer rates
 **********************************************************************/
void x300_impl::update_tick_rate(mboard_members_t &mb, const double rate)
{
    BOOST_FOREACH(const size_t &dspno, mb.rx_streamers.keys())
    {
        boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(mb.rx_streamers[dspno].lock());
        if (my_streamer) my_streamer->set_tick_rate(rate);
    }
    BOOST_FOREACH(const size_t &dspno, mb.tx_streamers.keys())
    {
        boost::shared_ptr<sph::send_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::send_packet_streamer>(mb.tx_streamers[dspno].lock());
        if (my_streamer) my_streamer->set_tick_rate(rate);
    }
    //////// RFNOC ///////////////
    BOOST_FOREACH(const std::string &block_id, mb.ce_rx_streamers.keys())
    {
        UHD_MSG(status) << "setting ce rx streamer " << block_id << " rate to " << rate << std::endl;
        boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(mb.ce_rx_streamers[block_id].lock());
        if (my_streamer) my_streamer->set_tick_rate(rate);
        if (my_streamer) my_streamer->set_samp_rate(rate);
    }
    //BOOST_FOREACH(const std::string &block_id, mb.ce_tx_streamers.keys())
    //{
        //UHD_MSG(status) << "setting ce tx streamer " << block_id << std::endl;
        //boost::shared_ptr<sph::send_packet_streamer> my_streamer =
            //boost::dynamic_pointer_cast<sph::send_packet_streamer>(mb.ce_tx_streamers[block_id].lock());
        //if (my_streamer) my_streamer->set_tick_rate(rate);
        //if (my_streamer) my_streamer->set_samp_rate(rate);
    //}
    //////// RFNOC ///////////////
}

void x300_impl::update_rx_samp_rate(mboard_members_t &mb, const size_t dspno, const double rate)
{
    if (not mb.rx_streamers.has_key(dspno)) return;
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::recv_packet_streamer>(mb.rx_streamers[dspno].lock());
    if (not my_streamer) return;
    my_streamer->set_samp_rate(rate);
    const double adj = mb.radio_perifs[dspno].ddc->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void x300_impl::update_tx_samp_rate(mboard_members_t &mb, const size_t dspno, const double rate)
{
    if (not mb.tx_streamers.has_key(dspno)) return;
    boost::shared_ptr<sph::send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::send_packet_streamer>(mb.tx_streamers[dspno].lock());
    if (not my_streamer) return;
    my_streamer->set_samp_rate(rate);
    const double adj = mb.radio_perifs[dspno].duc->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

/***********************************************************************
 * Setup dboard muxing for IQ
 **********************************************************************/
void x300_impl::update_subdev_spec(const std::string &tx_rx, const size_t mb_i, const subdev_spec_t &spec)
{
    UHD_ASSERT_THROW(tx_rx == "tx" or tx_rx == "rx");
    UHD_ASSERT_THROW(mb_i < _mb.size());
    const std::string mb_name = boost::lexical_cast<std::string>(mb_i);
    fs_path mb_root = "/mboards/" + mb_name;

    //sanity checking
    validate_subdev_spec(_tree, spec, tx_rx, mb_name);
    UHD_ASSERT_THROW(spec.size() <= 2);
    if (spec.size() == 1) {
        UHD_ASSERT_THROW(spec[0].db_name == "A" || spec[0].db_name == "B");
    }
    else if (spec.size() == 2) {
        UHD_ASSERT_THROW(
            (spec[0].db_name == "A" && spec[1].db_name == "B") ||
            (spec[0].db_name == "B" && spec[1].db_name == "A")
        );
    }

    std::vector<size_t> chan_to_dsp_map(spec.size(), 0);
    // setup mux for this spec
    for (size_t i = 0; i < spec.size(); i++)
    {
        const int radio_idx = _mb[mb_i].get_radio_index(spec[i].db_name);
        chan_to_dsp_map[i] = radio_idx;

        //extract connection
        const std::string conn = _tree->access<std::string>(mb_root / "dboards" / spec[i].db_name / (tx_rx + "_frontends") / spec[i].sd_name / "connection").get();

        if (tx_rx == "tx") {
            //swap condition
            _mb[mb_i].radio_perifs[radio_idx].tx_fe->set_mux(conn);
        } else {
            //swap condition
            const bool fe_swapped = (conn == "QI" or conn == "Q");
            _mb[mb_i].radio_perifs[radio_idx].ddc->set_mux(conn, fe_swapped);
            //see usrp/io_impl.cpp if multiple DSPs share the frontend:
            _mb[mb_i].radio_perifs[radio_idx].rx_fe->set_mux(fe_swapped);
        }
    }

    _tree->access<std::vector<size_t> >(mb_root / (tx_rx + "_chan_dsp_mapping")).set(chan_to_dsp_map);
}


/***********************************************************************
 * VITA stuff
 **********************************************************************/
#ifdef BOOST_BIG_ENDIAN
    #define BE_MACRO(x) (x)
    #define LE_MACRO(x) uhd::byteswap(x)
#else
    #define BE_MACRO(x) uhd::byteswap(x)
    #define LE_MACRO(x) (x)
#endif
static void x300_if_hdr_unpack_be(
    const boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_unpack_be(packet_buff, if_packet_info);
}

static void x300_if_hdr_pack_be(
    boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_pack_be(packet_buff, if_packet_info);
}

static void x300_if_hdr_unpack_le(
    const boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_unpack_le(packet_buff, if_packet_info);
}

static void x300_if_hdr_pack_le(
    boost::uint32_t *packet_buff,
    vrt::if_packet_info_t &if_packet_info
){
    if_packet_info.link_type = vrt::if_packet_info_t::LINK_TYPE_CHDR;
    return vrt::if_hdr_pack_le(packet_buff, if_packet_info);
}

/***********************************************************************
 * RX flow control handler
 **********************************************************************/
static size_t get_rx_flow_control_window(size_t frame_size, size_t sw_buff_size, const device_addr_t& rx_args)
{
    double fullness_factor = rx_args.cast<double>("recv_buff_fullness", X300_RX_SW_BUFF_FULL_FACTOR);

    if (fullness_factor < 0.01 || fullness_factor > 1) {
        throw uhd::value_error("recv_buff_fullness must be between 0.01 and 1 inclusive (1% to 100%)");
    }

    size_t window_in_pkts = (static_cast<size_t>(sw_buff_size * fullness_factor) / frame_size);
    if (window_in_pkts == 0) {
        throw uhd::value_error("recv_buff_size must be larger than the recv_frame_size.");
    }
    return window_in_pkts;
}

static void handle_rx_flowctrl(const boost::uint32_t sid, zero_copy_if::sptr xport, bool big_endian, boost::shared_ptr<boost::uint32_t> seq32_state, const size_t last_seq)
{
    static size_t fc_pkt_count = 0;
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

    UHD_MSG(status) << "sending flow ctrl packet " << fc_pkt_count++ << ", acking " << str(boost::format("%04d\tseq32==0x%08x") % last_seq % seq32) << std::endl;

    //load packet info
    vrt::if_packet_info_t packet_info;
    packet_info.packet_type = vrt::if_packet_info_t::PACKET_TYPE_FC; // FC!
    packet_info.num_payload_words32 = 2;
    packet_info.num_payload_bytes = packet_info.num_payload_words32*sizeof(boost::uint32_t);
    packet_info.packet_count = seq32;
    packet_info.sob = false;
    packet_info.eob = false;
    packet_info.sid = sid;
    packet_info.has_sid = true;
    packet_info.has_cid = false;
    packet_info.has_tsi = false;
    packet_info.has_tsf = false;
    packet_info.has_tlr = false;

    //load header
    if (big_endian)
        x300_if_hdr_pack_be(pkt, packet_info);
    else
        x300_if_hdr_pack_le(pkt, packet_info);

    //load payload
    pkt[packet_info.num_header_words32+0] = uhd::htonx<boost::uint32_t>(0);
    pkt[packet_info.num_header_words32+1] = uhd::htonx<boost::uint32_t>(seq32);

    // hardcode bits
    pkt[0] = (pkt[0] & 0xFFFFFF00) | 0x00000040;

    //std::cout << "  SID=" << std::hex << sid << " hdr bits=" << packet_info.packet_type << " seq32=" << seq32 << std::endl;
    //std::cout << "num_packet_words32: " << packet_info.num_packet_words32 << std::endl;
    //for (size_t i = 0; i < packet_info.num_packet_words32; i++) {
        //std::cout << str(boost::format("0x%08x") % pkt[i]) << " ";
        //if (i % 2) {
            //std::cout << std::endl;
        //}
    //}

    //send the buffer over the interface
    buff->commit(sizeof(boost::uint32_t)*(packet_info.num_packet_words32));
}


/***********************************************************************
 * TX flow control handler
 **********************************************************************/
struct x300_tx_fc_guts_t
{
    x300_tx_fc_guts_t(void):
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
    boost::shared_ptr<x300_impl::async_md_type> async_queue;
    boost::shared_ptr<x300_impl::async_md_type> old_async_queue;
};

#define X300_ASYNC_EVENT_CODE_FLOW_CTRL 0

static size_t get_tx_flow_control_window(size_t frame_size, const device_addr_t& tx_args)
{
    double hw_buff_size = tx_args.cast<double>("send_buff_size", X300_TX_HW_BUFF_SIZE);
    size_t window_in_pkts = (static_cast<size_t>(hw_buff_size) / frame_size);
    if (window_in_pkts == 0) {
        throw uhd::value_error("send_buff_size must be larger than the send_frame_size.");
    }
    return window_in_pkts;
}

static void handle_tx_async_msgs(boost::shared_ptr<x300_tx_fc_guts_t> guts, zero_copy_if::sptr xport, bool big_endian, x300_clock_ctrl::sptr clock)
{
    managed_recv_buffer::sptr buff = xport->get_recv_buff();
    if (not buff) return;

    //extract packet info
    vrt::if_packet_info_t if_packet_info;
    if_packet_info.num_packet_words32 = buff->size()/sizeof(boost::uint32_t);
    const boost::uint32_t *packet_buff = buff->cast<const boost::uint32_t *>();

    //unpacking can fail
    boost::uint32_t (*endian_conv)(boost::uint32_t) = uhd::ntohx;
    try
    {
        if (big_endian)
        {
            x300_if_hdr_unpack_be(packet_buff, if_packet_info);
            endian_conv = uhd::ntohx;
        }
        else
        {
            x300_if_hdr_unpack_le(packet_buff, if_packet_info);
            endian_conv = uhd::wtohx;
        }
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "Error parsing async message packet: " << ex.what() << std::endl;
        return;
    }

    //fill in the async metadata
    async_metadata_t metadata;
    load_metadata_from_buff(
        endian_conv, metadata, if_packet_info, packet_buff,
        clock->get_master_clock_rate(), guts->stream_channel);

    //The FC response and the burst ack are two indicators that the radio
    //consumed packets. Use them to update the FC metadata
    if (metadata.event_code == X300_ASYNC_EVENT_CODE_FLOW_CTRL or
        metadata.event_code == async_metadata_t::EVENT_CODE_BURST_ACK
    ) {
        const size_t seq = metadata.user_payload[0];
        guts->seq_queue.push_with_pop_on_full(seq);
    }

    //FC responses don't propagate up to the user so filter them here
    if (metadata.event_code != X300_ASYNC_EVENT_CODE_FLOW_CTRL) {
        guts->async_queue->push_with_pop_on_full(metadata);
        metadata.channel = guts->device_channel;
        guts->old_async_queue->push_with_pop_on_full(metadata);
        standard_async_msg_prints(metadata);
    }
}

static managed_send_buffer::sptr get_tx_buff_with_flowctrl(
    task::sptr /*holds ref*/,
    boost::shared_ptr<x300_tx_fc_guts_t> guts,
    zero_copy_if::sptr xport,
    size_t fc_pkt_window,
    const double timeout
){
    while (true)
    {
        const size_t delta = (guts->last_seq_out & 0xfff) - (guts->last_seq_ack & 0xfff);
        if ((delta & 0xfff) <= fc_pkt_window) break;

        const bool ok = guts->seq_queue.pop_with_timed_wait(guts->last_seq_ack, timeout);
        if (not ok) return managed_send_buffer::sptr(); //timeout waiting for flow control
    }

    managed_send_buffer::sptr buff = xport->get_send_buff(timeout);
    if (buff) {
        guts->last_seq_out++; //update seq, this will actually be a send
    }
    return buff;
}

/***********************************************************************
 * Async Data
 **********************************************************************/
bool x300_impl::recv_async_msg(
    async_metadata_t &async_metadata, double timeout
){
    return _async_md->pop_with_timed_wait(async_metadata, timeout);
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
rx_streamer::sptr x300_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    boost::mutex::scoped_lock lock(_transport_setup_mutex);
    stream_args_t args = args_;

    // 0. Sanity check on stream args
    // TODO: This check should be performed by the block controls
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("x300_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    // I. Generate the channel list
    std::vector<uhd::rfnoc::block_id_t> chan_list;
    std::vector<device_addr_t> chan_args;
    if (args.args.has_key("block_id")) { // Override channel settings
        // TODO: Figure out how to put in more than one block ID in the stream args args
        if (not (args.channels.size() == 1 and args.channels[0] == 0)) {
            throw uhd::runtime_error("When specifying the block ID in stream args, channels must start at 0.");
        }
        chan_list.push_back(uhd::rfnoc::block_id_t(args.args["block_id"]));
        chan_args.push_back(device_addr_t());
    } else {
        BOOST_FOREACH(const size_t chan_idx, args.channels) {
            fs_path chan_root = str(boost::format("/channels/%d") % args.channels[chan_idx]);
            if (not _tree->exists(chan_root)) {
                throw uhd::runtime_error("No channel definition for " + chan_root);
            }
            device_addr_t this_chan_args;
            if (not _tree->exists(chan_root / "args")) {
                this_chan_args = _tree->access<device_addr_t>(chan_root / "args").get();
            }
            chan_list.push_back(_tree->access<uhd::rfnoc::block_id_t>(chan_root).get());
            chan_args.push_back(this_chan_args);
        }
    }


    // II. Iterate over all channels
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer;
    for (size_t stream_i = 0; stream_i < chan_list.size(); stream_i++)
    {
        // Get block ID and mb index
        uhd::rfnoc::block_id_t block_id = chan_list[stream_i];
        size_t mb_index = block_id.get_device_no();
        UHD_ASSERT_THROW(mb_index < _mb.size());

        // Access to this channel's mboard and block control
        mboard_members_t &mb = _mb[mb_index];
        uhd::rfnoc::rx_block_ctrl_base::sptr ce_ctrl =
            boost::dynamic_pointer_cast<uhd::rfnoc::rx_block_ctrl_base>(get_block_ctrl(block_id));

        // Setup the DSP transport hints (default to a large recv buff)
        device_addr_t device_addr = mb.recv_args;
        if (not device_addr.has_key("recv_buff_size"))
        {
            if (mb.xport_path != "nirio") {
                //For the ethernet transport, the buffer has to be set before creating
                //the transport because it is independent of the frame size and # frames
                //For nirio, the buffer size is not configurable by the user
                #if defined(UHD_PLATFORM_MACOS) || defined(UHD_PLATFORM_BSD)
                    //limit buffer resize on macos or it will error
                    device_addr["recv_buff_size"] = boost::lexical_cast<std::string>(X300_RX_SW_BUFF_SIZE_ETH_MACOS);
                #elif defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)
                    //set to half-a-second of buffering at max rate
                    device_addr["recv_buff_size"] = boost::lexical_cast<std::string>(X300_RX_SW_BUFF_SIZE_ETH);
                #endif
            }
        }

        //allocate sid and create transport
        boost::uint8_t sid_lower = ce_ctrl->get_address() & 0xFF;
        sid_lower /= 4; // TODO remove this line ASAP
        boost::uint32_t data_sid;
        UHD_LOG << "creating rx stream " << device_addr.to_string() << std::endl;
        both_xports_t xport = this->make_transport(mb_index, sid_lower, 0x00, device_addr, data_sid);
        UHD_LOG << boost::format("data_sid = 0x%08x, actual recv_buff_size = %d\n") % data_sid % xport.recv_buff_size << std::endl;

        // To calculate the max number of samples per packet, we assume the maximum header length
        // to avoid fragmentation should the entire header be used.
        const size_t bpp = xport.recv->get_recv_frame_size() - X300_RX_MAX_HDR_LEN; // bytes per packet
        const size_t bpi = convert::get_bytes_per_item(args.otw_format); // bytes per item
        const size_t spp = std::min(args.args.cast<size_t>("spp", bpp/bpi), bpp/bpi); // samples per packet

        //make the new streamer given the samples per packet
        if (not my_streamer) my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);
        my_streamer->resize(args.channels.size());

        //init some streamer stuff
        std::string conv_endianness;
        if (mb.if_pkt_is_big_endian) {
            my_streamer->set_vrt_unpacker(&x300_if_hdr_unpack_be);
            conv_endianness = "be";
        } else {
            my_streamer->set_vrt_unpacker(&x300_if_hdr_unpack_le);
            conv_endianness = "le";
        }

        //set the converter
        uhd::convert::id_type id;
        id.input_format = args.otw_format + "_item32_" + conv_endianness;
        id.num_inputs = 1;
        id.output_format = args.cpu_format;
        id.num_outputs = 1;
        my_streamer->set_converter(id);

        // Configure the block
        ce_ctrl->setup_rx_streamer(args, data_sid);

        //flow control setup
        const size_t pkt_size = spp * bpi + X300_RX_MAX_HDR_LEN;
        UHD_VAR(pkt_size);
        const size_t fc_window = get_rx_flow_control_window(pkt_size, xport.recv_buff_size, device_addr);
        const size_t fc_handle_window = std::max<size_t>(1, fc_window / X300_RX_FC_REQUEST_FREQ);
        UHD_LOG << "RX Flow Control Window = " << fc_window << ", RX Flow Control Handler Window = " << fc_handle_window << std::endl;
        ce_ctrl->configure_flow_control_out(fc_window);

        //Give the streamer a functor to get the recv_buffer
        //bind requires a zero_copy_if::sptr to add a streamer->xport lifetime dependency
        my_streamer->set_xport_chan_get_buff(
            stream_i,
            boost::bind(&zero_copy_if::get_recv_buff, xport.recv, _1),
            true /*flush*/
        );

        //Give the streamer a functor to handle overflows
        //bind requires a weak_ptr to break the a streamer->streamer circular dependency
        //Using "this" is OK because we know that x300_impl will outlive the streamer
        my_streamer->set_overflow_handler(
            stream_i,
            boost::bind(
                &x300_impl::handle_overflow, this,
                boost::weak_ptr<uhd::rfnoc::rx_block_ctrl_base>(ce_ctrl),
                boost::weak_ptr<uhd::rx_streamer>(my_streamer)
            )
        );

        //Give the streamer a functor to send flow control messages
        //handle_rx_flowctrl is static and has no lifetime issues
        boost::shared_ptr<boost::uint32_t> seq32(new boost::uint32_t(0));
        my_streamer->set_xport_handle_flowctrl(
            stream_i, boost::bind(&handle_rx_flowctrl, data_sid, xport.send, mb.if_pkt_is_big_endian, seq32, _1),
            fc_handle_window,
            true/*init*/
        );

        //Give the streamer a functor issue stream cmd
        //bind requires a shared pointer to add a streamer->framer lifetime dependency
        my_streamer->set_issue_stream_cmd(
            stream_i, boost::bind(&uhd::rfnoc::rx_block_ctrl_base::issue_stream_cmd, ce_ctrl, _1)
        );

        // Tell the streamer which SID is valid for this channel
        my_streamer->set_xport_chan_sid(stream_i, true, data_sid);

        // Store a weak pointer to prevent a streamer->x300_impl->streamer circular dependency
        mb.ce_rx_streamers[ce_ctrl->get_block_id().get()] = boost::weak_ptr<sph::recv_packet_streamer>(my_streamer);

        //sets all tick and samp rates on this streamer
        const fs_path mb_path = "/mboards/"+boost::lexical_cast<std::string>(mb_index);
        _tree->access<double>(mb_path / "tick_rate").update();
        // TODO this is specific to radios and thus should be done by radio_ctrl
        if (ce_ctrl->get_block_id().get_block_name() == "Radio") {
            UHD_MSG(status) << "This is a radio, thus updating sample rate" << std::endl;
            _tree->access<double>(mb_path / "rx_dsps" / boost::lexical_cast<std::string>(ce_ctrl->get_block_id().get_block_count()) / "rate" / "value").update();
        }
    }

    return my_streamer;
}


boost::uint32_t x300_impl::rfnoc_cmd(
                const std::string &dst, const std::string &type,
                boost::uint32_t arg1, boost::uint32_t arg2
) {
    mboard_members_t &mb = _mb[0];
    if (dst == "ce0" or dst == "ce1" or dst == "ce2") {
        size_t ce_index = boost::lexical_cast<size_t>(dst[2]);
        if (type == "poke") {
            UHD_MSG(status) << "Setting register " << std::dec << arg1 << " on CE " << ce_index << " to " << str(boost::format("0x%08x") % arg2) << std::endl;
            _rfnoc_block_ctrl[ce_index]->sr_write(arg1, arg2);
            return 0;
        }
        else if (type == "set_fc") {
            if (arg1) {
                UHD_MSG(status) << "Activating downstream flow control for CE " << ce_index << ". Downstream block buffer size: " << arg1 << " packets." << std::endl;
                //_rfnoc_block_ctrl[ce_index]->poke32(SR_ADDR(0x0000, 1), 0);
                //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                _rfnoc_block_ctrl[ce_index]->configure_flow_control_out(arg1);
            } else {
                UHD_MSG(status) << "Disabling downstream flow control for CE " << ce_index << "." << std::endl;
                _rfnoc_block_ctrl[ce_index]->configure_flow_control_out(0, false);
            }
            if (arg2) {
                UHD_MSG(status) << "Activating upstream flow control for CE " << ce_index << ". Send ACKs every " << arg2 << " packets." << std::endl;
                _rfnoc_block_ctrl[ce_index]->configure_flow_control_in(0 /* cycs off */, arg2);
            } else {
                UHD_MSG(status) << "Disabling upstream flow control for CE " << ce_index << "." << std::endl;
                _rfnoc_block_ctrl[ce_index]->configure_flow_control_in(0 , 0);
            }
            UHD_MSG(status) << "Resetting fc counters." << std::endl;
            _rfnoc_block_ctrl[ce_index]->reset_flow_control();
            return 0;
        }
        throw uhd::value_error("x300_impl::rfnoc_cmd only supports poke and setup_fc on CEs");
    }
    else if (dst == "radio_rx0") {
        if (type == "setup_dsp") {
            UHD_MSG(status) << "Setting radio0, spp=" << arg1 << ", SID=" << str(boost::format("0x%08x") % arg2) << std::endl;
            uhd::stream_args_t args;
            args.otw_format = "sc16";
            args.cpu_format = "sc16";
            radio_perifs_t &perif = mb.radio_perifs[0];
            size_t spp = arg1;
            perif.framer->clear();
            perif.framer->set_nsamps_per_packet(spp); //seems to be a good place to set this
            perif.framer->set_sid(arg2);
            perif.framer->setup(args);
            perif.ddc->setup(args);
            return 0;
        }
        else if (type == "setup_fc") {
            UHD_MSG(status) << "Setting radio0, downstream buffer size =" << arg1 << std::endl;
            radio_perifs_t &perif = mb.radio_perifs[0];
            perif.framer->configure_flow_control(arg1);
            return 0;
        }
        else if (type == "stream_cmd") {
            UHD_MSG(status) << "Stream cmd to radio0 " << arg1 << std::endl;
            radio_perifs_t &perif = mb.radio_perifs[0];
            uhd::stream_cmd_t stream_cmd(
                    arg1 == uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS : 
                    arg1 == uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS : 
                    uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
            );
            UHD_MSG(status) << stream_cmd.stream_mode << std::endl;
            switch (arg1) {
                case 97:
                    UHD_MSG(status) << "STREAM_MODE_START_CONTINUOUS " << arg1<< std::endl;
                    break;

                case 100:
                    UHD_MSG(status) << "STREAM_MODE_STOP_CONTINUOUS " << arg1<< std::endl;
                    break;

                case 111:
                    UHD_MSG(status) << "STREAM_MODE_NUM_SAMPS_AND_DONE " << arg1<< std::endl;
                    stream_cmd.num_samps = arg2;
                    break;
            }
            stream_cmd.stream_now = true;
            stream_cmd.time_spec = uhd::time_spec_t();
            perif.framer->issue_stream_command(stream_cmd);
            return 0;
        }
        throw uhd::value_error("x300_impl::rfnoc_cmd got invalid cmd type");
    }
    else if (dst == "radio_tx0") {
        if (type == "setup_dsp") {
            uhd::stream_args_t args;
            args.otw_format = "sc16";
            args.cpu_format = "sc16";
            radio_perifs_t &perif = mb.radio_perifs[0];
            perif.deframer->clear();
            perif.deframer->setup(args);
            perif.duc->setup(args);
            return 0;
        }
        else if (type == "setup_fc") {
            UHD_MSG(status) << "Setting radio0, ack packet every N==" << arg1 << std::endl;
            radio_perifs_t &perif = mb.radio_perifs[0];
            perif.deframer->configure_flow_control(0/*cycs off*/, arg1);
            return 0;
        }
        throw uhd::value_error("x300_impl::rfnoc_cmd got invalid cmd type");
    }
    throw uhd::value_error("x300_impl::rfnoc_cmd got unknown dst");
    return 0;
}
//////////////// RFNOC ////////////////////////////////

void x300_impl::handle_overflow(
        boost::weak_ptr<uhd::rfnoc::rx_block_ctrl_base> blk_ctrl,
        boost::weak_ptr<uhd::rx_streamer> streamer
) {
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(streamer.lock());
    if (not my_streamer) return; //If the rx_streamer has expired then overflow handling makes no sense.

    uhd::rfnoc::rx_block_ctrl_base::sptr my_blk_ctrl = blk_ctrl.lock();

    if (my_streamer->get_num_channels() == 1)
    {
        my_blk_ctrl->handle_overrun();
        return;
    }

    /////////////////////////////////////////////////////////////
    // MIMO overflow recovery time
    /////////////////////////////////////////////////////////////
    //find out if we were in continuous mode before stopping
    const bool in_continuous_streaming_mode = my_blk_ctrl->in_continuous_streaming_mode();
    //stop streaming
    my_streamer->issue_stream_cmd(stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    //flush transports
    my_streamer->flush_all(0.001);
    //restart streaming
    if (in_continuous_streaming_mode)
    {
        stream_cmd_t stream_cmd(stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = my_blk_ctrl->get_time_now() + time_spec_t(0.01);
        my_streamer->issue_stream_cmd(stream_cmd);
    }
}

/***********************************************************************
 * Transmit streamer
 **********************************************************************/
tx_streamer::sptr x300_impl::get_tx_stream(const uhd::stream_args_t &args_)
{
    boost::mutex::scoped_lock lock(_transport_setup_mutex);
    stream_args_t args = args_;

    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("x300_impl::get_rx_stream only supports otw_format sc16");
    }
    args.otw_format = "sc16";
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    UHD_MSG(status) << "checking dst_addr" << std::endl;
    if (args.args.has_key("dst_addr")) {
        boost::uint16_t dest_addr = uhd::cast::hexstr_cast<boost::uint16_t>(args.args["dst_addr"]);
        UHD_MSG(status) << "dst_addr==" << dest_addr << std::endl;
        UHD_ASSERT_THROW(args.channels.size() == 1);
        return this->get_tx_stream_ce(args, dest_addr);
    }
    UHD_MSG(status) << "nope, not there" << std::endl;

    //shared async queue for all channels in streamer
    boost::shared_ptr<async_md_type> async_md(new async_md_type(1000/*messages deep*/));

    boost::shared_ptr<sph::send_packet_streamer> my_streamer;
    for (size_t stream_i = 0; stream_i < args.channels.size(); stream_i++)
    {
        // Find the mainboard and subdev that corresponds to channel args.channels[stream_i]
        const size_t chan = args.channels[stream_i];
        size_t mb_chan = chan, mb_index;
        for (mb_index = 0; mb_index < _mb.size(); mb_index++) {
            const subdev_spec_t &curr_subdev_spec =
                _tree->access<subdev_spec_t>("/mboards/" + boost::lexical_cast<std::string>(mb_index) / "tx_subdev_spec").get();
            if (mb_chan < curr_subdev_spec.size()) {
                break;
            } else {
                mb_chan -= curr_subdev_spec.size();
            }
        }
        // Find the DSP that corresponds to this mainboard and subdev
        mboard_members_t &mb = _mb[mb_index];
	const size_t radio_index = _tree->access<std::vector<size_t> >("/mboards/" + boost::lexical_cast<std::string>(mb_index) / "tx_chan_dsp_mapping")
                                            .get().at(mb_chan);
        radio_perifs_t &perif = mb.radio_perifs[radio_index];

        //setup the dsp transport hints (TODO)
        device_addr_t device_addr = mb.send_args;

        //allocate sid and create transport
        boost::uint8_t dest = (radio_index == 0)? X300_XB_DST_R0 : X300_XB_DST_R1;
        boost::uint32_t data_sid;
        UHD_LOG << "creating tx stream " << device_addr.to_string() << std::endl;
        both_xports_t xport = this->make_transport(mb_index, dest, X300_RADIO_DEST_PREFIX_TX, device_addr, data_sid);
        UHD_LOG << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

	// To calculate the max number of samples per packet, we assume the maximum header length
	// to avoid fragmentation should the entire header be used.
        const size_t bpp = xport.send->get_send_frame_size() - X300_TX_MAX_HDR_LEN;
        const size_t bpi = convert::get_bytes_per_item(args.otw_format);
        const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

        //make the new streamer given the samples per packet
        if (not my_streamer) my_streamer = boost::make_shared<sph::send_packet_streamer>(spp);
        my_streamer->resize(args.channels.size());

        std::string conv_endianness;
        if (mb.if_pkt_is_big_endian) {
            my_streamer->set_vrt_packer(&x300_if_hdr_pack_be);
            conv_endianness = "be";
        } else {
            my_streamer->set_vrt_packer(&x300_if_hdr_pack_le);
            conv_endianness = "le";
        }

        //set the converter
        uhd::convert::id_type id;
        id.input_format = args.cpu_format;
        id.num_inputs = 1;
        id.output_format = args.otw_format + "_item32_" + conv_endianness;
        id.num_outputs = 1;
        my_streamer->set_converter(id);

        perif.deframer->clear();
        perif.deframer->setup(args);
        perif.duc->setup(args);

        //flow control setup
        size_t fc_window = get_tx_flow_control_window(xport.send->get_send_frame_size(), device_addr);  //In packets
        const size_t fc_handle_window = std::max<size_t>(1, fc_window/X300_TX_FC_RESPONSE_FREQ);

        UHD_LOG << "TX Flow Control Window = " << fc_window << ", TX Flow Control Handler Window = " << fc_handle_window << std::endl;

        perif.deframer->configure_flow_control(0/*cycs off*/, fc_handle_window);
        boost::shared_ptr<x300_tx_fc_guts_t> guts(new x300_tx_fc_guts_t());
        guts->stream_channel = stream_i;
        guts->device_channel = chan;
        guts->async_queue = async_md;
        guts->old_async_queue = _async_md;
        task::sptr task = task::make(boost::bind(&handle_tx_async_msgs, guts, xport.recv, mb.if_pkt_is_big_endian, mb.clock));

        //Give the streamer a functor to get the send buffer
        //get_tx_buff_with_flowctrl is static so bind has no lifetime issues
        //xport.send (sptr) is required to add streamer->data-transport lifetime dependency
        //task (sptr) is required to add  a streamer->async-handler lifetime dependency
        my_streamer->set_xport_chan_get_buff(
            stream_i,
            boost::bind(&get_tx_buff_with_flowctrl, task, guts, xport.send, fc_window, _1)
        );
        //Give the streamer a functor handled received async messages
        my_streamer->set_async_receiver(
            boost::bind(&async_md_type::pop_with_timed_wait, async_md, _1, _2)
        );
        my_streamer->set_xport_chan_sid(stream_i, true, data_sid);
        my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet

        //Store a weak pointer to prevent a streamer->x300_impl->streamer circular dependency
        mb.tx_streamers[radio_index] = boost::weak_ptr<sph::send_packet_streamer>(my_streamer);

        //sets all tick and samp rates on this streamer
        const fs_path mb_path = "/mboards/"+boost::lexical_cast<std::string>(mb_index);
        _tree->access<double>(mb_path / "tick_rate").update();
        _tree->access<double>(mb_path / "tx_dsps" / boost::lexical_cast<std::string>(radio_index) / "rate" / "value").update();
    }

    return my_streamer;
}

tx_streamer::sptr x300_impl::get_tx_stream_ce(const uhd::stream_args_t &args_, boost::uint16_t dst_addr)
{
    stream_args_t args = args_;
    UHD_MSG(status) << "get_tx_stream_ce()" << std::endl;

    UHD_ASSERT_THROW(args.channels.size() == 1);
    //setup defaults for unspecified values
    if (not args.otw_format.empty() and args.otw_format != "sc16")
    {
        throw uhd::value_error("x300_impl::get_rx_stream_ce only supports otw_format sc16");
    }
    if (not args.cpu_format.empty() and args.cpu_format != "sc16")
    {
        throw uhd::value_error("x300_impl::get_rx_stream_ce only supports cpu_format sc16");
    }
    args.otw_format = "sc16";
    args.cpu_format = "sc16";

    //shared async queue for all channels in streamer
    boost::shared_ptr<async_md_type> async_md(new async_md_type(1000/*messages deep*/));

    boost::shared_ptr<sph::send_packet_streamer> my_streamer;

    // TODO: Find the right mainboard depending on the address
    size_t mb_index = 0;
    size_t stream_i = 0;
    // Find the DSP that corresponds to this mainboard and subdev
    UHD_ASSERT_THROW(mb_index < _mb.size());

    // Find the right CE TODO this is ugly
    size_t ce_index = dst_addr;
    boost::uint8_t sid_lower;
    switch (dst_addr) { // Right now this is ce index (should this be sid? TODO)
        case 0:
            sid_lower = X300_XB_DST_CE0;
            break;
        case 1:
            sid_lower = X300_XB_DST_CE1;
            break;
        case 2:
            sid_lower = X300_XB_DST_CE2;
            break;
    }
    UHD_ASSERT_THROW(ce_index <= 2);
    UHD_MSG(status) << "ce_index==" << ce_index << std::endl;
    UHD_MSG(status) << "mb_index==" << mb_index << std::endl;

    // Setup
    mboard_members_t &mb = _mb[mb_index];
    uhd::rfnoc::block_ctrl_base::sptr ce_ctrl = _rfnoc_block_ctrl[ce_index];

    //setup the dsp transport hints (TODO)
    device_addr_t device_addr = mb.send_args;

    //allocate sid and create transport
    boost::uint32_t data_sid;
    UHD_LOG << "creating tx stream " << device_addr.to_string() << std::endl;
    both_xports_t xport = this->make_transport(mb_index, sid_lower, 0x00, device_addr, data_sid);
    UHD_LOG << boost::format("data_sid = 0x%08x\n") % data_sid << std::endl;

    // To calculate the max number of samples per packet, we assume the maximum header length
    // to avoid fragmentation should the entire header be used.
    size_t bpp = xport.send->get_send_frame_size() - X300_TX_MAX_HDR_LEN;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));
    bpp = spp * bpi;
    UHD_MSG(status) << "spp==" << spp << std::endl;
    UHD_MSG(status) << "bpp==" << bpp << std::endl;

    //make the new streamer given the samples per packet
    if (not my_streamer) my_streamer = boost::make_shared<sph::send_packet_streamer>(spp);
    my_streamer->resize(args.channels.size());

    std::string conv_endianness;
    if (mb.if_pkt_is_big_endian) {
        my_streamer->set_vrt_packer(&x300_if_hdr_pack_be);
        conv_endianness = "be";
    } else {
        my_streamer->set_vrt_packer(&x300_if_hdr_pack_le);
        conv_endianness = "le";
    }

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.cpu_format;
    id.num_inputs = 1;
    id.output_format = args.otw_format + "_item32_" + conv_endianness;
    id.num_outputs = 1;
    UHD_MSG(status) << "converter id " << id.to_pp_string() << std::endl;
    my_streamer->set_converter(id);

    //flow control setup
    UHD_VAR(xport.send->get_send_frame_size());
    // THIS IS A BUG: get_send_frame_size() will not report the actual packet size if e.g. if spp is set
    //size_t fc_window = get_tx_flow_control_window(xport.send->get_send_frame_size(), device_addr);  //In packets
    size_t fc_window = 8000 / (bpp+8);
    //const size_t fc_handle_window = std::max<size_t>(1, fc_window/X300_TX_FC_RESPONSE_FREQ);
    const size_t fc_handle_window = 2;
    ce_ctrl->configure_flow_control_out(0 /* cycs off */, fc_handle_window);

    UHD_LOG << "TX Flow Control Window = " << fc_window << ", TX Flow Control Handler Window = " << fc_handle_window << std::endl;
    UHD_MSG(status) << "TX Flow Control Window = " << fc_window << ", TX Flow Control Handler Window = " << fc_handle_window << std::endl;

    boost::shared_ptr<x300_tx_fc_guts_t> guts(new x300_tx_fc_guts_t());
    guts->stream_channel = 0;
    guts->device_channel = 0;
    guts->async_queue = async_md;
    guts->old_async_queue = _async_md;
    task::sptr task = task::make(boost::bind(&handle_tx_async_msgs, guts, xport.recv, mb.if_pkt_is_big_endian, mb.clock));

    //Give the streamer a functor to get the send buffer
    //get_tx_buff_with_flowctrl is static so bind has no lifetime issues
    //xport.send (sptr) is required to add streamer->data-transport lifetime dependency
    //task (sptr) is required to add  a streamer->async-handler lifetime dependency
    my_streamer->set_xport_chan_get_buff(
        stream_i,
        boost::bind(&get_tx_buff_with_flowctrl, task, guts, xport.send, fc_window, _1)
    );
    //Give the streamer a functor handled received async messages
    my_streamer->set_async_receiver(
        boost::bind(&async_md_type::pop_with_timed_wait, async_md, _1, _2)
    );
    my_streamer->set_xport_chan_sid(stream_i, true, data_sid);
    my_streamer->set_enable_trailer(false); //TODO not implemented trailer support yet

    //Store a weak pointer to prevent a streamer->x300_impl->streamer circular dependency
    mb.ce_tx_streamers[ce_index] = boost::weak_ptr<sph::send_packet_streamer>(my_streamer);

    return my_streamer;
}
// vim: sw=4 expandtab:
