//
// Copyright 2014 Ettus Research LLC
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

#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <uhd/convert.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/ranges.hpp>
#include "radio_ctrl.hpp"
#include "../../transport/super_recv_packet_handler.hpp"

using namespace uhd;
using namespace uhd::rfnoc;

class radio_ctrl_impl : public radio_ctrl
{
public:
    UHD_RFNOC_BLOCK_CONSTRUCTOR(radio_ctrl)
    {
        //// RX Streamer args
        // CPU format doesn't really matter, just init it to something
        _rx_stream_args = uhd::stream_args_t("fc32", "sc16");
        _rx_bpi = uhd::convert::get_bytes_per_item(_rx_stream_args.otw_format);
        // Default: 1 Channel
        _rx_stream_args.channels = std::vector<size_t>(1, 0);
        // SPP is stored for calls to set_bytes_per_output_packet etc.
        _rx_spp = get_bytes_per_output_packet(0) / _rx_bpi;

        // TODO: Once the radio looks like a NoC-Block, remove this!
        _tree->remove(_root_path / "input_buffer_size");
        _tree->create<size_t>(_root_path / "input_buffer_size/0").set(0x90000/2);
    }

    void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd)
    {
        UHD_MSG(status) << "radio_ctrl::issue_stream_cmd()" << std::endl;
        _perifs.framer->issue_stream_command(stream_cmd);
    }

    void configure_flow_control_in(
            size_t cycles,
            size_t packets,
            size_t block_port
    ) {
        UHD_MSG(status) << "radio_ctrl::configure_flow_control_in()" << cycles << " " << packets << std::endl;
        UHD_ASSERT_THROW(block_port == 0);
        _perifs.deframer->configure_flow_control(cycles, packets);
    }

    void configure_flow_control_out(
                size_t buf_size_pkts,
                size_t,
                const uhd::sid_t &
    ) {
        UHD_MSG(status) << "radio_ctrl::configure_flow_control_out() " << buf_size_pkts << std::endl;
        _perifs.framer->configure_flow_control(buf_size_pkts);
    }

    void reset_flow_control()
    {
        // TODO can't call this here... but where?
        //_perifs.deframer->clear();
    }

    bool set_bytes_per_output_packet(
            size_t bpp,
            size_t out_block_port
    ) {
        UHD_MSG(status) << "radio_ctrl::set_bytes_per_output_packet()" << std::endl;
        UHD_ASSERT_THROW(out_block_port == 0);
        if (bpp % _rx_bpi) {
            return false;
        }
        _tree->access<size_t>(_root_path / "bytes_per_packet/default").set(bpp);

        _rx_spp = bpp / _rx_bpi;
        UHD_MSG(status) << "radio_ctrl::set_bytes_per_output_packet(): Setting spp to " << _rx_spp << std::endl;
        _perifs.framer->set_nsamps_per_packet(_rx_spp);
        return true;
    }

    size_t get_bytes_per_output_packet(UHD_UNUSED(size_t out_block_port))
    {
        return _tree->access<size_t>(_root_path / "bytes_per_packet/default").get();
    }

    void set_destination(
            boost::uint32_t next_address,
            size_t out_block_port
    ) {
        UHD_MSG(status) << "radio_ctrl::set_destination()" << std::endl;
        UHD_ASSERT_THROW(out_block_port == 0);
        uhd::sid_t sid(next_address);
        if (sid.get_src_address() == 0) {
            sid.set_src_address(get_address());
        }
        UHD_MSG(status) << "radio: setting sid to " << sid << std::endl;

        _perifs.framer->set_sid(sid.get());
    }

    void handle_overrun(boost::weak_ptr<uhd::rx_streamer> streamer)
    {
        UHD_MSG(status) << "radio_ctrl::handle_overrun()" << std::endl;
        boost::shared_ptr<transport::sph::recv_packet_streamer> my_streamer =
                boost::dynamic_pointer_cast<transport::sph::recv_packet_streamer>(streamer.lock());
        if (not my_streamer) return; //If the rx_streamer has expired then overflow handling makes no sense.

        if (my_streamer->get_num_channels() == 1) {
            _perifs.framer->handle_overflow();
            return;
        }

        /////////////////////////////////////////////////////////////
        // MIMO overflow recovery time
        /////////////////////////////////////////////////////////////
        //find out if we were in continuous mode before stopping
        const bool in_continuous_streaming_mode = _perifs.framer->in_continuous_streaming_mode();
        //stop streaming
        my_streamer->issue_stream_cmd(stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        //flush transports
        my_streamer->flush_all(0.001);
        //restart streaming
        if (in_continuous_streaming_mode)
        {
            stream_cmd_t stream_cmd(stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
            stream_cmd.stream_now = false;
            stream_cmd.time_spec = _perifs.time64->get_time_now() + time_spec_t(0.01);
            my_streamer->issue_stream_cmd(stream_cmd);
        }
    }

    void set_perifs(
        time_core_3000::sptr    time64,
        rx_vita_core_3000::sptr framer,
        rx_dsp_core_3000::sptr  ddc,
        tx_vita_core_3000::sptr deframer,
        tx_dsp_core_3000::sptr  duc
    ) {
        _perifs.time64 = time64;
        _perifs.framer = framer;
        _perifs.ddc = ddc;
        _perifs.deframer = deframer;
        _perifs.duc = duc;
    }

protected:
    void _init_rx(uhd::stream_args_t &args)
    {
        UHD_MSG(status) << "radio_ctrl::init_rx()" << std::endl;
        // TODO move this to my own clear()
        //_perifs.framer->clear();
        if (args.otw_format != "sc16") {
            throw uhd::value_error("this radio only supports otw_format sc16");
        }
        // Set spp, if applicable
        if (not args.args.has_key("spp")) {
            args.args["spp"] = str(boost::format("%d") % _rx_spp);
        } else {
            _rx_spp = boost::lexical_cast<size_t>(args.args["spp"]);
        }
        if (not set_bytes_per_output_packet(_rx_spp * _rx_bpi, 0)) {
            throw uhd::value_error("radio_ctrl::init_rx(): Invalid spp value.");
        }
        _perifs.framer->setup(args);
        _perifs.ddc->setup(args);
    }

    void _init_tx(uhd::stream_args_t &args)
    {
        UHD_MSG(status) << "radio_ctrl::init_tx()" << std::endl;

        _perifs.deframer->setup(args);
        _perifs.duc->setup(args);
        return;
    }

    bool _is_final_rx_block()
    {
        // Radio is end of line
        return true;
    }

    bool _is_final_tx_block()
    {
        // Radio is end of line
        return true;
    }

private:

    //! Stores pointers to all streaming-related radio cores
    struct radio_v_perifs_t
    {
        time_core_3000::sptr    time64;
        rx_vita_core_3000::sptr framer;
        rx_dsp_core_3000::sptr  ddc;
        tx_vita_core_3000::sptr deframer;
        tx_dsp_core_3000::sptr  duc;
    } _perifs;

    uhd::stream_args_t _rx_stream_args;
    //! Bytes per item
    size_t _rx_bpi;
    size_t _rx_spp;

};

UHD_RFNOC_BLOCK_REGISTER(radio_ctrl, "Radio");
// vim: sw=4 expandtab:
