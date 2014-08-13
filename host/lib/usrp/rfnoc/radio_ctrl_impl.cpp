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
        // SPP is important, and will be stored in the stream args
        _rx_stream_args.args["spp"] = get_bytes_per_output_packet(0) / _rx_bpi;

    }


    void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd)
    {
        _perifs.framer->issue_stream_command(stream_cmd);
    }

    void configure_flow_control_in(
            size_t cycles,
            size_t packets,
            size_t block_port
    ) {
        UHD_ASSERT_THROW(block_port == 0);
        _perifs.deframer->configure_flow_control(cycles, packets);
    }

    void configure_flow_control_out(
                size_t buf_size_pkts,
                size_t,
                const uhd::sid_t &
    ) {
        _perifs.framer->configure_flow_control(buf_size_pkts);
    }

    void reset_flow_control()
    {
        // nop
    }

    bool set_bytes_per_output_packet(
            size_t bpp,
            size_t out_block_port
    ) {
        UHD_ASSERT_THROW(out_block_port == 0);
        if (bpp % _rx_bpi) {
            return false;
        }
        _tree->access<size_t>(_root_path / "bytes_per_packet/default").set(bpp);

        size_t spp = bpp / _rx_bpi;
        _rx_stream_args.args["spp"] = spp;
        _perifs.framer->set_nsamps_per_packet(spp);
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
        UHD_ASSERT_THROW(out_block_port == 0);
        uhd::sid_t sid(next_address);
        if (sid.get_src_address() == 0) {
            sid.set_src_address(get_address());
        }
        _perifs.framer->set_sid(sid.get());
    }

    void setup_rx_streamer(uhd::stream_args_t &args, const uhd::sid_t &data_sid)
    {
        _perifs.framer->clear();
        // Set spp, if applicable
        if (not args.args.has_key("spp")) {
            args.args["spp"] = _rx_stream_args.args["spp"];
        } else {
            _rx_stream_args.args["spp"] = args.args["spp"];
        }
        size_t spp = boost::lexical_cast<size_t>(args.args["spp"]);
        if (not set_bytes_per_output_packet(spp * _rx_bpi, 0)) {
            throw uhd::value_error("radio_ctrl::setup_rx_streamer(): Invalid spp value.");
        }

        set_destination(data_sid.get_dst_address(), 0);

        _perifs.framer->setup(args);
        _perifs.ddc->setup(args);
    }

    void setup_tx_streamer(uhd::stream_args_t &args)
    {
        _perifs.deframer->clear();
        _perifs.deframer->setup(args);
        _perifs.duc->setup(args);
    }

    void handle_overrun(void) {
        _perifs.framer->handle_overflow();
    }

    uhd::time_spec_t get_time_now(void) {
        return _perifs.time64->get_time_now();
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

private:

    //! Stores pointers to all radio cores
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

};

UHD_RFNOC_BLOCK_MAKE_CALL(radio_ctrl);

