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

#include "rx_vita_core_3000.hpp"
#include <uhd/utils/safe_call.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/tuple/tuple.hpp>

#define REG_FRAMER_MAXLEN    _framer_base + 0
#define REG_FRAMER_SID       _framer_base + 4

#define REG_CTRL_CMD           _ctrl_base + 0
#define REG_CTRL_TIME_HI       _ctrl_base + 4
#define REG_CTRL_TIME_LO       _ctrl_base + 8

using namespace uhd;

struct rx_vita_core_3000_impl : rx_vita_core_3000
{
    rx_vita_core_3000_impl(
        wb_iface::sptr iface,
        const size_t framer_base,
        const size_t ctrl_base
    ):
        _iface(iface),
        _framer_base(framer_base),
        _ctrl_base(ctrl_base),
        _continuous_streaming(false)
    {
        this->set_tick_rate(1); //init to non zero
        this->set_nsamps_per_packet(1); //init to non zero
        this->clear();
    }

    ~rx_vita_core_3000_impl(void)
    {
        UHD_SAFE_CALL
        (
            this->clear();
        )
    }

    void clear(void)
    {
        //TODO
    }

    void set_nsamps_per_packet(const size_t nsamps)
    {
        _iface->poke32(REG_FRAMER_MAXLEN, nsamps);
    }

    void issue_stream_command(const uhd::stream_cmd_t &stream_cmd)
    {
        UHD_ASSERT_THROW(stream_cmd.num_samps <= 0x0fffffff);
        _continuous_streaming = stream_cmd.stream_mode == stream_cmd_t::STREAM_MODE_START_CONTINUOUS;

        //setup the mode to instruction flags
        typedef boost::tuple<bool, bool, bool, bool> inst_t;
        static const uhd::dict<stream_cmd_t::stream_mode_t, inst_t> mode_to_inst = boost::assign::map_list_of
                                                                //reload, chain, samps, stop
            (stream_cmd_t::STREAM_MODE_START_CONTINUOUS,   inst_t(true,  true,  false, false))
            (stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS,    inst_t(false, false, false, true))
            (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE, inst_t(false, false, true,  false))
            (stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE, inst_t(false, true,  true,  false))
        ;

        //setup the instruction flag values
        bool inst_reload, inst_chain, inst_samps, inst_stop;
        boost::tie(inst_reload, inst_chain, inst_samps, inst_stop) = mode_to_inst[stream_cmd.stream_mode];

        //calculate the word from flags and length
        boost::uint32_t cmd_word = 0;
        cmd_word |= boost::uint32_t((stream_cmd.stream_now)? 1 : 0) << 31;
        cmd_word |= boost::uint32_t((inst_chain)?            1 : 0) << 30;
        cmd_word |= boost::uint32_t((inst_reload)?           1 : 0) << 29;
        cmd_word |= boost::uint32_t((inst_stop)?             1 : 0) << 28;
        cmd_word |= (inst_samps)? stream_cmd.num_samps : ((inst_stop)? 0 : 1);

        //issue the stream command
        _iface->poke32(REG_CTRL_CMD, cmd_word);
        const boost::uint64_t ticks = (stream_cmd.stream_now)? 0 : stream_cmd.time_spec.to_ticks(_tick_rate);
        _iface->poke32(REG_CTRL_TIME_HI, boost::uint32_t(ticks >> 32));
        _iface->poke32(REG_CTRL_TIME_LO, boost::uint32_t(ticks >> 0)); //latches the command
    }

    void set_tick_rate(const double rate)
    {
        _tick_rate = rate;
    }

    void set_sid(const boost::uint32_t sid)
    {
        _iface->poke32(REG_FRAMER_SID, sid);
    }

    void setup(const uhd::stream_args_t &)
    {
    }

    wb_iface::sptr _iface;
    const size_t _framer_base;
    const size_t _ctrl_base;
    double _tick_rate;
    bool _continuous_streaming;
};

rx_vita_core_3000::sptr rx_vita_core_3000::make(
    wb_iface::sptr iface,
    const size_t framer_base,
    const size_t ctrl_base
)
{
    return rx_vita_core_3000::sptr(new rx_vita_core_3000_impl(iface, framer_base, ctrl_base));
}
