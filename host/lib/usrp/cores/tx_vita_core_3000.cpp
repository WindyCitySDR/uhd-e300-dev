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

#include "tx_vita_core_3000.hpp"
#include <uhd/utils/safe_call.hpp>

#define REG_CTRL_ERROR_POLICY           _ctrl_base + 0

using namespace uhd;

struct tx_vita_core_3000_impl : tx_vita_core_3000
{
    tx_vita_core_3000_impl(
        wb_iface::sptr iface,
        const size_t deframer_base,
        const size_t ctrl_base
    ):
        _iface(iface),
        _deframer_base(deframer_base),
        _ctrl_base(ctrl_base)
    {
        this->set_tick_rate(1); //init to non zero
        this->clear();
        this->set_underflow_policy("next_packet");
    }

    ~tx_vita_core_3000_impl(void)
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

    void set_tick_rate(const double rate)
    {
        _tick_rate = rate;
    }

    void set_underflow_policy(const std::string &policy)
    {
        if (policy == "next_packet")
        {
            _iface->poke32(REG_CTRL_ERROR_POLICY, (1 << 1));
        }
        else if (policy == "next_burst")
        {
            _iface->poke32(REG_CTRL_ERROR_POLICY, (1 << 2));
        }
        else if (policy == "wait")
        {
            _iface->poke32(REG_CTRL_ERROR_POLICY, (1 << 0));
        }
        else throw uhd::value_error("USRP TX cannot handle requested underflow policy: " + policy);
    }

    void setup(const uhd::stream_args_t &stream_args)
    {
        if (stream_args.args.has_key("underflow_policy"))
        {
            this->set_underflow_policy(stream_args.args["underflow_policy"]);
        }
    }

    wb_iface::sptr _iface;
    const size_t _deframer_base;
    const size_t _ctrl_base;
    double _tick_rate;
};

tx_vita_core_3000::sptr tx_vita_core_3000::make(
    wb_iface::sptr iface,
    const size_t deframer_base,
    const size_t ctrl_base
)
{
    return tx_vita_core_3000::sptr(new tx_vita_core_3000_impl(iface, deframer_base, ctrl_base));
}
