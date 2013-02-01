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

#include "time_core_3000.hpp"
#include <uhd/utils/safe_call.hpp>

using namespace uhd;

struct time_core_3000_impl : time_core_3000
{
    time_core_3000_impl(
        wb_iface::sptr iface, const size_t base,
        const readback_bases_type &readback_bases
    ):
        _iface(iface),
        _base(base),
        _readback_bases(readback_bases)
    {
        this->set_tick_rate(1); //init to non zero
    }

    ~time_core_3000_impl(void)
    {
        UHD_SAFE_CALL
        (
            //NOP
        )
    }

    void set_tick_rate(const double rate)
    {
        _tick_rate = rate;
    }

    uhd::time_spec_t get_time_now(void)
    {
        const boost::uint64_t ticks = _iface->peek64(_readback_bases.rb_now);
        return time_spec_t::from_ticks(ticks, _tick_rate);
    }

    uhd::time_spec_t get_time_last_pps(void)
    {
        const boost::uint64_t ticks = _iface->peek64(_readback_bases.rb_pps);
        return time_spec_t::from_ticks(ticks, _tick_rate);
    }

    void set_time_now(const uhd::time_spec_t &)
    {
        //TODO
    }

    void set_time_next_pps(const uhd::time_spec_t &)
    {
        //TODO
    }

    void set_time_source(const std::string &)
    {
        //TODO
    }

    std::vector<std::string> get_time_sources(void)
    {
        std::vector<std::string> sources;
        sources.push_back("internal");
        sources.push_back("external");
        return sources;
    }

    wb_iface::sptr _iface;
    const size_t _base;
    const readback_bases_type _readback_bases;
    double _tick_rate;
};

time_core_3000::sptr time_core_3000::make(
    wb_iface::sptr iface, const size_t base,
    const readback_bases_type &readback_bases
)
{
    return time_core_3000::sptr(new time_core_3000_impl(iface, base, readback_bases));
}
