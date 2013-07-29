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

#ifndef INCLUDED_B250_CLOCK_CTRL_HPP
#define INCLUDED_B250_CLOCK_CTRL_HPP

#include <uhd/types/serial.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>


enum b250_clock_which_t
{
    B250_CLOCK_WHICH_ADC0,
    B250_CLOCK_WHICH_ADC1,
    B250_CLOCK_WHICH_DAC0,
    B250_CLOCK_WHICH_DAC1,
    B250_CLOCK_WHICH_DB0_RX,
    B250_CLOCK_WHICH_DB0_TX,
    B250_CLOCK_WHICH_DB1_RX,
    B250_CLOCK_WHICH_DB1_TX,
    B250_CLOCK_WHICH_TEST,
};

struct b250_clock_ctrl : boost::noncopyable
{
    typedef boost::shared_ptr<b250_clock_ctrl> sptr;

    static sptr make(uhd::spi_iface::sptr spiface, const size_t slaveno, const double clock_rate);

    /*!
     * Get the master clock frequency for the fpga.
     * \return the clock frequency in Hz
     */
    virtual double get_master_clock_rate(void) = 0;

    //! enable/disable a particular clock
    virtual void enable_clock(const b250_clock_which_t which, const bool) = 0;

    /*!
     * Set the clock rate on the given clock.
     * \param rate the new clock rate
     * \throw exception when rate invalid
     */
    virtual void set_rate(const b250_clock_which_t which, double rate) = 0;

    /*!
     * Get a list of possible clock rates.
     * \return a list of clock rates in Hz
     */
    virtual std::vector<double> get_rates(const b250_clock_which_t which) = 0;

    //! enable the reference output
    virtual void set_ref_out(const bool) = 0;
};

#endif /* INCLUDED_B250_CLOCK_CTRL_HPP */
