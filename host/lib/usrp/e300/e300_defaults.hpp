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

#ifndef INCLUDED_E300_DEFAULTS_HPP
#define INCLUDED_E300_DEFAULTS_HPP

namespace uhd { namespace usrp { namespace e300 {

static const double DEFAULT_TICK_RATE       = 32e6;
static const double MAX_TICK_RATE           = 50e6;
static const double MIN_TICK_RATE           = 1e6;

static const double DEFAULT_TX_SAMP_RATE    = 1.0e6;
static const double DEFAULT_RX_SAMP_RATE    = 1.0e6;
static const double DEFAULT_DDC_FREQ        = 0.0;
static const double DEFAULT_DUC_FREQ        = 0.0;

static const double DEFAULT_FE_GAIN         = 0.0;
static const double DEFAULT_FE_FREQ         = 1.0e9;
static const double DEFAULT_FE_BW           = 56e6;

static const std::string DEFAULT_TIME_SRC   = "none";
static const std::string DEFAULT_CLOCK_SRC  = "internal";

static const size_t DEFAULT_RX_DATA_FRAME_SIZE = 2048;
static const size_t DEFAULT_RX_DATA_NUM_FRAMES = 128;

static const size_t DEFAULT_TX_DATA_FRAME_SIZE = 2048;
static const size_t DEFAULT_TX_DATA_NUM_FRAMES = 128;

static const size_t DEFAULT_CTRL_FRAME_SIZE    = 64;
static const size_t DEFAULT_CTRL_NUM_FRAMES    = 32;


}}} // namespace

#endif // INCLUDED_E300_DEFAULTS_HPP
