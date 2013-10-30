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

#ifndef NIFPGA_LVBITX_H_
#define NIFPGA_LVBITX_H_

#include <uhd/transport/nirio/nirio_resource_manager.h>
#include <uhd/transport/nirio/nirio_interface.h>
#include <boost/smart_ptr.hpp>

namespace nifpga_interface {

class nifpga_lvbitx {
public:
    typedef boost::shared_ptr<nifpga_lvbitx> sptr;

    virtual const char* get_bitfile_path() = 0;
    virtual const char* get_signature() = 0;
    virtual const char* get_bitstream_checksum() = 0;

    virtual size_t get_input_fifo_count() = 0;
    virtual const char** get_input_fifo_names() = 0;

    virtual size_t get_output_fifo_count() = 0;
    virtual const char** get_output_fifo_names() = 0;

    virtual size_t get_control_count() = 0;
    virtual const char** get_control_names() = 0;

    virtual size_t get_indicator_count() = 0;
    virtual const char** get_indicator_names() = 0;

    virtual void init_register_info(nirio_interface::nirio_register_info_vtr& vtr) = 0;
    virtual void init_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr) = 0;
};
}

#endif /* NIFPGA_LVBITX_H_ */

