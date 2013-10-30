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

#define X300_FPGA_IMAGE_SIZE_BYTES 15877916
#define X300_FPGA_PROG_UDP_PORT 49157
#define X300_FLASH_SECTOR_SIZE 131072
#define X300_PACKET_SIZE_BYTES 256
#define X300_FPGA_SECTOR_START 32
#define X300_MAX_RESPONSE_BYTES 128
#define UDP_TIMEOUT 3
#define FPGA_LOAD_TIMEOUT 15

#define X300_FPGA_PROG_FLAGS_ACK     1
#define X300_FPGA_PROG_FLAGS_ERROR   2
#define X300_FPGA_PROG_FLAGS_INIT    4
#define X300_FPGA_PROG_FLAGS_CLEANUP 8
#define X300_FPGA_PROG_FLAGS_ERASE   16
#define X300_FPGA_PROG_FLAGS_VERIFY  32
#define X300_FPGA_PROG_CONFIGURE     64
#define X300_FPGA_PROG_CONFIG_STATUS 128

#include <boost/cstdint.hpp>

boost::uint8_t bitswap(boost::uint8_t b);

typedef struct {
    boost::uint32_t flags;
    boost::uint32_t sector;
    boost::uint32_t index;
    boost::uint32_t size;
    boost::uint16_t data[128];
} x300_fpga_update_data_t;
