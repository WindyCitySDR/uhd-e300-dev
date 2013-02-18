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

#include "e200_impl.hpp"
#include "e200_regs.hpp"
#include <uhd/utils/msg.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/images.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

using namespace uhd;
namespace fs = boost::filesystem;

/***********************************************************************
 * Discovery
 **********************************************************************/
static device_addrs_t e200_find(const device_addr_t &hint)
{
    device_addrs_t e200_addrs;

    //return an empty list of addresses when type is set to non-e200
    if (hint.has_key("type") and hint["type"] != "e200") return e200_addrs;

    //device node not provided, assume its 0
    if (not hint.has_key("node"))
    {
        device_addr_t new_addr = hint;
        new_addr["node"] = "/dev/axi_fpga";
        return e200_find(new_addr);
    }

    //use the given device node name
    if (fs::exists(hint["node"]))
    {
        device_addr_t new_addr;
        new_addr["type"] = "e200";
        new_addr["node"] = fs::system_complete(fs::path(hint["node"])).string();
        //TODO read EEPROM!
        new_addr["name"] = "";
        new_addr["serial"] = "";
        if (
            (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
            (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
        ){
            e200_addrs.push_back(new_addr);
        }
    }

    return e200_addrs;
}

/***********************************************************************
 * Make
 **********************************************************************/
static device::sptr e200_make(const device_addr_t &device_addr)
{
    return device::sptr(new e200_impl(device_addr));
}

UHD_STATIC_BLOCK(register_e200_device)
{
    device::register_device(&e200_find, &e200_make);
}

/***********************************************************************
 * Structors
 **********************************************************************/
e200_impl::e200_impl(const uhd::device_addr_t &device_addr)
{
    //extract the FPGA path for the E200
    const std::string e200_fpga_image = find_image_path(
        device_addr.get("fpga", E200_FPGA_FILE_NAME)
    );

    //load fpga image - its super fast
    this->load_fpga_image(e200_fpga_image);

    _fifo_iface = e200_fifo_interface::make(e200_read_sysfs());

    _tree = property_tree::make();
}

e200_impl::~e200_impl(void)
{
    /* NOP */
}

void e200_impl::load_fpga_image(const std::string &path)
{
    if (not fs::exists("/dev/xdevcfg"))
    {
        ::system("mknod /dev/xdevcfg c 259 0");
        //throw uhd::runtime_error("no xdevcfg, please run: mknod /dev/xdevcfg c 259 0");
    }

    UHD_MSG(status) << "Loading FPGA image: " << path << "..." << std::flush;

    /*
    std::ifstream fpga_file(path.c_str(), std::ios_base::binary);
    UHD_ASSERT_THROW(fpga_file.good());

    std::ofstream xdev_file("/dev/xdevcfg", std::ios_base::binary);
    UHD_ASSERT_THROW(xdev_file.good());

    char buff[(1 << 22)]; //big enough for entire image
    fpga_file.read(buff, sizeof(buff));
    xdev_file.write(buff, fpga_file.gcount());

    fpga_file.close();
    xdev_file.close();
    //*/

    //cat works, fstream blows
    const std::string cmd = str(boost::format("cat \"%s\" > /dev/xdevcfg") % path);
    ::system(cmd.c_str());

    UHD_MSG(status) << " done" << std::endl;
}

//sshfs jblum@blarg:/home/jblum/src src
//utils/uhd_usrp_probe --tree --args="fpga=/home/root/b200_xport_t0.bin,type=e200"
