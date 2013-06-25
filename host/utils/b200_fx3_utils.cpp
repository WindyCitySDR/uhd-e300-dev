//
// Copyright 2010-2013 Ettus Research LLC
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

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <libusb-1.0/libusb.h>
#include <sstream>
#include <string>
#include <cmath>
#include <cstring>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <uhd/transport/usb_zero_copy.hpp>


const static uint16_t FX3_VID = 0x04b4;
const static uint16_t FX3_DEFAULT_PID = 0x00f3;
const static uint16_t FX3_REENUM_PID = 0x00f0;

namespace po = boost::program_options;

using namespace uhd;
using namespace uhd::transport;

//used with lexical cast to parse a hex string
template <class T> struct to_hex{
    T value;
    operator T() const {return value;}
    friend std::istream& operator>>(std::istream& in, to_hex& out){
        in >> std::hex >> out.value;
        return in;
    }
};

uint16_t atoh(const std::string &string){
    if (string.substr(0, 2) == "0x"){
        return boost::lexical_cast<to_hex<uint16_t> >(string);
    }
    return boost::lexical_cast<uint16_t>(string);
}


int32_t main(int32_t argc, char *argv[]) {

    uint16_t vid, pid, address;
    std::string pid_str, vid_str, readwrite;
    bool read = true;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("vid", po::value<std::string>(&vid_str)->default_value("0x04b4"), "VID of device.")
        ("pid", po::value<std::string>(&pid_str)->default_value("0x00f0"), "PID of device.")
        ("speed, S", "Read back the USB mode currently in use")
        ("reset-device, D", "Reset the B2xx Device")
        ("reset-fpga, F", "Reset the FPGA (does not require re-programming")
        ("reset-usb, U", "Reset the USB subsystem on your host computer")
        ("init-device, I", "Initialize a B2xx device")
        ("load-fw, W", "Load a firmware (hex) file into the FX3")
        ("load-fpga, L", "Load a FPGA (bin) file into the FPGA")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Slave Fifo Tests %s") % desc << std::endl;
        return ~0;
    }

    vid = atoh(vid_str);
    pid = atoh(pid_str);

    if(readwrite == "write") { read = false; }

    /* Pointer to pointer of device, used to retrieve a list of devices. */
    libusb_device **devs;
    libusb_device_handle *dev_handle;
    libusb_context *ctx = NULL;
    libusb_error error_code;
    ssize_t num_devices;

    libusb_init(&ctx);
    libusb_set_debug(ctx, 4);
    libusb_get_device_list(ctx, &devs);
    dev_handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if(dev_handle == NULL) {
        std::cout << "Cannot open device with vid: " << vid << " and pid: "
            << pid << std::endl;
    } else { std::cout << "Device Opened" << std::endl; }
    libusb_free_device_list(devs, 1);

    /* Find out if kernel driver is attached, and if so, detach it. */
    if(libusb_kernel_driver_active(dev_handle, 0) == 1) {
        std::cout << "Kernel Driver Active... " << std::flush;

        if(libusb_detach_kernel_driver(dev_handle, 0) == 0) {
            std::cout << "Kernel Driver Detached!" << std::endl;
        }
    }

    /* Claim interface 0 of device. */
    error_code = (libusb_error) libusb_claim_interface(dev_handle, 0);
    std::cout << "Claimed Interface" << std::endl;

    uint32_t data_buffer[16];

    if(read) {
        memset(data_buffer, 0, sizeof(data_buffer));

        error_code = (libusb_error) libusb_control_transfer(dev_handle, \
                (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT), 0xBB, 0x00, \
                0x00, (uint8_t *) data_buffer, 64, 10000);
        std::cout << "Data sent!" << std::endl;

        if(error_code != 64) {
            std::cout << "ARGH! Return value: "
                << libusb_strerror(error_code) << std::endl;
        }

        error_code = (libusb_error) libusb_control_transfer(dev_handle, \
                (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN), 0x22, 0x00, \
                0x00, (uint8_t *) data_buffer, 64, 10000);
        std::cout << "Received " << error_code << " bytes!" << std::endl;

        for(int i = 0; i < error_code; i++) {
            std::cout << std::hex << std::noshowbase << std::setw(2)
                << (int) ((uint8_t *) data_buffer)[i] << " ";
        } std::cout << std::endl;

    } else {
        memset(data_buffer, 0, sizeof(data_buffer));
        data_buffer[0] = 0xB2145943;
        data_buffer[1] = 0x04B40008;

        error_code = (libusb_error) libusb_control_transfer(dev_handle, \
                (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT), 0xBA, 0x00, \
                0x00, (uint8_t *) data_buffer, 64, 10000);
        std::cout << "Data sent!" << std::endl;

        if(error_code != 64) {
            std::cout << "ARGH! Actual num bytes transferred: " \
                << error_code << std::endl;
        }

        memset(data_buffer, 0, sizeof(data_buffer));
        error_code = (libusb_error) libusb_control_transfer(dev_handle, \
                (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN), 0x22, 0x00, \
                0x00, (uint8_t *) data_buffer, 64, 10000);
        std::cout << "Received " << error_code << " bytes!" << std::endl;

        for(int i = 0; i < error_code; i++) {
            std::cout << std::hex << std::noshowbase << std::setw(2)
                << (int) ((uint8_t *) data_buffer)[i] << " ";
        } std::cout << std::endl;
    }

    error_code = (libusb_error) libusb_release_interface(dev_handle, 0);
    std::cout << "Released Interface" << std::endl;

    return 0;

}

