//
// Copyright 2013-2014 Ettus Research LLC
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

#ifdef E300_NATIVE

#include <cstdio>
#include <cstdlib>
#include <string>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <libudev.h>

#include <boost/format.hpp>
#include <uhd/utils/msg.hpp>

static const size_t NPAGES = 8;

static const std::string E300_AXI_FPGA_SYSFS = "40000000.axi-fpga";
static const std::string E300_XDEV_SYSFS = "f8007000.ps7-dev-cfg";

static int get_params_from_sysfs(unsigned long *buffer_length,
                                 unsigned long *control_length,
                                 unsigned long *phys_addr)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    udev = udev_new();
    if (!udev) {
        printf("Fail\n");
        return 1;
    }

    //TODO read /sys/devices/amba.0/f8007000.devcfg/prog_done for FPGA load
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysname(enumerate, E300_AXI_FPGA_SYSFS.c_str());
    //udev_enumerate_add_match_subsystem(enumerate, "amba.0");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        UHD_MSG(status) << boost::format("Sys Path: %s") % udev_device_get_syspath(dev)
                        << std::endl;

        *buffer_length = atol(udev_device_get_sysattr_value(dev, "buffer_length"));
        *control_length = atol(udev_device_get_sysattr_value(dev, "control_length"));
        *phys_addr = atol(udev_device_get_sysattr_value(dev, "phys_addr"));

        //printf("buffer_length = %lX\n", *buffer_length);
        //printf("control_length = %lX\n", *control_length);
        //printf("phy_addr = %lX\n", *phys_addr);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return 0;
}

static bool e300_fpga_loaded_successfully(void)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    udev = udev_new();
    if (!udev) {
        printf("Fail\n");
        return 1;
    }
    long result = 0;

    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysname(enumerate, E300_XDEV_SYSFS.c_str());
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        UHD_MSG(status) << boost::format("Sys Path: %s") %  udev_device_get_syspath(dev)
                        << std::endl;

        result = atol(udev_device_get_sysattr_value(dev, "prog_done"));
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    if (result == 1)
        return true;
    else
        return false;
}

#include "e300_fifo_config.hpp"
#include <uhd/exception.hpp>

e300_fifo_config_t e300_read_sysfs(void)
{

    if (not e300_fpga_loaded_successfully())
    {
        throw uhd::runtime_error("E300 FPGA load failed!");
    }

    e300_fifo_config_t config;

    unsigned long control_length = 0;
    unsigned long buffer_length = 0;
    unsigned long phys_addr = 0;
    const int ret = get_params_from_sysfs(&buffer_length, &control_length, &phys_addr);
    if (ret != 0)
        throw uhd::runtime_error("E300: get_params_from_sysfs failed!");

    config.ctrl_length = control_length;
    config.buff_length = buffer_length;
    config.phys_addr = phys_addr;
    return config;
}

#else //E300_NATIVE

#include "e300_fifo_config.hpp"
#include <uhd/exception.hpp>

e300_fifo_config_t e300_read_sysfs(void)
{
    throw uhd::runtime_error("e300_read_sysfs() !E300_NATIVE");
}

#endif //E300_NATIVE
