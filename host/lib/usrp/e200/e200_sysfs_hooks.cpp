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

#ifdef E200_NATIVE

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <libudev.h>
#include <stdlib.h>

#define NPAGES 8

static int get_params_from_sysfs(unsigned long *buffer_length, unsigned long *control_length, unsigned long *phys_addr)
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
	udev_enumerate_add_match_sysname(enumerate, "40000000.axi-fpga");
//	udev_enumerate_add_match_subsystem(enumerate, "amba.0");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;

		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);

		printf("Sys Path: %s\n", udev_device_get_syspath(dev));

		*buffer_length = atol(udev_device_get_sysattr_value(dev, "buffer_length"));
		*control_length = atol(udev_device_get_sysattr_value(dev, "control_length"));
		*phys_addr = atol(udev_device_get_sysattr_value(dev, "phys_addr"));

		printf("buffer_length = %lX\n", *buffer_length);
		printf("control_length = %lX\n", *control_length);
		printf("phy_addr = %lX\n", *phys_addr);
	}

	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	return 0;
}

static bool e200_fpga_loaded_successfully(void)
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
     udev_enumerate_add_match_sysname(enumerate, "f8007000.devcfg");
     udev_enumerate_scan_devices(enumerate);
     devices = udev_enumerate_get_list_entry(enumerate);

     udev_list_entry_foreach(dev_list_entry, devices) {
          const char *path;

          path = udev_list_entry_get_name(dev_list_entry);
          dev = udev_device_new_from_syspath(udev, path);

          printf("Sys Path: %s\n", udev_device_get_syspath(dev));

          result = atol(udev_device_get_sysattr_value(dev, "prog_done"));
    }

	udev_enumerate_unref(enumerate);
	udev_unref(udev);

    if (result == 1) 
        return true;
    else
        return false;
}

#include "e200_fifo_config.hpp"
#include <uhd/exception.hpp>

e200_fifo_config_t e200_read_sysfs(void)
{

    if (not e200_fpga_loaded_successfully())
    {
        throw uhd::runtime_error("E200 FPGA load failed!");
    }

    e200_fifo_config_t config;

    unsigned long control_length = 0;
    unsigned long buffer_length = 0;
    unsigned long phys_addr = 0;
    const int ret = get_params_from_sysfs(&buffer_length, &control_length, &phys_addr);
    if (ret != 0)
    {
        throw uhd::runtime_error("e200: get_params_from_sysfs failed!");
    }

    config.ctrl_length = control_length;
    config.buff_length = buffer_length;
    config.phys_addr = phys_addr;
    return config;
}

#else //E200_NATIVE

#include "e200_fifo_config.hpp"
#include <uhd/exception.hpp>

e200_fifo_config_t e200_read_sysfs(void)
{
    throw uhd::runtime_error("e200_read_sysfs() !E200_NATIVE");
}

#endif //E200_NATIVE
