/*
 * fpga_utils.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/fpga_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>

namespace nirio_interface
{

nirio_status fpga_utils::download_fpga(
	niriok_proxy& driver_proxy,
	download_mode_t mode,
	const std::string& filename)
{
    boost::scoped_array<uint8_t> buffer;
    uint32_t bytes_read = _read_fpga_bitstream(filename, mode == DOWNLOAD_TO_FLASH, buffer);
    if (bytes_read > 0) {
    	switch (mode) {
			case PROGRAM_FPGA:
		    	return driver_proxy.download_fpga(kRioDeviceDownloadAttributeDestinationFpga, buffer.get(), bytes_read);

			case DOWNLOAD_TO_FLASH:
		    	return driver_proxy.download_fpga(kRioDeviceDownloadAttributeDestinationFlash, buffer.get(), bytes_read);
    	}
    } else {
    	return -1;
    }
    return -1;
}

nirio_status fpga_utils::erase_fpga_from_flash(
	niriok_proxy& driver_proxy)
{
	return driver_proxy.download_fpga(kRioDeviceDownloadAttributeDestinationFlash, NULL, 0);
}

nirio_status fpga_utils::configure_fpga_from_flash(
	niriok_proxy& driver_proxy)
{
	return driver_proxy.poke(0x58, (uint32_t)1);
}
uint32_t fpga_utils::_read_fpga_bitstream(
	const std::string& filename,
	bool reverse_bits,
	boost::scoped_array<uint8_t>& buffer)
{
	if (filename == "") return 0;

	struct stat image_stat;
	stat(filename.c_str(), &image_stat);
	size_t file_size = image_stat.st_size;

	FILE* image_fd;
	buffer.reset(new uint8_t[file_size + 1]);
	image_fd = fopen(filename.c_str(), "rb");
	if (!image_fd) return 0;

	size_t bytes_read = fread(buffer.get(), 1, file_size, image_fd);
	fclose(image_fd);

	if (reverse_bits) {
		for (size_t i = 0; i < bytes_read; i++)
			buffer.get()[i] = _reverse(buffer.get()[i]);
	}

	return bytes_read;
}

uint8_t fpga_utils::_reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
} /* namespace nirio_interface */
