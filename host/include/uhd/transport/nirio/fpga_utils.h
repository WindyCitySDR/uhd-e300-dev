/*
 * fpga_utils.h
 *
 *  Created on: Apr 15, 2013
 *      Author: ashish
 */

#ifndef FPGA_UTILS_H_
#define FPGA_UTILS_H_

#include <uhd/transport/nirio/nirio_interface.h>
#include <string>
#include <boost/smart_ptr.hpp>

namespace nirio_interface
{
class fpga_utils
{
public:
	enum download_mode_t {
		PROGRAM_FPGA,
		DOWNLOAD_TO_FLASH
	};

	static nirio_status download_fpga(
		niriok_proxy& driver_proxy,
		download_mode_t mode,
		const std::string& filename);

	static nirio_status erase_fpga_from_flash(
		niriok_proxy& driver_proxy);

	static nirio_status configure_fpga_from_flash(
		niriok_proxy& driver_proxy);
	static uint32_t _read_fpga_bitstream(
		const std::string& filename,
		bool reverse_bits,
		boost::scoped_array<uint8_t>& buffer);
	static uint8_t _reverse(uint8_t b);
};
}
#endif /* FPGA_UTILS_H_ */
