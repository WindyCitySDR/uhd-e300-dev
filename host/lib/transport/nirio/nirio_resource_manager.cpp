/*
 * nirio_fifo_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/nirio_resource_manager.h>

//@TODO: Figure out a better way to suppress anonymous struct init warnings
#ifdef __GNUC__
    #pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

namespace nirio_interface
{

nirio_resource_manager::nirio_resource_manager(
	niriok_proxy& proxy) : _kernel_proxy(proxy), _fifo_info_map(), _reg_info_map()
{
}

nirio_resource_manager::~nirio_resource_manager()
{
	finalize();
}

nirio_status nirio_resource_manager::initialize(
	const nirio_register_info_vtr& reg_info_vtr,
	const nirio_fifo_info_vtr& fifo_info_vtr)
{
	nirio_status status = 0;
	for (nirio_fifo_info_vtr::const_iterator it = fifo_info_vtr.begin(); it != fifo_info_vtr.end(); it++) {
		const nirio_fifo_info_t& fifo_info = *it;
		status = _add_fifo_resource(fifo_info);
		if (nirio_status_fatal(status)) return status;

		_fifo_info_map.insert(fifo_info_map_t::value_type(fifo_info.name, fifo_info));
	}
	for (nirio_register_info_vtr::const_iterator it = reg_info_vtr.begin(); it != reg_info_vtr.end(); it++) {
		const nirio_register_info_t& reg_info = *it;

		_reg_info_map.insert(register_info_map_t::value_type(reg_info.name, reg_info));
	}
	return _set_driver_config();
}

void nirio_resource_manager::finalize()
{
	_fifo_info_map.clear();
}

nirio_status nirio_resource_manager::get_register_offset(
	const char* register_name,
	uint32_t& offset)
{
	register_info_map_t::const_iterator it = _reg_info_map.find(fifo_info_map_t::key_type(register_name));
	if (it == _reg_info_map.end()) return NiRio_Status_InvalidParameter;

	offset = (*it).second.offset;

	return NiRio_Status_Success;
}


nirio_status nirio_resource_manager::_add_fifo_resource(
	const nirio_fifo_info_t& fifo_info)
{
	nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
	nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};

	in.function    = nNIRIOSRV200::nRioFunction::kAddResource;
	in.subfunction = (fifo_info.direction == OUTPUT_FIFO) ?
			nNIRIOSRV200::nRioDeviceAddResourceFunction::kOutputFifo :
			nNIRIOSRV200::nRioDeviceAddResourceFunction::kInputFifo;

	in.params.add.fifoWithDataType.channel        = fifo_info.channel;
	in.params.add.fifoWithDataType.baseAddress    = fifo_info.base_addr;
	in.params.add.fifoWithDataType.depthInSamples = fifo_info.depth;
	in.params.add.fifoWithDataType.scalarType     = fifo_info.scalar_type;
	in.params.add.fifoWithDataType.bitWidth       = fifo_info.width;
	in.params.add.fifoWithDataType.version        = fifo_info.version;

	return _kernel_proxy.sync_operation(&in, sizeof(in), &out, sizeof(out));
}

nirio_status nirio_resource_manager::_set_driver_config()
{
	nNIRIOSRV200::tRioDeviceSocketInputParameters in = {};
	nNIRIOSRV200::tRioDeviceSocketOutputParameters out = {};
	in.function    = nNIRIOSRV200::nRioFunction::kSetDriverConfig;
	in.subfunction = 0;

	return _kernel_proxy.sync_operation(&in, sizeof(in), &out, sizeof(out));
}

nirio_fifo_info_t* nirio_resource_manager::_lookup_fifo_info(const char* fifo_name) {
	fifo_info_map_t::iterator it = _fifo_info_map.find(fifo_info_map_t::key_type(fifo_name));
	if (it == _fifo_info_map.end()) return NULL;

	return &((*it).second);
}

} /* namespace nirio_interface */

#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif
