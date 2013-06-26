/*
 * nirio_fifo.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/nirio_fifo.h>

namespace nirio_interface {

template <>
datatype_info_t nirio_fifo<int8_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_I8, 1);
}

template <>
datatype_info_t nirio_fifo<int16_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_I16, 2);
}

template <>
datatype_info_t nirio_fifo<int32_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_I32, 4);
}

template <>
datatype_info_t nirio_fifo<int64_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_I64, 8);
}

template <>
datatype_info_t nirio_fifo<uint8_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_U8, 1);
}

template <>
datatype_info_t nirio_fifo<uint16_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_U16, 2);
}

template <>
datatype_info_t nirio_fifo<uint32_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_U32, 4);
}

template <>
datatype_info_t nirio_fifo<uint64_t>::_get_datatype_info()
{
	return datatype_info_t(SCALAR_U64, 8);
}


}
