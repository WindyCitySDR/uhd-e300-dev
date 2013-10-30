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
