//! \file
//! This file defines the API for the USRP RIO.
//! \note Fixed-width integer types are required to use the USRP RIO API, niusrprio.  Either manually define these types or #include a file that defines these types (such as stdint.h).
//
// Copyright (c) National Instruments Corporation.  All rights reserved.
//

#ifndef niusrprio_h
#define niusrprio_h

#include "NiFpga.h"

#if NiFpga_Cpp
extern "C"
{
#endif

//! Open a handle to a NI USRP RIO device.
//! \param [in]  deviceIdentifier The NI USRP RIO device to open a handle to.  The device identifier can be found by calling niusrprio_getDevicesInformation(...).
//! \param [out] handle           The opened handle will be placed in this parameter.
//! \returns the numeric status code
//! \pre The caller is responsible for allocating (and deallocating) memory for the handle.
//!
int32_t niusrprio_open(uint32_t const deviceIdentifier, uint64_t * const handle);

//! Close a handle to a NI USRP RIO device.
//! \param [in] handle The handle to close.
//! \returns the numeric status code
//!
int32_t niusrprio_close(uint64_t const handle);

//! Query the Flash Auto Load Mode.
//! \param [in]  handle       The handle to perform the function on.
//! \param [out] autoLoadMode The flash auto load mode value will be read into this value.
//! \returns the numeric status code
//! \pre The caller is responsible for allocating (and deallocating) memory for the autoLoadMode.
//!
int32_t niusrprio_getFlashAutoLoadMode(uint64_t const handle, uint32_t * const autoLoadMode);

//! Set the Flash Auto Load Mode.
//! \param [in] handle       The handle to perform the function on.
//! \param [in] autoLoadMode The flash auto load mode value will be set to this value.
//! \returns the numeric status code
//!
int32_t niusrprio_setFlashAutoLoadMode(uint64_t const handle, uint32_t const autoLoadMode);

//! Download an FPGA bitstream to the Flash.
//! \param [in] handle        The handle to perform the function on.
//! \param [in] bitstream     The bitstream to write to Flash.
//! \param [in] bitstreamSize The size of the bitstream (in bytes).
//! \returns the numeric status code
//!
int32_t niusrprio_downloadToFlash(uint64_t const handle, uint8_t const * const bitstream, uint32_t const bitstreamSize);

//! Query the number of devices that are on the system.
//! \param [out] numberOfDevices The number of NI USRP RIO devices that are detected on the system will be written into this parameter.
//! \returns the numeric status code
//! \pre The caller is responsible for allocating (and deallocating) memory for the numberOfDevices.
//!
int32_t niusrprio_getNumberOfDevices(uint64_t * const numberOfDevices);

//! Query the information for all the devices on the system.
//! \param [in]  numberOfDevices   The number of NI USRP RIO devices on the system.  This number is used for safety.  The numberOfDevices must match exactly with the actual number of devices as seen during this call.  This is a small step to ensure the caller knows what they are doing, and hopefully allocates enough memory for the arrays.
//! \param [out] deviceIdentifiers The array of device identifiers for the devices will be written into this parameter.
//! \param [out] serialNumbers     The array of serial numbers for the devices will be written into this parameter.
//! \returns the numeric status code
//! \pre The caller is responsible for allocating (and deallocating) memory for the arrays.  The array must be at least of size numberOfDevices.
//!
int32_t niusrprio_getDevicesInformation(uint64_t const numberOfDevices, uint32_t * const deviceIdentifiers, uint64_t * const serialNumbers);

/**
 * You must call this function before all other function calls. This function
 * loads the NiFpga library so that all the other functions will work. If this
 * function succeeds, you must call NiFpga_Finalize after all other function
 * calls.
 *
 * @warning This function is not thread safe.
 *
 * @return result of the call
 */
int32_t niusrprio_Initialize(void);

/**
 * You must call this function after all other function calls if
 * NiFpga_Initialize succeeds. This function unloads the NiFpga library.
 *
 * @warning This function is not thread safe.
 *
 * @return result of the call
 */
int32_t niusrprio_Finalize(void);

#if NiFpga_Cpp
}
#endif

#endif
