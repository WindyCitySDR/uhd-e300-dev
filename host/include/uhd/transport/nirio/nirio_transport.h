/**
    \file       nirio_transport.h
    \author     Erik Hons <erik.hons@ni.com>
    \date       12/06/2007

    \brief Constants for RIO IOCTLs

    � Copyright 2007. National Instruments. All rights reserved.
*/

#ifndef __NIRIO_TRANSPORT_H__
#define __NIRIO_TRANSPORT_H__

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <uhd/transport/nirio/status.h>

typedef int32_t tRioStatusCode;

namespace nNIRIOSRV200
{

const uint32_t kPrivateIoctlBase 			= 0x800;

/// The synchronous operation code. Note: We
/// must use METHOD_OUT_DIRECT on the syncOp()
/// IOCTL to ensure the contents of the output
/// block are available in the kernel.
const uint32_t kRioIoctlSyncOp              = kPrivateIoctlBase + 4;

/// Get the interface number for a device
const uint32_t kRioIoctlGetInterfaceNumber	= kPrivateIoctlBase + 6;

/// Gets a previously opened session to a device
const uint32_t kRioIoctlGetSession 			= kPrivateIoctlBase + 8;

/// Called after opening a session
const uint32_t kRioIoctlPostOpen            = kPrivateIoctlBase + 9;

/// Called before closing a session
const uint32_t kRioIoctlPreClose            = kPrivateIoctlBase + 10;

struct tIoctlPacketOut {
   tIoctlPacketOut(void* const _outBuf,
               const uint32_t _outSize,
               const tRioStatusCode _statusCode)
      {
         //Clear all 64 bits before assignment since it may only be 32bit pointer
         outBuf._64BitField = 0;
         outBuf.pointer = _outBuf;

         outSize    = _outSize;
         statusCode = _statusCode;
      };

   union
   {
      void* pointer;
      uint64_t _64BitField;
   } outBuf;

   uint32_t outSize;
   tRioStatusCode statusCode;
};

// -------------------------------
// kRioIoctlSyncOp defines

// Function Codes: defined as integers rather than enums because they
// are going to be carried accross boundaries so size matters

/// RIO function codes
namespace nRioFunction
{
   const uint32_t kGet32              = 0x00000001;
   const uint32_t kSet32              = 0x00000002;
   const uint32_t kGet64              = 0x00000003; // Obsolete in 230
   const uint32_t kSet64              = 0x00000004; // Obsolete in 230
   const uint32_t kBeginDownload      = 0x00000005; // Obsolete in 230
   const uint32_t kEndDownload        = 0x00000006; // Obsolete in 230
   const uint32_t kSetDriverConfig    = 0x00000007;
   const uint32_t kFifo               = 0x00000008;
   const uint32_t kUnused             = 0x00000009; // Used to be kIO
   const uint32_t kIO                 = 0x0000000A; // Used to be kAtomicIO
   const uint32_t kEvent              = 0x0000000B;
   const uint32_t kFifoStopAll        = 0x0000000C;
   const uint32_t kAddResource        = 0x0000000D;
   const uint32_t kGetString          = 0x0000000E;
   const uint32_t kSetString          = 0x0000000F;
   const uint32_t kGetResourceStrings = 0x00000010;
   const uint32_t kResolveAlias       = 0x00000011;
   const uint32_t kIrq                = 0x00000012;
   const uint32_t kDownload           = 0x00000013;
   const uint32_t kReset              = 0x00000014;
}

/// Add resource sub-function codes
namespace nRioDeviceAddResourceFunction
{
   const uint32_t kInputFifo        = 0xD0000001;
   const uint32_t kOutputFifo       = 0xD0000002;
   const uint32_t kAtomicRange      = 0xD0000003; // Obsolete in 230
}

/// Event sub-function codes
namespace nRioDeviceEventFunction
{
   const uint32_t kEnable           = 0xB0000001;
   const uint32_t kEnableLocal      = 0xB0000002;
   const uint32_t kDisable          = 0xB0000003;
   const uint32_t kDisableLocal     = 0xB0000004; // Obsolete in 230
   const uint32_t kWait             = 0xB0000005;
}

/// Fifo sub-function codes
namespace nRioDeviceFifoFunction
{
   const uint32_t kConfigure        = 0x80000001;
   const uint32_t kStart            = 0x80000002;
   const uint32_t kStop             = 0x80000003;
   const uint32_t kRead             = 0x80000004;
   const uint32_t kWrite            = 0x80000005;
   const uint32_t kWait             = 0x80000006;
   const uint32_t kGrant            = 0x80000007;
}

/// IO sub-function codes
///
/// Used to be called nRioDeviceAtomicIOFunction
///
namespace nRioDeviceIOFunction
{
   const uint32_t kRead32                       = 0xA0000001; // Obsolete
   const uint32_t kWrite32                      = 0xA0000002; // Obsolete
   const uint32_t kRegAcquire                   = 0xA0000003; // Obsolete
   const uint32_t kRegRelease                   = 0xA0000004; // Obsolete
   const uint32_t kPoke64                       = 0xA0000005;
   const uint32_t kPoke32                       = 0xA0000006;
   const uint32_t kPoke16                       = 0xA0000007;
   const uint32_t kPoke8                        = 0xA0000008;
   const uint32_t kPeek64                       = 0xA0000009;
   const uint32_t kPeek32                       = 0xA000000A;
   const uint32_t kPeek16                       = 0xA000000B;
   const uint32_t kPeek8                        = 0xA000000C;
   const uint32_t kReadBlock                    = 0xA000000D;
   const uint32_t kWriteBlock                   = 0xA000000E;
   const uint32_t kGetMemoryMappedIoWindow      = 0xA000000F;
   const uint32_t kGetMemoryMappedIoWindowSize  = 0xA0000010;
}

/// Irq sub-function codes
namespace nRioDeviceIrqFunction
{
   const uint32_t kIrqReserve      = 0x12000001;
   const uint32_t kIrqReserveLocal = 0x12000002;
   const uint32_t kIrqUnreserve    = 0x12000003;
   const uint32_t kIrqWait         = 0x12000004;
}

struct tRioIoctlBlock
{
	uint64_t inBuf;
	uint64_t outBuf;
	uint32_t inBufLength;
	uint32_t outBufLength;
	uint32_t bytesReturned;
	uint32_t padding;
};

struct tRioDeviceSocketInputParameters
{
   uint32_t function;
   uint32_t subfunction;

   union
   {
      struct
      {
         uint32_t attribute;
         uint32_t value;
      } attribute32;

      struct
      {
         uint32_t attribute;
         uint64_t value;
      } attribute64;

      struct
      {
         uint32_t attribute;
      } attributeStr;

      struct
      {
         uint32_t attribute;
      } download;

      union
      {
         struct
         {
            uint32_t eventType;
         } enable;
         struct
         {
            uint32_t eventType;
            uint32_t timeout;
         } wait;
         struct
         {
            uint32_t eventType;
         } disable;
      } event;

      union
      {
         struct
         {
            uint32_t channel;
            uint32_t baseAddress;
            uint32_t depthInSamples;
            uint32_t version;
         } fifo;
         struct
         {
            uint32_t channel;
            uint32_t baseAddress;
            uint32_t depthInSamples;
            uint32_t version;
            uint32_t scalarType;
            uint32_t bitWidth;
         } fifoWithDataType;
         struct
         {
            uint64_t rangeBaseAddress;
            uint32_t rangeSizeInBytes;
            uint32_t rangeAttribute;
         } atomic; // obsolete
      } add;

      struct
      {
         uint32_t channel;

         union
         {
            struct
            {
               uint32_t requestedDepth; // TODO: when we make win64U builds, we'll want size_t
               uint8_t  requiresActuals;
            } config;
            struct
            {
               uint32_t timeout;
            } read;
            struct
            {
               uint32_t timeout;
               uint32_t scalarType;
               uint32_t bitWidth;
            } readWithDataType;
            struct
            {
               uint32_t timeout;
            } write;
            struct
            {
               uint32_t timeout;
               uint32_t scalarType;
               uint32_t bitWidth;
            } writeWithDataType;
            struct
            {
               // didn't use size_t because it made our struct too big!
               uint32_t elementsRequested; // TODO: when we make win64U builds, we'll want size_t
               uint32_t scalarType;
               uint32_t bitWidth;
               uint32_t timeout;
               uint8_t  output;
            } wait;
            struct
            {
               uint32_t elements; // TODO: when we make win64U builds, we'll want size_t
            } grant;
         } op;
      } fifo;

      struct
      {
         uint64_t addr64;
         uint32_t timeout;
         uint32_t attribute;
      } atomic; // Obsolete

      struct
      {
         uint32_t offset;
         union
         {
            uint64_t value64;
            uint32_t value32;
            uint16_t value16;
            uint8_t  value8;
         } value;
         union
         {
            uint32_t   sizeToMap;
         } memoryMappedIoWindow;
      } io;

      struct
      {
         uint32_t offset;
         uint32_t attribute;
      } ioblock;

      struct
      {
         uint32_t reverse;
      } resolveAlias;

      union
      {
         struct
         {
            uint32_t irqs;
            int32_t  timeout;
         } wait;
      } irq;

   } params;

   uint32_t inbufByteLen;

   union
   {
      const void* pointer;
      uint64_t    _64BitField;   // Makes us 64-bit ready
   } inbuf;
};

static inline void initRioDeviceSocketInputParameters(tRioDeviceSocketInputParameters& param, const void* const buf, const uint32_t len)
{
   param.inbuf._64BitField = 0;  //Zero's out .pointer in case it isn't 64-bit
   param.inbuf.pointer = buf;
   param.inbufByteLen = len;
}


struct tRioDeviceSocketOutputParameters
{
   // Return paramters: [out]
   union
   {
      struct
      {
         uint32_t value;
      } attribute32;

      struct
      {
         uint64_t value;
      } attribute64;

      union
      {
         struct
         {
            uint32_t handle;
         } enable;
      } event;

      struct
      {
         union
         {
            struct
            {
               uint32_t actualDepth; // TODO: when we make win64U builds, we'll want size_t
               uint32_t actualSize; // TODO: when we make win64U builds, we'll want size_t
            } config;
            struct
            {
               uint32_t numberRead;
               uint32_t numberRemaining;
            } read;
            struct
            {
               uint32_t numberRemaining;
            } write;
            struct
            {
               union
               {
                  void*    pointer;
                  uint64_t _64BitField; // Makes us 64-bit ready
               } elements;
               // elementsAcquired: stuffed in the buffer so the struct wouldn't grow
               // elementsRemaining: stuffed in the buffer so the struct wouldn't grow
            } wait;
         } op;
      } fifo;

      struct
      {
        union
         {
            union
            {
               uint64_t value64;
               uint32_t value32;
               uint16_t value16;
               uint8_t  value8;
            } value;
            union
            {
               void*    memoryMappedAddress;
               uint64_t _64BitField; // Makes us 64-bit ready
            } memoryMappedIoWindow;
            union
            {
               uint32_t   size;
            } memoryMappedIoWindowSize;
         };
      } io;

      uint32_t stringLength;

      struct
      {
         uint32_t asserted;
      } irq;

   } params;

   uint32_t    outbufByteLen;

   union
   {
      void*    pointer;
      uint64_t _64BitField; // Makes us 64-bit ready
   } outbuf;
};

static inline void initRioDeviceSocketOutputParameters(tRioDeviceSocketOutputParameters& param, void* buf, uint32_t len)
{
   param.outbuf._64BitField = 0;  //Zero's out .pointer in case it isn't 64-bit
   param.outbuf.pointer = buf;
   param.outbufByteLen = len;
}

} // namespace nNIRIOSRV200


namespace nirio_transport {

struct rio_memory_map {
	rio_memory_map() : addr(NULL), size(0) {}
    void *addr;
    size_t size;

    bool is_null() { return size == 0; }
};

inline static nirio_status rio_ioctl(
	int device_handle,
	uint32_t ioctl_code,
	const void *write_buf,
	size_t write_buf_len,
	void *read_buf,
	size_t read_buf_len)
{
	nNIRIOSRV200::tRioIoctlBlock ioctl_block = {0,0,0,0,0,0};

	// two-casts necessary to prevent pointer sign-extension
	ioctl_block.inBuf        = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(write_buf));
	ioctl_block.inBufLength  = write_buf_len;
	ioctl_block.outBuf       = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(read_buf));
	ioctl_block.outBufLength = read_buf_len;

	return ::ioctl(device_handle, ioctl_code, &ioctl_block);
}

inline static nirio_status rio_mmap(
	int device_handle,
	uint16_t memory_type,
	size_t size,
	int access_mode,
	rio_memory_map &map)
{
	if (access_mode == PROT_WRITE) access_mode |= PROT_READ;	//Write-only mode not supported
	map.addr = ::mmap(NULL, size, access_mode, MAP_SHARED, device_handle, (off_t) memory_type * sysconf(_SC_PAGESIZE));
	map.size = size;

	if (map.addr == MAP_FAILED)	{
		map.addr = NULL;
		map.size = 0;
		if (errno == EINVAL)
		    return NiRio_Status_InvalidParameter;
		else if (errno == ENOMEM)
		    return NiRio_Status_MemoryFull;
		else
		    return NiRio_Status_SoftwareFault;
	}
	return 0;
}

inline static nirio_status rio_munmap(
	rio_memory_map &map)
{
	nirio_status status = 0;
	if (map.addr != NULL) {
		status = ::munmap(map.addr, map.size);

		map.addr = NULL;
		map.size = 0;
	}
	return status;
}

}

#endif
