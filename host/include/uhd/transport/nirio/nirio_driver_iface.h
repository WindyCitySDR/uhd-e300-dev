#ifndef __NIRIO_DRIVER_IFACE_H__
#define __NIRIO_DRIVER_IFACE_H__

#include <stddef.h>
#include <stdint.h>
#include <uhd/transport/nirio/status.h>
#include <uhd/config.hpp>
#ifdef UHD_PLATFORM_WIN32
    #include <Windows.h>
    #pragma warning(disable:4201)  // nonstandard extension used : nameless struct/union
        #include <WinIoCtl.h>
    #pragma warning(pop)
#endif

// CTL_CODE macro for non-win OSes
#ifndef UHD_PLATFORM_WIN32
    #define CTL_CODE(a,controlCode,b,c) (controlCode)
#endif

typedef int32_t tRioStatusCode;

namespace nNIRIOSRV200
{

const uint32_t kPrivateIoctlBase = 0x800;

const uint32_t kRioIoctlSyncOp =
   CTL_CODE(FILE_DEVICE_UNKNOWN,
            kPrivateIoctlBase + 4,
            METHOD_OUT_DIRECT,
            FILE_READ_DATA | FILE_WRITE_DATA);
                                ///< The synchronous operation code. Note: We
                                /// must use METHOD_OUT_DIRECT on the syncOp()
                                /// IOCTL to ensure the contents of the output
                                /// block are available in the kernel.

const uint32_t kRioIoctlGetInterfaceNumber =
   CTL_CODE(FILE_DEVICE_UNKNOWN,
            kPrivateIoctlBase + 6,
            METHOD_BUFFERED,
            FILE_READ_DATA);    ///< Get the interface number for a device

const uint32_t kRioIoctlGetSession =
   CTL_CODE(FILE_DEVICE_UNKNOWN,
            kPrivateIoctlBase + 8,
            METHOD_BUFFERED,
            FILE_READ_ACCESS);  ///< Gets a previously opened session to a device


const uint32_t kRioIoctlPostOpen =
   CTL_CODE(FILE_DEVICE_UNKNOWN,
            kPrivateIoctlBase + 9,
            METHOD_BUFFERED,
            FILE_READ_ACCESS);  ///< Called after opening a session


const uint32_t kRioIoctlPreClose =
   CTL_CODE(FILE_DEVICE_UNKNOWN,
            kPrivateIoctlBase + 10,
            METHOD_BUFFERED,
            FILE_READ_ACCESS);  ///< Called before closing a session

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


namespace nirio_driver_iface {

//Device handle definition
#if defined(UHD_PLATFORM_LINUX)
    typedef int rio_dev_handle_t;
#elif defined(UHD_PLATFORM_WIN32)
    typedef HANDLE rio_dev_handle_t;
#elif defined(UHD_PLATFORM_MACOS)
    typedef io_connect_t rio_dev_handle_t;
#else
    #error OS not supported by nirio_driver_iface.
#endif
static const rio_dev_handle_t INVALID_RIO_HANDLE = ((rio_dev_handle_t)-1);

//Memory mapping container definition
#if defined(UHD_PLATFORM_LINUX)
    struct rio_mmap_t {
        rio_mmap_t() : addr(NULL), size(0) {}
        void *addr;
        size_t size;

        bool is_null() { return (size == 0 || addr == NULL); }
    };
#elif defined(UHD_PLATFORM_WIN32)
    enum access_mode_t {
       nNIAPAL200_kAccessModeRead,
       nNIAPAL200_kAccessModeWrite
    };

    struct rio_mmap_params_t
    {
       uint64_t mapped_va_ptr;
       uint64_t map_ready_event_handle;
       uint32_t size;
       uint16_t memoryType;
       uint8_t access_mode;
    };

    struct rio_mmap_threadargs_t
    {
        rio_dev_handle_t device_handle;
        rio_mmap_params_t params;
        nirio_status status;
    };

    struct rio_mmap_t
    {
        rio_mmap_t() : addr(NULL) {}
        void *addr;
        HANDLE map_thread_handle;
        rio_mmap_threadargs_t map_thread_args;

        bool is_null() { return addr == NULL; }

//@TODO: Fix this
//    private:
//        rio_mmap_t &operator=(const rio_mmap_t &);
    };
#elif defined(UHD_PLATFORM_MACOS)
     struct rio_mmap_t {
         rio_mmap_t() : addr(NULL) {}
         void *addr;

         bool is_null() { return addr == NULL; }
     };
#else
    #error OS not supported by nirio_driver_iface.
#endif

    nirio_status rio_open(
        const char* device_path,
        rio_dev_handle_t& device_handle);

    void rio_close(
        rio_dev_handle_t& device_handle);

    bool rio_isopen(
        rio_dev_handle_t device_handle);

    nirio_status rio_ioctl(
        rio_dev_handle_t device_handle,
        uint32_t ioctl_code,
        const void *write_buf,
        size_t write_buf_len,
        void *read_buf,
        size_t read_buf_len);

    nirio_status rio_mmap(
        rio_dev_handle_t device_handle,
        uint16_t memory_type,
        size_t size,
        bool writable,
        rio_mmap_t &map);

    nirio_status rio_munmap(
        rio_mmap_t &map);
}

#endif
