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


#ifndef  ___NiRioConstants_H___
#define  ___NiRioConstants_H___

#include <stddef.h>
#include <stdint.h>

const char* const kRioNIVendorName = "National Instruments";
//
// Basic constants.
//
typedef uint64_t tNIRIO_u64;
typedef uint32_t tNIRIO_u32;
typedef uint16_t tNIRIO_u16;
typedef uint8_t tNIRIO_u8;
typedef int32_t tNIRIO_i32;

const tNIRIO_u64 kRioInvalid64        = 0xffffffffffffffffULL;
const tNIRIO_u32 kRioInvalid32        = 0xffffffff;
const tNIRIO_u16 kRioInvalid16        = 0xffff;
const tNIRIO_u8  kRioInvalid8         = 0xff;
const tNIRIO_u32 kRioInvalid          = kRioInvalid32;
const tNIRIO_u32 kRioMaxLen           = 256;
const tNIRIO_u32 kRioInvalidHandle    = 0;

//
// FIFO Constants
//
const tNIRIO_u32 kRioMinFifoVersionWithClearCapability = 1;

//
// Device Access
//

typedef char tRioDeviceAccess;
const tRioDeviceAccess kRioDeviceAccessDeny  = '-';
const tRioDeviceAccess kRioDeviceAccessAllow = '+';

/**
      DeviceUserData registered data structure for non R-series devices that
      utilize RIO technology.
*/

const tNIRIO_u16 kRIODeviceUserDataKey  = 0x4352;  /// 'CR' for CompactRio

typedef struct {
   tNIRIO_u32 identifier;    /// Use kDevDeviceClassRIO.
   tNIRIO_u32 serialNumber;
} tRIODeviceUserData;

// Timeouts
const tNIRIO_u32 kRioTimeoutZero      = 0;
// kRioTimeoutInfinite should be in sync with NiFpga_InfiniteTimeout in
// chi/trunk/X.X/source/fpgainterfacecapi/NiFpga.h
const tNIRIO_u32 kRioTimeoutInfinite  = 0xFFFFFFFFUL;

//
// Device attribute identifiers
//
// We reserve "blocks" of attributes for the "channels" that carry DMA
// FIFOs and P2P endpoints. The "max channels" constant makes these
// blocks consistent in the attribute enum.
//

const tNIRIO_u32 kRioDmaMaxChannels   = 256;

// 32-bit attributes
// Name                           =  X,   // AddedVersion: Comments
typedef enum {
   kRioInitialized                =  0UL, // 200
   kRioInterfaceNumber            =  1UL, // 200
   kRioProductNumber              =  2UL, // 200
   kRioVendorNumber               =  3UL, // 200
   kRioSerialNumber               =  4UL, // 200
   kRioSignature                  =  5UL, // 200: Obsolete in 360
   kRioResourceType               =  6UL, // 200
   kRioDeviceManager              =  7UL, // 200: Obsolete in 230
   kRioDefaultTimeout             =  8UL, // 200
   kRioLocalFifoRatio             =  9UL, // 200: Obsolete in 230

   kRioBusNumber                  = 10UL, // 200
   kRioDeviceNumber               = 11UL, // 200
   kRioFunctionNumber             = 12UL, // 200

   kRioRestrictedAccess           = 13UL, // 220: Obsolete in 230
   kRioCurrentVersion             = 14UL, // 220
   kRioOldestCompatibleVersion    = 15UL, // 220
   kRioClientIsBigEndian          = 16UL, // 220: kFalse means little-endian
   kRioFpgaInterruptControlOffset = 17UL, // 220

   kRioNumberMemWindows           = 18UL, // 230

   kRioMemBaseBar0                = 19UL, // 230
   kRioMemBaseBar1                = 20UL, // 230
   kRioMemSizeBar0                = 21UL, // 230
   kRioMemSizeBar1                = 22UL, // 230

   kRioSessionState               = 23UL, // 230
   kRioPersonalityLockTimeout     = 24UL, // 230
   kRioAddressSpace               = 25UL, // 230

   kRioChassisNumber              = 27UL, // 230
   kRioSlotNumber                 = 28UL, // 230
   kRioLocalFifoDefaultDepth      = 29UL, // 230

   kRioTriggerBusNumber           = 30UL, // 230
   kRioTriggerReserveLine         = 31UL, // 230
   kRioTriggerUnreserveLine       = 32UL, // 230
   kRioTriggerReservedLines       = 33UL, // 230

   kRioIrqNodeReserve             = 34UL, // 230 Obsolete in 3.5
   kRioFpgaInterruptEnable        = 35UL, // 230

   kRioIsItOkToDownload           = 36UL, // 230

   kRioFpgaResetOffset            = 37UL, // 230
   kRioFpgaResetWidthInBits       = 38UL, // 230
   kRioFpgaControlOffset          = 39UL, // 230

   kRioResetIfLastSession         = 40UL, // 230
   kRioHasDeviceAccess            = 41UL, // 230

   kRioBusInterfaceType           = 42UL, // 240

   kRioNumberOfDmaChannels        = 43UL, // 300
   kRioFpgaAddressSpaceOffset     = 44UL, // 300
   kRioBitstreamVersion           = 45UL, // 300
   kRioIsInAPciSlot               = 46UL, // 300
   kRioIsRio                      = 47UL, // 300

   kRioIsFpgaProgrammed           = 48UL, // 300

   // kRioFpgaGuid* are deprecated 13.0
   kRioFpgaGuid0                  = 49UL, // 300 32-bit values from
   kRioFpgaGuid1                  = 50UL, // 300    signature register
   kRioFpgaGuid2                  = 51UL, // 300    in the order read
   kRioFpgaGuid3                  = 52UL, // 300

   kRioFpgaDefaultSignatureOffset = 53UL, // 300 Default Offsets for FPGA
   kRioFpgaDefaultResetOffset     = 54UL, // 300    registers. Supplied by
   kRioFpgaDefaultResetSize       = 55UL, // 300    the board driver on device
   kRioFpgaDefaultControlOffset   = 56UL, // 300    start.
   kRioFpgaDefaultInterruptOffset = 57UL, // 300

   kRioIsItOkToTalkToHardware     = 58UL, // 300
   kRioHasNoFPGASwitch            = 59UL, // 310

   kRioIrql                       = 60UL, // 320

   kRioNumberOfDreqLines          = 61UL, // 320

   kRioFlashIoPortSize            = 62UL, // 320
   kRioFlashWindowNumber          = 63UL, // 320
   kRioFlashIoPortWindowSize      = 64UL, // 320

   kBoardFlashAutoLoadMode        = 65UL, // 320

   kRioHostInterfaceData          = 66UL, // 320

   kRioFlashWindowOffset          = 67UL, // 330

   kRioIsPciExpress               = 68UL, // 330

   kRioMemBaseBar                 = 69UL, // 330
   kRioMemSizeBar                 = 70UL, // 330

   kStc3P2PReaderDisabled         = 71UL, // 330
   kStc3P2PReaderEnabled          = 72UL, // 330

   kRioNumberOfSlots              = 73UL, // 350
   kRioBusType                    = 74UL, // 360
   kRioSupportsVisaPassport       = 75UL, // Added in 3.6.  This attribute is intended
                                          // to be used between RIO VISA passport and
                                          // RIO driver.
                                          // Since the RIO passport always runs locally
                                          // on the target, it always matches the version
                                          // of the RIO driver.
                                          // Therefore, we don't have to add this attribute
                                          // to the compatibility layer.  And when we remove
                                          // support for this attribute in the driver & the
                                          // RIO passport, we can just deprecate this attribute
                                          // without having to worry about backward compatibility

   kRioFirmwareVersion            = 76UL, // 400. Usb based RIO has a firmware version number
                                          // that needs to be reported by system api
   kUsbInterfaceNumber            = 77UL, // 400. Usb based RIO has a usb interface number
                                          // that needs to be reported by system api
   kRioTotalFifoMemoryLimitMiB    = 78UL, // 13.0. Amount of memory in mebibytes that all DMA FIFO host
                                          // memory buffers on the system _combined_ must not exceed.

   // Each P2P endpoint is associated at compile time with a "channel"
   // and at run time with a process-unique "endpoint session"
   // ID. This set of attributes is used to obtain the session ID for
   // a particular endpoint channel.
   kRioEndpointSessionIDBase      = 1000UL, // 320
   kRioEndpointSessionIDMax       = kRioEndpointSessionIDBase + kRioDmaMaxChannels, // 320

   // Each P2P endpoint is associated at compile time with a "channel"
   // and at run time with a globally-unique "hardware ID". This set
   // of attributes is used to obtain the hardware ID for a particular
   // endpoint channel.
   kRioEndpointHardwareIDBase     = 2000UL, // 320
   kRioEndpointHardwareIDMax      = kRioEndpointHardwareIDBase + kRioDmaMaxChannels, // 320

} tRioDeviceAttribute32;

// String attributes
typedef enum {
   kRioProductName                = 0UL,
   kRioWhatFpgaIsDoing            = 1UL,   // 230
   kRioResourceName               = 2UL,   // 230
   kRioCompilationResults         = 3UL,   // 300
   kRioCompatibleIDs              = 4UL,   // 300
   kRioChassisName                = 5UL,   // 320
   kRioFpgaTargetClass            = 6UL,   // 320
   kRioSavedBitFile               = 7UL,   // 360
} tRioDeviceAttributeStr;

//
// Host attributes
//

// 32-bit attributes
typedef enum {
   kRioHostCurrentVersion          = 0UL,
   kRioHostOldestCompatibleVersion = 1UL,
   kRioHostRpcServerPort           = 2UL,
   kRioHostRpcTimeout              = 3UL, // NI-RPC timeout (seconds)
   kRioHostDeviceDiscoveryTimeout  = 4UL,
   kRioHostHasDeviceAccess         = 5UL,
   kRioHostRpcSessionTimeout       = 6UL, // RIO session timeout (seconds)
   kRioHostTableSegmentSize        = 7UL, //< Number of entries per segment
                                          //  of the growable session table
   kRioActivePowerSource           = 8UL, // 310
   kRioDriverInitializeTimeout     = 9UL, // 330
} tRioHostAttribute32;

// String attributes
typedef enum {
   kRioHostAliases       = 0UL,
   kRioHostAlias         = 1UL,
   kRioHostDeviceAccess  = 2UL,
   kRioHostRecentDevices = 3UL,
   kRioHostInputVoltage  = 4UL,  // 310
   kRioHostTemperature   = 5UL,  // 310
} tRioHostAttributeString;

const tNIRIO_u32 kRioDynamicRpcServerPort = 0; // Reserved port that
                                               // indicates RPC server
                                               // should use a dynamic
                                               // port

//
// Flash Auto load modes
//

const tNIRIO_u32 kBoardFlashAutoLoadModeNone         = 0UL;
const tNIRIO_u32 kBoardFlashAutoLoadModeAnyReset     = 1UL;
const tNIRIO_u32 kBoardFlashAutoLoadModePowerOnReset = 2UL;

inline bool rioValidateFlashAutoLoadMode(const tNIRIO_u32 value)
{
   return value == kBoardFlashAutoLoadModeNone
          || value == kBoardFlashAutoLoadModeAnyReset
          || value == kBoardFlashAutoLoadModePowerOnReset;
}

//
// IO Window types
//

typedef enum {
   kRioAddressSpaceInvalid = 0,
   kRioAddressSpaceBusInterface = 1,
   kRioAddressSpaceFpga = 2,
   kRioAddressSpaceBarWindow = 3,            // 330
} tRioAddressSpace;

//
// IO accessors attributes
//

typedef enum {
   kRioIoAccessorDefault          = 0UL,
   kRioIoAccessorDestructiveRead  = 1UL,
} tRioIoAccessorAttribute;

//
// Device block[Read|Write] attributes
//

const tNIRIO_u32 kRioIoAttributeAccessByteWidthMask      = 0x0F;
const tNIRIO_u32 kRioIoAttributeAutoAddressIncrement     = 0x10;
const tNIRIO_u32 kRioIoAttributeDustMiteNtFlashBitstream = 0x20;

//
// Device download attributes
//

typedef enum {
   kRioDeviceDownloadAttributeDestinationFpga  = 0x1,
   kRioDeviceDownloadAttributeDestinationFlash = 0x2,
} tRioDeviceDownloadAttribute;

//
// Event types
//

typedef enum {
   kRioEventInvalidEvent      = -1,
   kRioEventFirstEvent        = 0,
   kRioEventPciInterrupt      = 0,
   kRioEventRemoval           = 1,
   kRioEventMaxNumberOfEvents = 2, // Add all other events before this one
} tRioEventType;

//
// Bus types
//

typedef enum {
   kRioBusTypeInvalid   = 0,
   kRioBusTypePci       = 1,
   kRioBusTypeBuiltIn   = 2,
   kRioBusTypeUsb       = 3,
} tRioBusType;

//
// Session states
//

typedef enum {
   kRioSSNothing        = 1,
   kRioSSExclusive      = 2,
   kRioSSInvariant      = 4,
   kRioSSOverride       = 6,
   kRioSSHardwareAccess = 7,
} tRioSessionState;

const tNIRIO_u32 kRioBusTypeShift     = 28;
const tNIRIO_u32 kRioBusTypeMask      = 0xF;   // bus type range: 0 - 15 (4 bits)

/// Signature bus number shift
const tNIRIO_u32 kRioBusNumberShift      = 8;
const tNIRIO_u32 kRioBusNumberMask       = 0xFF;  // range: 0 - 255 (8 bits)

/// Signature device number shift
const tNIRIO_u32 kRioSocketNumberShift   = 3;
const tNIRIO_u32 kRioSocketNumberMask    = 0x1F;  // range: 0 - 31 (5 bits)

/// Signature function number shift
const tNIRIO_u32 kRioFunctionNumberShift = 0;
const tNIRIO_u32 kRioFunctionNumberMask  = 0x7;   // range: 0 - 7 (3 bits)

// IO Address space mask
const tNIRIO_u32 kRioAddressSpaceShift = 16;  // Address space is first 16 bits
#define kRioAddressSpaceMask ((1<<kRioAddressSpaceShift) - 1)

const tNIRIO_u32 kRioMemBaseBarShift = 16;
#define kRioMemBaseBarMask ((1<<kRioMemBaseBarShift) - 1)

const tNIRIO_u32 kRioMemSizeBarShift = 16;
#define kRioMemSizeBarMask ((1<<kRioMemSizeBarShift) - 1)

//
// RIO Device categories.
//

typedef enum {
   kRioDeviceTypeInvalid = 0x00000000UL,   //!< invalid device
   kRioDeviceCRio        = 0x00000001UL,   //!< compactRIO device
   kRioDevicePxi         = 0x00000002UL,   //!< PXI device
   kRioDeviceFW          = 0x00000004UL,   //!< FireWire (1394) device
   kRioDeviceBB          = 0x00000008UL,   //!< BlueBonnet device
   kRioDevicePuma        = 0x00000010UL,   //!< Puma device
   kRioDeviceRfRio       = 0x00000020UL,   //!< RF RIO device
   kRioDeviceMpc5125  = 0x00000040UL,   //!< Mpc5125 device
   kRioDeviceSync        = 0x73796E63UL,   //!< 'sync' (NI-PAL ID)
   kRioDeviceDaq         = 0x20646171UL,   //!< 'daq ' (NI-PAL ID for new sync)
   kRioDeviceIMAQ        = 0x696D6171UL,   //!< 'imaq' (NI-PAL ID)
} tRioDeviceType;

//
// RIO bus interface types
//

typedef enum {
   kRioBusInterfaceTypeInvalid = 0x00000000UL,
   kRioDustMite                = 0x00000001UL,
   kRioDustMiteNt              = 0x00000002UL,
   kRioStc2                    = 0x00000003UL,
   kRioStc3                    = 0x00000004UL,
   kRioBusMpc5125           = 0x00000005UL,
} tRioBusInterfaceType;

//============================================================================
//  tRioScalarType - Type enums for numbereric data types.
//    IMPORTANT:  The numbereric values of these enums need to match those of the
//    nNIRVIDF100::tFifoDataType enum (which in turn matches the values of the
//    LabVIEW NumberType enum).  Note also that we only support a subset of the
//    LabVIEW NumberType enums.  If this set of supported enums ever grows, we need
//    to be careful to match the appropriate NumberType numbereric value.
//    TODO:  Replace with LabVIEW numbereric type enum.
//============================================================================
typedef enum {
   kRioScalarType_iB = 1UL,
   kRioScalarType_iW = 2UL,
   kRioScalarType_iL = 3UL,
   kRioScalarType_iQ = 4UL,
   kRioScalarType_uB = 5UL,
   kRioScalarType_uW = 6UL,
   kRioScalarType_uL = 7UL,
   kRioScalarType_uQ = 8UL,
} tRioScalarType;


//============================================================================
//  tRioFifoDataType - Defines data type information for an associated data
//  value (which is generally identified via an opaque pointer).  Note that we
//  currently only support scalar data type definitions.  One example of how we
//  use this information is for supporting FIFOs with arbitrary scalar
//    data types.
//       scalarType - The type of the underlying scalar we're representing
//       bitWidth   - The number of active data bits in the scalar value.  For
//          non fixed-point types, this will equal the precision of the datatype
//          (i.e. 8, 16, 32, 64).  For fixed-point datatypes, this will equal
//          the number of bits in the fixed-point number that contain information.
//       integerWordLength - The relative offset of the decimal point in regards
//          to the most-significant bit of the actual FXP data-type.
//          For non fixed-point types, this will equal the precision of the
//          datatype (i.e. 8, 16, 32, 64).  For fixed-point datatypes,
//          this value (somewhat unintuitively, given the name) can range from
//          -2048 to 2047.  An example of this would be bitWidth of 16,
//          integerWordLength of -1.  This configuration would give you a delta
//          of 2^-17 and a range of +/-0.25.  Note that this name is consistent
//          with LabVIEW FPGA.
//============================================================================
typedef struct {
   tRioScalarType scalarType;
   tNIRIO_u32     bitWidth;
   tNIRIO_i32     integerWordLength;
} tRioFifoDataType;

//
// Client identifiers. Used in NiRioSrv_fpga_OpenEx call to indicate
// the behaviour the client expects from the driver.
//
// NOTE: If you change this, update _toRioClient() in _NiRioUtilities.h
//

typedef enum {
   kRioClientDefault     = 0UL, // Supports all 8.6 and earlier clients
   kRioClientLabVIEW2009 = 1UL, // A LV 2009 client
   kRioClientNiFpga1_2   = 2UL, // An NiFpga 1.2 client
} tRioClient;


/// Data direction for a DMA channel
enum tDmaChannelDirection{
   kDmaChannelDirectionInvalid        = 0UL,
   kDmaChannelDirectionDeviceToHost   = 1UL,
   kDmaChannelDirectionHostToDevice   = 2UL,
};


inline bool rioClientUsesCodepage(const tRioClient client)
{
   //
   // The only client that ever provided codepage bits to us was
   // LabVIEW 2009. Earlier clients didn't support codepage chars, and
   // so couldn't pass them to us. LabVIEW 2010 and later converts all
   // codepage to utf-8 before passing to us.
   //

   return client == kRioClientLabVIEW2009;
}

#endif // ___NiRioConstants_H___
