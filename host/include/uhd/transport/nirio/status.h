/*
 * status.h
 *
 *  Created on: Apr 3, 2013
 *      Author: ashish
 */

#ifndef STATUS_H_
#define STATUS_H_

#define ENABLE_EXTENDED_ERROR_INFO false

typedef int32_t nirio_status;

#define nirio_status_fatal(status) ((status) < 0)
#define nirio_status_not_fatal(status) ((status) >= 0)

#define nirio_status_chain(func, status) 	\
	if (nirio_status_not_fatal(status)) {	\
		status = (func);					\
		if (ENABLE_EXTENDED_ERROR_INFO && nirio_status_fatal(status)) {	\
			fprintf(stderr,"ERROR: The following function call returned status code %d\n%s\n%s:%d\n",status,#func,__FILE__,__LINE__);	\
		}									\
	}										\

/**
 * No errors or warnings.
 */
static const nirio_status NiRio_Status_Success = 0;

/**
 * The timeout expired before the FIFO operation could complete.
 */
static const nirio_status NiRio_Status_FifoTimeout = -50400;

/**
 * A memory allocation failed. Try again after rebooting.
 */
static const nirio_status NiRio_Status_MemoryFull = -52000;

/**
 * An unexpected software error occurred.
 */
static const nirio_status NiRio_Status_SoftwareFault = -52003;

/**
 * A parameter to a function was not valid. This could be a NULL pointer, a bad
 * value, etc.
 */
static const nirio_status NiRio_Status_InvalidParameter = -52005;

/**
 * A required resource was not found. The NiFpga.* library, the RIO resource, or
 * some other resource may be missing.
 */
static const nirio_status NiRio_Status_ResourceNotFound = -52006;

/**
 * A required resource was not properly initialized. This could occur if
 * NiFpga_Initialize was not called or a required NiFpga_IrqContext was not
 * reserved.
 */
static const nirio_status NiRio_Status_ResourceNotInitialized = -52010;

/**
 * The FPGA is already running.
 */
static const nirio_status NiRio_Status_FpgaAlreadyRunning = -61003;

/**
 * The bitfile was not compiled for the specified resource's device type.
 */
static const nirio_status NiRio_Status_DeviceTypeMismatch = -61024;

/**
 * An error was detected in the communication between the host computer and the
 * FPGA target.
 */
static const nirio_status NiRio_Status_CommunicationTimeout = -61046;

/**
 * The timeout expired before any of the IRQs were asserted.
 */
static const nirio_status NiRio_Status_IrqTimeout = -61060;

/**
 * The specified bitfile is invalid or corrupt.
 */
static const nirio_status NiRio_Status_CorruptBitfile = -61070;

/**
 * The requested FIFO depth is invalid. It is either 0 or an amount not
 * supported by the hardware.
 */
static const nirio_status NiRio_Status_BadDepth = -61072;

/**
 * The number of FIFO elements is invalid. Either the number is greater than the
 * depth of the host memory DMA FIFO, or more elements were requested for
 * release than had been acquired.
 */
static const nirio_status NiRio_Status_BadReadWriteCount = -61073;

/**
 * A hardware clocking error occurred. A derived clock lost lock with its base
 * clock during the execution of the LabVIEW FPGA VI. If any base clocks with
 * derived clocks are referencing an external source, make sure that the
 * external source is connected and within the supported frequency, jitter,
 * accuracy, duty cycle, and voltage specifications. Also verify that the
 * characteristics of the base clock match the configuration specified in the
 * FPGA Base Clock Properties. If all base clocks with derived clocks are
 * generated from free-running, on-board sources, please contact National
 * Instruments technical support at ni.com/support.
 */
static const nirio_status NiRio_Status_ClockLostLock = -61083;

/**
 * The operation could not be performed because the FPGA is busy. Stop all
 * activities on the FPGA before requesting this operation. If the target is in
 * Scan Interface programming mode, put it in FPGA Interface programming mode.
 */
static const nirio_status NiRio_Status_FpgaBusy = -61141;

/**
 * The operation could not be performed because the FPGA is busy operating in
 * FPGA Interface C API mode. Stop all activities on the FPGA before requesting
 * this operation.
 */
static const nirio_status NiRio_Status_FpgaBusyFpgaInterfaceCApi = -61200;

/**
 * The chassis is in Scan Interface programming mode. In order to run FPGA VIs,
 * you must go to the chassis properties page, select FPGA programming mode, and
 * deploy settings.
 */
static const nirio_status NiRio_Status_FpgaBusyScanInterface = -61201;

/**
 * The operation could not be performed because the FPGA is busy operating in
 * FPGA Interface mode. Stop all activities on the FPGA before requesting this
 * operation.
 */
static const nirio_status NiRio_Status_FpgaBusyFpgaInterface = -61202;

/**
 * The operation could not be performed because the FPGA is busy operating in
 * Interactive mode. Stop all activities on the FPGA before requesting this
 * operation.
 */
static const nirio_status NiRio_Status_FpgaBusyInteractive = -61203;

/**
 * The operation could not be performed because the FPGA is busy operating in
 * Emulation mode. Stop all activities on the FPGA before requesting this
 * operation.
 */
static const nirio_status NiRio_Status_FpgaBusyEmulation = -61204;

/**
 * LabVIEW FPGA does not support the Reset method for bitfiles that allow
 * removal of implicit enable signals in single-cycle Timed Loops.
 */
static const nirio_status NiRio_Status_ResetCalledWithImplicitEnableRemoval = -61211;

/**
 * LabVIEW FPGA does not support the Abort method for bitfiles that allow
 * removal of implicit enable signals in single-cycle Timed Loops.
 */
static const nirio_status NiRio_Status_AbortCalledWithImplicitEnableRemoval = -61212;

/**
 * LabVIEW FPGA does not support Close and Reset if Last Reference for bitfiles
 * that allow removal of implicit enable signals in single-cycle Timed Loops.
 * Pass the NiFpga_CloseAttribute_NoResetIfLastSession attribute to NiFpga_Close
 * instead of 0.
 */
static const nirio_status NiRio_Status_CloseAndResetCalledWithImplicitEnableRemoval = -61213;

/**
 * For bitfiles that allow removal of implicit enable signals in single-cycle
 * Timed Loops, LabVIEW FPGA does not support this method prior to running the
 * bitfile.
 */
static const nirio_status NiRio_Status_ImplicitEnableRemovalButNotYetRun = -61214;

/**
 * Bitfiles that allow removal of implicit enable signals in single-cycle Timed
 * Loops can run only once. Download the bitfile again before re-running the VI.
 */
static const nirio_status NiRio_Status_RunAfterStoppedCalledWithImplicitEnableRemoval = -61215;

/**
 * A gated clock has violated the handshaking protocol. If you are using
 * external gated clocks, ensure that they follow the required clock gating
 * protocol. If you are generating your clocks internally, please contact
 * National Instruments Technical Support.
 */
static const nirio_status NiRio_Status_GatedClockHandshakingViolation = -61216;

/**
 * A session cannot be closed, reset, nor can a bitfile be downloaded while
 * DMA FIFO region references are still outstanding for the specified session.
 * Using Delete Data Value Reference, delete any regions acquired from Acquire
 * Read Region or Acquire Write Region first before taking any of these actions.
 */
static const nirio_status NiRio_Status_RegionsOutstandingForSession = -61217;

/**
 * The number of elements requested must be less than or equal to the number of
 * unacquired elements left in the host memory DMA FIFO. There are currently
 * fewer unacquired elements left in the FIFO than are being requested. Release
 * some acquired elements before acquiring more elements.
 */
static const nirio_status NiRio_Status_ElementsNotPermissibleToBeAcquired = -61219;

/**
 * An unexpected internal error occurred.
 */
static const nirio_status NiRio_Status_InternalError = -61499;

/**
 * Access to the remote system was denied. Use MAX to check the Remote Device
 * Access settings under Software>>NI-RIO>>NI-RIO Settings on the remote system.
 */
static const nirio_status NiRio_Status_AccessDenied = -63033;

/**
 * A connection could not be established to the specified remote device. Ensure
 * that the device is on and accessible over the network, that NI-RIO software
 * is installed, and that the RIO server is running and properly configured.
 */
static const nirio_status NiRio_Status_RpcConnectionError = -63040;

/**
 * The RPC session is invalid. The target may have reset or been rebooted. Check
 * the network connection and retry the operation.
 */
static const nirio_status NiRio_Status_RpcSessionError = -63043;

/**
 * The operation could not complete because another session is accessing the
 * FIFO. Close the other session and retry.
 */
static const nirio_status NiRio_Status_FifoReserved = -63082;

/**
 * A Configure FIFO, Stop FIFO, Read FIFO, or Write FIFO function was called
 * while the host had acquired elements of the FIFO. Release all acquired
 * elements before configuring, stopping, reading, or writing.
 */
static const nirio_status NiRio_Status_FifoElementsCurrentlyAcquired = -63083;

/**
 * A function was called using a misaligned address. The address must be a
 * multiple of the size of the datatype.
 */
static const nirio_status NiRio_Status_MisalignedAccess = -63084;

/**
 * A valid .lvbitx bitfile is required. If you are using a valid .lvbitx
 * bitfile, the bitfile may not be compatible with the software you are using.
 * Determine which version of LabVIEW was used to make the bitfile, update your
 * software to that version or later, and try again.
 */
static const nirio_status NiRio_Status_BitfileReadError = -63101;

/**
 * The specified signature does not match the signature of the bitfile. If the
 * bitfile has been recompiled, regenerate the C API and rebuild the
 * application.
 */
static const nirio_status NiRio_Status_SignatureMismatch = -63106;

/**
 * The bitfile you are trying to use is not compatible with the version of
 * NI-RIO installed on the target and/or the host. Determine which versions of
 * NI-RIO and LabVIEW were used to make the bitfile, update the software on the
 * target and host to that version or later, and try again.
 */
static const nirio_status NiRio_Status_IncompatibleBitfile = -63107;

/**
 * Either the supplied resource name is invalid as a RIO resource name, or the
 * device was not found. Use MAX to find the proper resource name for the
 * intended device.
 */
static const nirio_status NiRio_Status_InvalidResourceName = -63192;

/**
 * The requested feature is not supported.
 */
static const nirio_status NiRio_Status_FeatureNotSupported = -63193;

/**
 * The NI-RIO software on the target system is not compatible with this
 * software. Upgrade the NI-RIO software on the target system.
 */
static const nirio_status NiRio_Status_VersionMismatch = -63194;

/**
 * The session is invalid or has been closed.
 */
static const nirio_status NiRio_Status_InvalidSession = -63195;

/**
 * The maximum number of open FPGA sessions has been reached. Close some open
 * sessions.
 */
static const nirio_status NiRio_Status_OutOfHandles = -63198;

#endif /* STATUS_H_ */
