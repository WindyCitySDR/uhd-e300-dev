#!/bin/bash
#

### Helper functions ##########################################
function help {
	cat <<EOHELP

Usage: impact_jtag_programmer.sh [--help|-h] [--impact-path=<PATH>] --fpga=<FPGA Image File>

-h             - Shows this.
--impact-path  - Path to the iMPACT binary (if not already in PATH).
                 Often something like /opt/Xilinx/14.4/ISE_DS/ISE/bin/lin64/impact
--fpga-path    - Path to the FPGA image.

This script uses Xilinx iMPACT to reprogram the FPGA through the USB JTAG connector.
You can use this to unbrick USRP X3x0 devices.

EOHELP
}

### Go, go, go! ###############################################
echo "======================================="
echo " Copyright 2014 Ettus Research LLC"
echo ""
echo " JTAG Programming Tool"
echo "======================================="
echo ""

# Check for sudo
if [ ! $USER = root -a ! $UID -eq 2 ]
then
	echo Running without root or sudo privileges. If this doesn\'t work, try using \"sudo\".
	echo ""
fi

IMPACTPATH="impact" # This'll work if impact is in $PATH
FPGAIMG=""

# Go through cmd line options
for i in "$@"
do
case $i in
    -h|--help)
        help
        exit
        ;;
    --impact-path=*)
    IMPACTPATH="${i#*=}"
    ;;
    --fpga-path=*)
    FPGAIMG="${i#*=}"
    ;;
    *)
        echo Unrecognized option: $i
        echo
        help
        exit
        break
        ;;
esac
done

# Test impact binary is available
IMPACTEXECUTABLE=`which $IMPACTPATH`
if [ ! $? -eq 0 ]; then
    echo "ERROR: Cannot find 'impact' executable. Make sure you have iMPACT installed"
    echo "and that it is in your PATH, or use the --impact-path option to provide the"
    echo "location of the 'impact' executable."
    exit 1
fi

# Test the FPGA image file is readable
if [ -z $FPGAIMG ]; then
    echo "ERROR: No FPGA image file provided."
    exit 1
fi
if [ ! -r $FPGAIMG ]; then
    echo "ERROR: Can't read the FPGA image file ($FPGAIMG)."
    exit 1
fi

# Create batch file
CMD_PATH=`mktemp XXXXXXXX.impact.cmd`
echo "Generating impact batch file ${CMD_PATH}..."
echo "setmode -bscan" > ${CMD_PATH}
echo "setcable -p auto" >> ${CMD_PATH}
echo "addDevice -p 1 -file ${FPGAIMG}" >> ${CMD_PATH}
echo "program -p 1" >> ${CMD_PATH}
echo "quit" >> ${CMD_PATH}

# Run impact
echo "Running impact -- loading ${FPGAIMG} into the FPGA..."
${IMPACTEXECUTABLE} -batch ${CMD_PATH}
RETVAL=$?
if [ ! $RETVAL -eq 0 ]; then
    echo "ERROR: Programming failed. Check output above for hints. Maybe you forgot to use sudo?"
else
    echo "Programming complete!"
fi

# Remove batch file
rm $CMD_PATH
exit $RETVAL

# C'est tout
