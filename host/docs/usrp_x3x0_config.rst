========================================================================
UHD - System Configuration for USRP X3x0 Series
========================================================================

.. contents:: Table of Contents

------------------------------------------------------------------------
Configuring your Host PC
------------------------------------------------------------------------

The USRP X3x0 is capable of delivering very fast sample rates to the host PC,
and even high-powered desktops can have trouble keeping up at the higher rates.
You can improve the performance of your host by configuring a number of
settings that affect the performance of your computer.

These are:

 * Kernel Version
 * Real-Time Scheduling
 * Building with ORC & Volk
 * Network Configuration
 * Power Management Configuration

These items are covered in more detail, below.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Kernel Version
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Performance issues may be encountered for Linux kernels earlier than 3.11.  Ettus Research strongly recommends using version 3.11 or higher for high sample rates.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Real-Time Scheduling
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Building with ORC & Volk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Network Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
When using Ethernet interfaces to communicate with the device, it is necessary to configure the maximum socket buffer sizes and the MTU size.

Configuring the Socket Buffers
-------------------------------------
It is necessary to increase the maximum size of the socket buffers to avoid potential overflows and underruns at high sample rates.  Add the following entries into /etc/sysctl.conf (root privileges required):
::

	net.core.rmem_max=33554432
	net.core.wmem_max=33554432

Either restart the system or issue the following commands:
::

	sudo sysctl net.core.rmem_max=33554432
	sudo sysctl net.core.wmem_max=33554432


Configuring the MTU
-------------------------------------
UHD uses large frame sizes over UDP, which requires a larger than normal MTU size.  The default MTU size for most Ethernet cards is 1500.  UHD uses an MTU size of 9000 by default.  Nework hardware may or may not support higher MTU sizes, so the manufacturer's documentation for any network hardware should be consulted to see if it supports larger MTU sizes.  To set the MTU size of the interface to 9000:
::

	sudo ifconfig <interface> mtu 9000

If your network hardware is limited to an MTU of 1500, the UHD frame size can be reduced by adding the argument "<send/recv>_frame_size=1472".  However, higher sample rates will not be possible.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Power Management
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Power management on the host system attempts to save power by reducing clock frequencies or even powering off devices while not in use.  This can lead to significant performance issues when trying to operate at high sample rates.  Ettus strongly recommends disabling all power management.


Setting the CPU Governors
-------------------------------------
In Linux, the CPU governors dictate the frequency at which the CPU operates and attempt to reduce the CPU frequencies at certain times to save power.  When running at high sample rates, reduction of CPU frequencies can cause significant performance issues.  To prevent those issues, set the governor to "performance".

**Ubuntu:**
1. Install cpufrequtils:

::

	sudo apt-get install cpufrequtils

2. Edit /etc/init.d/cpufrequtils and set GOVERNOR="performance" on the appropriate line (run as root):

::

	sed s/^GOVERNOR=.*$/GOVERNOR=\"performance\"/g /etc/init.d/cpufrequtils > /etc/init.d/cpufrequtils

3. Restart cpufrequtils:

::

	sudo /etc/init.d/cpufrequtils restart

**Fedora:**
::

	sudo cpupower frequency-set -g performance

------------------------------------------------------------------------
Host PC Hardware Selection
------------------------------------------------------------------------
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Motherboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Testing has shown that some motherboards do not provide enough PCIe bus bandwidth to support higher sample rates.  Motherboards with PCIe 3.0 are required and the PCIe architecture of the motherboard should be carefully considered.  Slots with dedicated PCIe lanes should be used for PCIe or 10GbE cards that will be connected to the X3x0 device.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
10GbE NIC
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Intel or Myricom 10GbE NICs are recommeded.  Mellanox, SolarFlare, and Chelsio 10GbE NICs are not currently supported.  The Ethernet card should be plugged into the slot that has the most direct connection with the CPU (PCIe lanes are not shared with another slot).  Refer to the motherboard manual for more information on PCIe architecture.

------------------------------------------------------------------------
Troubleshooting Performance Issues
------------------------------------------------------------------------
The output on the host console provides indicators of performance issues in the form of single upper-case letters.  The following table lists the letters, their meanings, and possible causes:

========= ====================== ====================================================================
Indicator Meaning                Possible Causes
========= ====================== ====================================================================
O         Overflow on RX         - Data is not being consumed by user's application fast enough.
                                 - CPU governor or other power management not configured correctly.
D         Dropped packet on RX   - Network hardware failure.  (Check host NIC, cable, switch, etc...)
                                 - PCIe bus on host cannot sustain throughput. (Check ethtool -S <interface>).
                                 - CPU governor or other power management not configured correctly.
U         Underflow on TX        - Samples are not being produced by user's application fast enough.
                                 - CPU governor or other power management not configured correctly.
L         Late packet            - Samples are not being produced by user's application fast enough.
          (usually on MIMO TX)   - CPU governor or other power management not configured correctly.
                                 - Incorrect/invalid time_spec provided.
S         Sequence error on TX   - Network hardware failure.  (Check host NIC, cable, switch, etc...)
========= ====================== ====================================================================

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Troubleshooting Ethernet Issues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. First, check 'ifconfig <interface>' to see if there are any errors reported on the interface.  If there are errors, it is most likely a network hardware problem.
2. Next, check the output of 'ethtool -S <interface>'.  The output is driver-specific, but may give important clues as to what may be happening.  For example, a high value on rx_missed_errors for an Intel NIC indicates that the bus (i.e. PCIe) is not keeping up.
3. Finally, Wireshark can be used to validate the traffic between the host and device and make sure there is no unwanted traffic on the interface.

