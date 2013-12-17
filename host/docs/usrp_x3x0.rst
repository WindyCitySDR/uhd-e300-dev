===============================
UHD - X3x0 Series Device Manual
===============================

.. contents:: Table of Contents

-------------------------
Comparative features list
-------------------------

**Hardware Capabilities:**
 * 2 transceiver card slots
 * Dual SFP+ Transceivers (can be used with 1 GigE, 10 GigE, or CPRI)
 * PCI Express over cable (MXI) gen1 x4
 * External PPS input & output
 * External 10MHz input & output
 * Expandable via 2nd SFP+ interface
 * Supported master clock rates: 200 MHz, 184.32 MHz
 * External GPIO Connector with UHD API control
 * External USB Connection for built-in JTAG debugger
 * Internal GPSDO option

**FPGA Capabilities:**
 * 2 RX DDC chains in FPGA
 * 2 TX DUC chain in FPGA
 * Timed commands in FPGA
 * Timed sampling in FPGA
 * sc8 and sc16 sample modes
 * Up to 200 MHz of RF BW with 16-bit samples

Hardware setup
LED indications
Ref clock and 1 PPS
Using GPIO expansion
Using internal GPSDO 

Burning FPGA images
1/10 GigE
PCI Express and ExpressCard
JTAG (for FPGA devel)
1 GigE setup
10 Gig,)E setup
PCI Express setup
Express card setup
Addressing 
Alternate Stream Dest

--------------------------------
Load FPGA Images onto the Device
--------------------------------
The USRP-X Series device ships with a bitstream pre-programmed in the flash, and it is loaded 
onto the FPGA during device power-up. However, a new FPGA image can be configured over the 
PCI Express interface or the on-board USB-JTAG programmer.

^^^^^^^^^^^^^^^^^^
FPGA Image Flavors
^^^^^^^^^^^^^^^^^^
The USRP-X Series devices contains two SPF+ port for the two Ethernet channels. Because the 
SFP+ ports support both 1 Gigabit and 10 Gigabit transcievers, multiple the FPGA images are 
shipped with UHD to determine the behavior of the above interfaces.

+---------------------+------------------------+------------------------+
|  FPGA Image Flavor  |  SFP+ Port 0 Interface |  SFP+ Port 1 Interface |
+=====================+========================+========================+
|  HGS (Default)      |  1 Gigabit Ethernet    |  10 Gigabit Ethernet   |
+---------------------+------------------------+------------------------+
|  XGS                |  10 Gigabit Ethernet   |  10 Gigabit Ethernet   |
+---------------------+------------------------+------------------------+

FPGA images are shipped in 2 formats:

* **LVBITX**: LabVIEW FPGA configuration bitstream format (for use over PCI Express)
* **BIT**: Xilinx configuration bitstream format (for use over JTAG)

To get the latest images, simply use the uhd_images_downloader script:

**UNIX:**

::

    <install-path>/share/uhd/utils/uhd_images_downloader.py

**Windows:**

::

    <path_to_python.exe> <install-path>/share/uhd/utils/uhd_images_downloader.py


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use PCI Express to load FPGA images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
UHD requires a valid LabVIEW FPGA configuration bitstream file (LVBITX) to use the USRP-X Series
device over the PCI Express bus. LabVIEW FPGA is **NOT** required to use UHD with a USRP-X Series device.
Because FPGA configuration is a part of normal operation over PCI Express, there is no setup required
before running UHD.

The **fpga** tag can be set in the optional device args passed to indicate the FPGA image flavor to UHD.
If the above tag is speficied, UHD will attempt to load the FPGA image with the requested flavor from the
UHD images directory. If the tag is not specified, UHD will automatically detect the flavor of the image
and attempt to load the corresponding configuration bitstream onto the device. Note that if UHD detects
that the requested image is already loaded onto the FPGA then it will not reload it. 

^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use JTAG to load FPGA images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The USRP-X Series device features an on-board USB-JTAG programmer that can be accessed on the front-panel
of the device. The iMPACT tool in the `Xilinx Programming Tools <http://www.xilinx.com/support/download/index.htm>`_ package can be used to load an image over
the JTAG interface.

---------------------------------------
Load the Images onto the On-board Flash
---------------------------------------
The USRP-X Series device can be reprogrammed over the network or PCI Express to update or change the FPGA image.

**Note:**
Different hardware revisions require different FPGA images.
Determine the revision number from the sticker on the rear of the chassis.
Use this number to select the correct FPGA image for your device.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use the burner tool over Ethernet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**UNIX:**

::

    <install-path>/share/uhd/utils/usrp_x3xx_burner --addr=<ip address> --fpga-path=<path to FPGA image>

**Windows:**

::

    <install-path>\share\uhd\utils\usrp_x3xx_burner.exe --addr=<ip address> --fpga-path=<path to FPGA image>

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use the burner tool over PCI Express
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**UNIX:**

::

    <install-path>/share/uhd/utils/usrp_x3xx_burner --resource=<device resource name> --fpga-path=<path to FPGA image>

**Windows:**

::

    <install-path>\share\uhd\utils\usrp_x3xx_burner.exe --resource=<device resource name> --fpga-path=<path to FPGA image>

^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Device recovery and bricking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Its possible to put the device into an unusable state by loading bad images.
Fortunately, the USRP-X Series device can be loaded with a good image temporarily using the USB-JTAG interface. 
Once booted into the safe image, the user can once again load images onto the device over Ethernet or PCI Express.

----------------
Setup Networking
----------------
The USRP-X Series only supports Gigabit and Ten Gigabit Ethernet and will not work with a 10/100 Mbps interface.
However, a 10/100 Mbps interface can be connected indirectly to a USRP-X through a Gigabit Ethernet switch.

^^^^^^^^^^^^^^^^^^^^^^^^
Setup the host interface
^^^^^^^^^^^^^^^^^^^^^^^^
The USRP-X Series communicates at the IP/UDP layer over the gigabit and ten gigabit ethernet.
The default IP address for the USRP X300/X310 device depends on the Ethernet Port and interface used. 
You must configure the host Ethernet interface with a static IP address on the same subnet as the connected 
device to enable communication, as shown in the following table:

+---------------+-------------------------+----------------+----------------+---------------+
|  Ethernet     | USRP                    |  Default USRP  |  Host Static   | Host Static   |
| Interface     | Ethernet Port           | IP Address     | IP Address     | Subnet Mask   |
+===============+=========================+================+================+===============+
|  Gigabit      |  Port 0 (HGS Image)     |  192.168.10.2  | 192.168.10.1   | 255.255.255.0 |
+---------------+-------------------------+----------------+----------------+---------------+
|  Ten Gigabit  |  Port 1 (HGS/XGS Image) |  192.168.40.2  | 192.168.40.1   | 255.255.255.0 |
+---------------+-------------------------+----------------+----------------+---------------+
|  Ten Gigabit  |  Port 0 (XGS Image)     |  192.168.30.2  | 192.168.30.1   | 255.255.255.0 |
+---------------+-------------------------+----------------+----------------+---------------+


On a Linux system, you can set a static IP address very easily by using the
'ifconfig' command

::

    sudo ifconfig <interface> 192.168.10.1

Note that **<interface>** is usually something like **eth0**.  You can discover the
names of the network interfaces in your computer by running **ifconfig** without
any parameters:

::

    ifconfig -a

**Note:**
When using UHD software, if an IP address for the USRP-X Series device is not specified,
the software will use UDP broadcast packets to locate the USRP-X Series device.
On some systems, the firewall will block UDP broadcast packets.
It is recommended that you change or disable your firewall settings.

^^^^^^^^^^^^^^^^^^^^^^^^^
Multiple devices per host
^^^^^^^^^^^^^^^^^^^^^^^^^
For maximum throughput, one Ethernet interface per USRP is recommended,
although multiple devices may be connected via an Ethernet switch.
In any case, each Ethernet interface should have its own subnet,
and the corresponding USRP device should be assigned an address in that subnet.
Example:

**Configuration for USRP-X Series device 0:**

* Ethernet interface IPv4 address: **192.168.10.1**
* Ethernet interface subnet mask: **255.255.255.0**
* USRP-X Series device IPv4 address: **192.168.10.2**

**Configuration for USRP-X Series device 1:**

* Ethernet interface IPv4 address: **192.168.110.1**
* Ethernet interface subnet mask: **255.255.255.0**
* USRP-X Series device IPv4 address: **192.168.110.2**

^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Change the USRP's IP address
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You may need to change the USRP's IP address for several reasons:

* to satisfy your particular network configuration
* to use multiple USRP-X Series devices on the same host computer
* to set a known IP address into USRP (in case you forgot)

To change the USRP's IP address,
you must know the current address of the USRP,
and the network must be setup properly as described above.
Run the following commands:

**UNIX:**

::

    cd <install-path>/share/uhd/utils
    ./usrp_burn_mb_eeprom --args=<optional device args> --key=ip-addr --val=192.168.10.3

**Windows:**

::

    cd <install-path>\share\uhd\utils
    usrp_burn_mb_eeprom.exe --args=<optional device args> --key=ip-addr --val=192.168.10.3

----------------------
Communication Problems
----------------------
When setting up a development machine for the first time,
you may have various difficulties communicating with the USRP device.
The following tips are designed to help narrow down and diagnose the problem.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RuntimeError: no control response
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is a common error that occurs when you have set the subnet of your network
interface to a different subnet than the network interface of the USRP device.  For
example, if your network interface is set to **192.168.20.1**, and the USRP device is
**192.168.10.2** (note the difference in the third numbers of the IP addresses), you
will likely see a 'no control response' error message.

Fixing this is simple - just set the your host PC's IP address to the same
subnet as that of your USRP device. Instructions for setting your IP address are in the
previous section of this documentation.


^^^^^^^^^^^^^^^
Firewall issues
^^^^^^^^^^^^^^^
When the IP address is not specified,
the device discovery broadcasts UDP packets from each ethernet interface.
Many firewalls will block the replies to these broadcast packets.
If disabling your system's firewall
or specifying the IP address yields a discovered device,
then your firewall may be blocking replies to UDP broadcast packets.
If this is the case, we recommend that you disable the firewall
or create a rule to allow all incoming packets with UDP source port **49152**.

^^^^^^^^^^^^^^^
Ping the device
^^^^^^^^^^^^^^^
The USRP device will reply to ICMP echo requests.
A successful ping response means that the device has booted properly
and that it is using the expected IP address.

::

    ping 192.168.10.2

^^^^^^^^^^^^^^^^^^^^^^^^^
Monitor the serial output
^^^^^^^^^^^^^^^^^^^^^^^^^
Read the serial port to get debug verbose output from the embedded microcontroller.
The microcontroller prints useful information about IP addresses,
MAC addresses, control packets, fast-path settings, and bootloading.
Use a standard USB to 3.3v-level serial converter at 230400 baud.
Connect **GND** to the converter ground, and connect **TXD** to the converter receive.
The **RXD** pin can be left unconnected as this is only a one-way communication.

* **USRP2:** Serial port located on the rear edge
* **N210:** Serial port located on the left side

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Monitor the host network traffic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use Wireshark to monitor packets sent to and received from the device.

---------------------
Addressing the Device
---------------------

^^^^^^^^^^^^^^^^^^^^^^^^^^^
Single device configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^
In a single-device configuration,
the USRP device must have a unique IPv4 address on the host computer.
The USRP can be identified through its IPv4 address, resolvable hostname, or by other means.
See the application notes on `device identification <./identification.html>`_.
Use this addressing scheme with the **single_usrp** interface.

Example device address string representation for a USRP2 with IPv4 address **192.168.10.2**:

::

    addr=192.168.10.2

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Multiple device configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In a multi-device configuration,
each USRP device must have a unique IPv4 address on the host computer.
The device address parameter keys must be suffixed with the device index.
Each parameter key should be of the format <key><index>.
Use this addressing scheme with the **multi_usrp** interface.

* The order in which devices are indexed corresponds to the indexing of the transmit and receive channels.
* The key indexing provides the same granularity of device identification as in the single device case.

Example device address string representation for 2 USRP2s with IPv4 addresses **192.168.10.2** and **192.168.20.2**:
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    addr0=192.168.10.2, addr1=192.168.20.2

--------------------
Using the MIMO Cable
--------------------
The MIMO cable allows two USRP devices to share reference clocks,
time synchronization, and the Ethernet interface.
One of the devices will sync its clock and time references to the MIMO cable.
This device will be referred to as the slave, and the other device, the master.

* The slave device acquires the clock and time references from the master device.
* The master and slave may be used individually or in a multi-device configuration.
* External clocking is optional and should only be supplied to the master device.

^^^^^^^^^^^^^^^^^^^^
Shared ethernet mode
^^^^^^^^^^^^^^^^^^^^
In shared Ethernet mode,
only one device in the configuration can be attached to the Ethernet.

* Clock reference, time reference, and data are communicated over the MIMO cable.
* Master and slave must have different IPv4 addresses in the same subnet.

^^^^^^^^^^^^^^^^^^
Dual ethernet mode
^^^^^^^^^^^^^^^^^^
In dual Ethernet mode,
both devices in the configuration must be attached to the Ethernet.

* Only clock reference and time reference are communicated over the MIMO cable.
* The master and slave must have different IPv4 addresses in different subnets.

^^^^^^^^^^^^^^^^^^^^^
Configuring the slave
^^^^^^^^^^^^^^^^^^^^^
In order for the slave to synchronize to the master over MIMO cable,
the following clock configuration must be set on the slave device:
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    usrp->set_time_source("mimo", slave_index);
    usrp->set_clock_source("mimo", slave_index);


------------------------------
Alternative stream destination
------------------------------
It is possible to program the USRP device to send RX packets to an alternative IP/UDP destination.

^^^^^^^^^^^^^^^^^^^^^^^^^^
Set the subnet and gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^
To use an alternative streaming destination,
the device needs to be able to determine if the destination address
is within its subnet, and ARP appropriately.
Therefore, the user should ensure that subnet and gateway addresses
have been programmed into the device's EEPROM.

Run the following commands:
:::::::::::::::::::::::::::

    cd <install-path>/share/uhd/utils
    ./usrp_burn_mb_eeprom --args=<optional device args> --key=subnet --val=255.255.255.0
    ./usrp_burn_mb_eeprom --args=<optional device args> --key=gateway --val=192.168.10.1

^^^^^^^^^^^^^^^^^^^^^^^^^
Create a receive streamer
^^^^^^^^^^^^^^^^^^^^^^^^^
Set the stream args "addr" and "port" values to the alternative destination.
Packets will be sent to this destination when the user issues a stream command.

::

    //create a receive streamer, host type does not matter
    uhd::stream_args_t stream_args("fc32");

    //resolvable address and port for a remote udp socket
    stream_args.args["addr"] = "192.168.10.42";
    stream_args.args["port"] = "12345";

    //create the streamer
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //issue stream command
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = true;
    usrp->issue_stream_cmd(stream_cmd);

**Note:**
Calling recv() on this streamer object should yield a timeout.

--------------------
Hardware Setup Notes
--------------------

^^^^^^^^^^^^^^^^
Front panel LEDs
^^^^^^^^^^^^^^^^
The LEDs on the front panel can be useful in debugging hardware and software issues.
The LEDs reveal the following about the state of the device:

* **LED A:** transmitting
* **LED B:** mimo cable link
* **LED C:** receiving
* **LED D:** firmware loaded
* **LED E:** reference lock
* **LED F:** CPLD loaded


^^^^^^^^^^^^^^^^^
Ref Clock - 10MHz
^^^^^^^^^^^^^^^^^
Using an external 10MHz reference clock, a square wave will offer the best phase
noise performance, but a sinusoid is acceptable.  The reference clock requires the following power level:

* **USRP2** 5 to 15dBm
* **N2XX** 0 to 15dBm


^^^^^^^^^^^^^^^^^^^^^^
PPS - Pulse Per Second
^^^^^^^^^^^^^^^^^^^^^^
Using a PPS signal for timestamp synchronization requires a square wave signal with the following amplitude:

* **USRP2** 5Vpp
* **N2XX** 3.3 to 5Vpp

Test the PPS input with the following app:

* **<args>** are device address arguments (optional if only one USRP device is on your machine)

::

    cd <install-path>/share/uhd/examples
    ./test_pps_input --args=<args>

^^^^^^^^^^^^^^
Internal GPSDO
^^^^^^^^^^^^^^
Please see the `Internal GPSDO Application Notes <./gpsdo.html>`_
for information on configuring and using the internal GPSDO.

-------------
Miscellaneous
-------------

^^^^^^^^^^^^^^^^^
Available Sensors
^^^^^^^^^^^^^^^^^
The following sensors are available for the USRP2/N-Series motherboards;
they can be queried through the API.

* **mimo_locked** - clock reference locked over the MIMO cable
* **ref_locked** - clock reference locked (internal/external)
* other sensors are added when the GPSDO is enabled

^^^^^^^^^^^^^^^^^^^^
Multiple RX channels
^^^^^^^^^^^^^^^^^^^^
There are two complete DDC chains in the FPGA.
In the single channel case, only one chain is ever used.
To receive from both channels,
the user must set the **RX** subdevice specification.
This hardware has only one daughterboard slot,
which has been aptly named slot **A**.

In the following example, a TVRX2 is installed.
Channel 0 is sourced from subdevice **RX1**,
and channel 1 is sourced from subdevice **RX2**:
::::::::::::::::::::::::::::::::::::::::::::::::

    usrp->set_rx_subdev_spec("A:RX1 A:RX2");
