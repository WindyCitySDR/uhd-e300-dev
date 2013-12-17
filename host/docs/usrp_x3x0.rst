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

--------------
Hardware Setup
--------------

^^^^^^^^^^^^^^^^
Gigabit Ethernet
^^^^^^^^^^^^^^^^

Installing the USRP X300/X310
:::::::::::::::::::::::::::::
* Prior to installing the module, the host PC can remain powered on.
* Plug a 1 Gigabit SFP+ Transciever into Ethernet Port 0 on the USRP X300/x310 device.
* Use the Ethernet cable to connect the SFP+ transciever on the device to the host computer. For maximum throughput, Ettus Research recommends that you connect each device to its own dedicated Gigabit Ethernet interface on the host computer.
* Connect the AC/DC power supply to the device and plug the supply into a wall outlet.
* The OS with automatically recognize the device.

^^^^^^^^^^^^^^^^^^^^
Ten Gigabit Ethernet
^^^^^^^^^^^^^^^^^^^^

Installing the Host Ethernet Interface
::::::::::::::::::::::::::::::::::::::
Ettus Research recommends the Intel Ethernet Converged Network Adapter X520-DA2 interface for communication with the USRP X300/X310 device.
Installation instructions for this interface are available on the official Intel website.

Installing the USRP X300/X310
:::::::::::::::::::::::::::::
* Prior to installing the module, the host PC can remain powered on.
* Use a 10 Gigabit SFP+ cable to connect Ethernet Port 1 on the USRP X300/x310 device to the host computer. For maximum throughput, Ettus Research recommends that you connect the device to its own dedicated Ten Gigabit, Ettus Research recommended Ethernet interface on the host computer.
* Connect the AC/DC power supply to the device and plug the supply into a wall outlet.
* The OS with automatically recognize the device.

The LEDs on the front panel can be useful in debugging hardware and software issues.
The LEDs reveal the following about the state of the device:

^^^^^^^^^^^^^^^^^^^^^
PCI Express (Desktop)
^^^^^^^^^^^^^^^^^^^^^
Installing the PCI Express Interface Kit
::::::::::::::::::::::::::::::::::::::::
Follow the instructions listed in the `Set Up Your MXI-Express x4 System <http://www.ni.com/pdf/manuals/371976c.pdf>`_ 
document to setup the NI PCIe-8371 module.

Installing the USRP X300/X310
:::::::::::::::::::::::::::::
* Prior to installing the module, make sure that the PC is powered off.
* Using a MXI-Express Cable connect the USRP X300/X310 to the NI PCIe-8371.
* Connect the AC/DC power supply to the device and plug the supply into a wall outlet.
* Power on the USRP X300/X310 device using the power switch located in the bottom-right corner of the front panel.
* Power on the PC (The OS automatically recognizes the new device)

NOTE: The USRP device is not hot-pluggable over PCI Express. Any connection changes with only be detected by your 
computer after a successful reboot.

Troubleshooting
:::::::::::::::
If your computer does not boot when connected to your PXI chassis through MXI-Express, or if Windows does not 
properly discover your devices. (For example, there is a yellow exclamation point on a PCI to PCI bridge in 
Windows Device Manager, despite drivers for all devices being installed.) These situations often are due to 
programming errors in PCI Express device configuration of the BIOS. To use this software, you need a MXI-Express 
device that supports Mode 1 operation. 
Refer to `NI MXI-Express BIOS Compatibility Software Readme <http://download.ni.com/support/softlib//PXI/MXIe%20Compatibility%20Software/1.5.0/readme.html#SupportedHardware>`_ 
for more information.

The BIOS Compatibility Software can be downloaded for Windows from the `MXI-Express BIOS Compatibility Software <http://www.ni.com/download/mxi-express-bios-compatibility-software-1.5/3764/en/>`_ page

^^^^^^^^^^^^^^^^^^^^
PCI Express (Laptop)
^^^^^^^^^^^^^^^^^^^^
Installing the PCI Express Card
:::::::::::::::::::::::::::::::
Follow the instructions listed in the “Installing an NI ExpressCard-8360 Host Card” section of the 
`Set Up Your MXI-Express x1 System <http://www.ni.com/pdf/manuals/373259d.pdf#page=10>`_ 
document to setup the NI ExpressCard-8360B module.

Installing the USRP X300/X310
:::::::::::::::::::::::::::::
Because a laptop computer is not grounded, follow this procedure to safely connect a laptop
computer to your USRP device.

* Connect the AC/DC power supply to the device and plug the supply into a wall outlet. Ensure that the USRP device is powered off
* Touch the NI ExpressCard-8360B and a metal part of the USRP device simultaneously. Do not install the NI ExpressCard-8360B into the laptop computer yet.
* Connect the cable to the NI ExpressCard-8360B and USRP.
* Plug the NI ExpressCard-8360B into an available ExpressCard slot. If your laptop computer is already running (or hibernating, suspended, etc) when you install an NI ExpressCard-8360B, you must reboot to detect the USRP. Otherwise, the USRP is detected when you start your computer.

NOTE: The USRP device is not hot-pluggable over PCI Express. Any connection changes will only be detected by your computer after a successful reboot.

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

---------------------
Addressing the Device
---------------------

^^^^^^^^^^^^^^^^^^^^^^^^^^^
Single device configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^
In a single-device configuration,
the USRP device must have a unique IPv4 address on the host computer.
The USRP can be identified through its IPv4 address, resolvable hostname, NI-RIO resource name or by other means.
See the application notes on `device identification <./identification.html>`_.
Use this addressing scheme with the **single_usrp** interface.

Example device address string representation for a USRP-X Series device with IPv4 address **192.168.10.2**:

::

    addr=192.168.10.2

Example device address string representation for a USRP-X Series device with RIO resource name **RIO0** over PCI Express:

::

    resource=RIO0

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

Example device address string representation for 2 USRPs with IPv4 addresses **192.168.10.2** and **192.168.20.2**:

::

    addr0=192.168.10.2, addr1=192.168.20.2


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

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
USRP RIO device not enumerated (Linux)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
UHD requires the RIO device manager service to be running in order to communicate with any USRP RIO device.
This service is installed as a part of the USRP RIO (or NI-USRP) installer. On Linux, the service
is not started at system boot time. To start it, run the following command:

::

    sudo /etc/init.d/niusrpriorpc start

If the device still does not enumerate after starting the device manager, make sure that the host computer
has successfully detected it. You can do so by running the following command:

::

   lspci -k -d 1093:c4c4

A device similar to the following should be detected:

::

   $ lspci -k -d 1093:c4c4
   04:00.0 Signal processing controller: National Instruments ...
           Subsystem: National Instruments Device 76ca
           Kernel driver in use: niusrpriok_shipped

* A USRP X300 should appear with 'Subsystem: National Instruments Device 7736'
* A USRP X310 should appear with 'Subsystem: National Instruments Device 76ca'

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
USRP RIO device not enumerated (Windows)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
UHD requires the RIO device manager service to be running in order to communicate with any USRP RIO device.
This service is installed as a part of the USRP RIO (or NI-USRP) installer. On Windows, it can be found in
the **Services** section in the Control Panel and it is started at system boot time. To ensure that the 
service is indeed started, navigate to the Services tag in the Windows Task Manager and ensure that the 
status of **niusrpriorpc** is "Running" 

If the device still does not enumerate after starting the device manager, make sure that the host computer
has successfully detected it. You can do so by checking if your device shows up in the Windows Device Manager

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Monitor the host network traffic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use Wireshark to monitor packets sent to and received from the device.

--------------
Hardware Notes
--------------

^^^^^^^^^^^
Front Panel
^^^^^^^^^^^

.. image:: ./res/x3x0_fp_overlay.png
   :scale: 80%
   :align: left

* **JTAG**: USB connector for the on-board USB-JTAG programmer
* **RF A Group**

  * **TX/RX LED**: Indicates that data is streaming on the TX/RX channel on daughter board A
  * **RX2 LED**: Indicates that data is streaming on the RX2 channel on daughter board A

* **REF**: Indicates that the external Reference Clock is locked
* **PPS**: Indicates a valid PPS signal by pulsing once per second
* **AUX IO**: Front panel GPIO connector.
* **GPS**: Indicates that GPS reference is locked
* **LINK**: Indicates that the host computer is communicating with the device

* **RF B Group**

  * **TX/RX LED**: Indicates that data is streaming on the TX/RX channel on daughter board B
  * **RX2 LED**: Indicates that data is streaming on the RX2 channel on daughter board B

* **PWR**: Power switch

^^^^^^^^^^
Rear Panel
^^^^^^^^^^
   
.. image:: ./res/x3x0_rp_overlay.png
   :scale: 80%
   :align: left
   

* **PWR**: Connector for the USRP-X Series power supply
* **1G/10G ETH**: SFP+ ports for Ethernet interfaces
* **REF OUT**: Output port for the exported reference clock
* **REF IN**: Reference clock input
* **PCIe x4**: Connector for Cabled PCI Express link
* **PPS/TRIG OUT**: Output port for the PPS signal
* **PPS/TRIG IN**: Input port for the PPS signal 
* **GPS**: Connection for the GPS antenna

^^^^^^^^^^^^^^^^^
Ref Clock - 10MHz
^^^^^^^^^^^^^^^^^
Using an external 10MHz reference clock, a square wave will offer the best phase
noise performance, but a sinusoid is acceptable.  The power level of the reference clock cannot exceed +15dBm.

^^^^^^^^^^^^^^^^^^^^^^
PPS - Pulse Per Second
^^^^^^^^^^^^^^^^^^^^^^
Using a PPS signal for timestamp synchronization requires a square wave signal with the following a 5Vpp amplitude.

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

^^^^^^^^^^^^^^^^
Front Panel GPIO
^^^^^^^^^^^^^^^^

Connector
:::::::::

.. image:: ./res/x3x0_gpio_conn.png
   :scale: 25%
   :align: left

Pin Mapping
:::::::::::

* Pin 1:  +3.3V
* Pin 2:  Data[0]
* Pin 3:  Data[1]
* Pin 4:  Data[2]
* Pin 5:  Data[3]
* Pin 6:  Data[4]
* Pin 7:  Data[5]
* Pin 8:  Data[6]
* Pin 9:  Data[7]
* Pin 10: Data[8]
* Pin 11: Data[9]
* Pin 12: Data[10]
* Pin 13: Data[11]
* Pin 14: 0V
* Pin 15: 0V


Please see the `GPIO API Notes <./gpio_api.html>`_ for information on configuring and using the GPIO bus.

-------------
Miscellaneous
-------------

^^^^^^^^^^^^^^^^^^^^
Multiple RX channels
^^^^^^^^^^^^^^^^^^^^
There are two complete DDC and DUC DSP chains in the FPGA. In the single channel case, 
only one chain is ever used. To receive from both channels, the user must set the **RX** or **TX**
subdevice specification.

In the following example, a TVRX2 is installed.
Channel 0 is sourced from subdevice **RX1**,
and channel 1 is sourced from subdevice **RX2**:

::

    usrp->set_rx_subdev_spec("A:RX1 A:RX2");


^^^^^^^^^^^^^^^^^
Available Sensors
^^^^^^^^^^^^^^^^^
The following sensors are available for the USRP-X Series motherboards;
they can be queried through the API.

* **ref_locked** - clock reference locked (internal/external)
* Other sensors are added when the GPSDO is enabled
    