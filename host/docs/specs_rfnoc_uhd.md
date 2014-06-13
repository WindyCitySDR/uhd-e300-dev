vim: ft=markdown:
# 1. Feature Definition

These improvements/extensions to the UHD API will allow users to work with RFNoC blocks on the FPGA of a Generation 3 device (at the moment: X300, E300).
Given an FPGA image with RFNoC components and these changes, the user can connect components on the FPGA to suit their needs.

## 1.1 Definitions

* *Block*: In this context, a block is short for an RFNoC-Block, which is anything signal processing related connected to the crossbar. Blocks include custom signal processing components, such as FFTs, filters etc., but also the radios.
* *Port*: On the crossbar, this describes a locally connected block. Every crossbar on the FPGA has 16 ports (0 through 15), a certain number of which are available to connect computation engines (blocks).
* *Block port*: Because we are only using 4 bits to address blocks locally, every block has 4 bits left to address block ports. This means every block can have up to 16 different in- and outputs, which are named "block ports".
* *Transmission/Reception*: To avoid confusion, the host is always taken as reference. This means "transmission" (or tx) always describes data going from host to device, and "reception" or rx describes data coming from device to host, even if there is no actual radio transmission.
* *NoC-Shell*: A Verilog/FPGA structure to wrap user-generated FPGA code. See Section XXX


## 1.2 Problems and use cases

### 1.2.1 What is wrong with the current setup?

In its current state, the X300 implementation class is very much hard-coded around a specific setup,
which means there's two radios, and little else. Using RFNoC requires a whole arsenal of new commands,
such as:

* Access to block registers
* Access to the crossbar
* Querying SIDs of connections, and addresses of components (e.g. which SID is the controller for block 1 using, what's the address of block 2)

Another shortcoming is the existence of many hardcoded constants in the current code base.

Also, since some major changes are necessary to facilitate building RFNoC-based applications, it would be a good time to
change or clean up some other items.


#### 1.2.1.1 Backward compatibility

Maintaining backward compatibility will not always be possible, but careful design can go a long way towards
maintaining backward compatibility.

### 1.2.2 What is required of such a new API?

* Arbitrarily define channels to go to any endpoint
* The ability to create a streamer on any channel (doesn't have to go to radio)
* peek/poke settings registers on blocks
* Common settings must be accessible through simpler API calls than explicit register settings
* Must be extensible, so user-written blocks can have their own API calls (e.g. if the user implementes a filter with variable taps,
  allow them to have a function to set taps instead of forcing them to use pokes
* Debugging features:
** Print a list of all known SIDs, along with an identifier of who's using them

### 1.2.3 Use Case: Receiving through a filter block

An example of this would be the case where the signal coming from the radio is filtered first, on the device,
then sent to the host:

```
+-------+     +------+           +------+
|       | AXI |      |  Ethernet |      |
| Radio +-----> FIR  +-----------> Host |
|       |     |      |           |      |
+-------+     +------+           +------+
```

This requires the following steps:
* Configure radio to send to a specific address (not host)
* Run all necessary radio configuration steps (initialize clock, set frequency, ...)
* Set up flow control on radio and FIR filter block
* Open a rx stream from host to FIR
* Optional: Configure the FIR filter taps


### 1.2.4 Use Case: Welch Spectral Estimator

* On the device:
** FFT (Vector-based!)
** Mag-Square (Data type change!)
** Averaging (Block with history)

### 1.2.5 Use Case: 802.11a Receiver

* Entire demodulation done on the FPGA

### 1.2.6 Other use cases

* Regular streaming
* +MIMO
* Simple flow graphs on FPGA
* User-defined blocks, e.g. FFT -> User can change forward/reverse through `set_fft_direction()` on the controller class
* Multi-device


## 1.3 Additional Requirements

### 1.3.1 FPGA Changes / Crossbar

* Need to query ports
* Set which remote address maps to which output port (eth, pcie...)
* Static mapping local address -> port

### 1.3.2 FPGA Changes / NoC-Shell

* Defined registers to query buffer size, block type
* Make radio look like a special NoC-Shell
* Readback registers to query:
** Block type (this needs a special bit whether or not it's using the NoC-Shell)
** Input buffer size (for flow control)
** Input Data Type
** Output Data Type

### 1.3.2 FPGA Changes / General

* Consistent honoring of header bits (no categorization based on SID!)
** 00 Data, 01 Flow Control, 10 Command, 11 Command Response or Error (depends on EOB bit)

## 1.4 Limitations

* This design will allow to set up any topology of devices, but actual automatic setup of more than one device is not discussed here.

## 1.5 References

-

## 1.6 Action Items

(Currently all assigned to Martin).

* Implement RFNoC API


# 2 High-Level Description #################################################################

## 2.1 Device Initialization Phase

After power-up, the device needs to configure itself. All motherboard-related configuration is basically
unchanged. However, the configuration of the radios is replaced by the following sequence:

1. Query all ports on the crossbar to find out what type of block is connected
2. Create a block controller object for every block, and allocate a SID. This block controller object is of type
   `rfnoc_block` (see below), and is specialized towards the functionality of every block (i.e., it has different functions
   for a radio block than for an FFT block).
3. Every block's controller object extends the property tree

Setup of the radio is then done inside the block controller object for the radio. The same goes for other blocks
that need setting up (e.g., filter taps might need initializing, ...).

## 2.2 Accessing Blocks

For controlling purposes, it is necessary to allow users to access blocks for purposes
other than streaming. Two mechanisms will be made available:

1. Property Tree based control

To stay in tune with current UHD design, using the property tree is be the correct choice.
At initialization, every block controller object gets the opportunity to populate the property tree.

For the radios, this allows to maintain backward compabitility, as they can simply populate the property
tree in the same way they had so far. For other objects, they may populate the tree under the following
path:

    /mboards/MB_INDEX/rfnoc/PORT/

Where `MB_INDEX` is the board index (as before), and `PORT` is the local port number on the crossbar.
The following properties must be set:

    /block_type
    /input_type
    /output_type
    /input_buffer_size

*Note:* Radios must also populate these elements, although they may also populate other paths in the property tree!

2. Direct access to the block controller objects



However, it can be helpful to be able to access blocks directly. For this purpose

gen3_device::get_block(index)



## 2.3 Channel Definitions

*Channel definitions* replace subdevs, which then become obsolete. Since any block can potentially become
an endpoint for a streaming channel, subdevs are not adequate at describing RFNoC configurations.

Channel definitions map a channel index (an integer number) to a device endpoint, optionally specifying some arguments.
Unlike previous implementations, these arguments are not part of the endpoint address, but provided separately.

A typical channel definition could look like this:

Channel 0 -> Radio 0, Set gain to 11, frequency to 1.982 GHz, use antenna RX2
Channel 1 -> Radio 1, Set gain to 42, frequency to 2.1 GHz, use antenna RX2

Or like this:

Channel 0 -> FFT Block, Forward FFT, Set length to 64
Channel 1 -> OFDM Modulator, use FFT of length 128

Channels can be bidirectional, but only if the corresponding endpoint supports both modes.
For a radio, a single channel can be used to receive and transmit. For a filter, the tx and rx
directions can be used to loop back the data from this block and perform filtering on the device instead of the host.


### 2.3.1 Channel Definition Files

A channel definition file is in fact a long string (with newlines), typically stored in a file.
This file has a complete description of the internal configuration of the FPGA(s).

Channel definition files have a very simple syntax, no XML or fancy markup is required.

    channel 0 192.168.10.2/CE1/
        option1=value1
        option2=value2


Channels can be used as placeholders to configure blocks that will have no streamer connected.
This allows configuring *everything* in the channel definition file

## 2.X Convenience functions: connect

Given enough standardization, it is an easy thing to provide convenience functions to the user to
perform routine tasks.

connect():
* Set up destination SID on upstream block
* Query buffer size on downstream block and configure flow control on both blocks
* Register one block as downstream from the other. This way, the device has a representation of how blocks are connected.



## 2.X GNU Radio Integration

GNU Radio integration has a low priority, but with the suggested design, it would be fairly simple.
Changes required to make this work in Python or C++ GNU Radio applications (i.e., there still is only one UHD source or sink,
but it takes additional configuration to set up the crossbar for RFNoC):

* Create new UHD sinks/sources (or extend the existing ones) to read channel definitions instead of setting up the channels simply
  by setting the antenna. Using channel defintion files, existing GNU Radio apps can be re-run with different FPGA configurations
  by changing the channel definition files.

Changes required to make RFNoC available in GNU Radio Companion (this means the user can connect RFNoC blocks like GNU Radio blocks):

* Create a new GRC component which describes an `rfnoc_device`. This would have no in/out ports (e.g. like a Constellation object)
* Create some blocks which describe the functionality of RFNoC blocks (filter, FFT, converter... whatever we have available).
* Extend GRC to understand the connection type `rfnoc`.
* Blocks will then have streaming ports (which are connected to streamers) and rfnoc ports (which, when connected, use the connect()
  convenience function)


# 3 Detailed Description ###################################################################


## 3.X Required C++ components

*Note: All of the names are working titles!*

These are all C++ classes which need to be implemented.

### 3.X `uhd::rfnoc_device`

This derives from `uhd::device`, and extends it with the following public member functions:

* `rfnoc_block::sptr get_block_ctrl(mb_index, port)` -- Returns an sptr to the block controller object for block connected to a given port and device
* `rfnoc_block::sptr get_block_ctrl(channel_index)` -- Returns an sptr to the block controller object connected to a specific channel
* `connect(rfnoc_block::sptr block1, rfnoc_block::sptr block2)` -- Convenience function to connect block1 to block2


### 3.X Block controller classes

#### 3.X.1 `rfnoc_block` (implements `uhd::wb_iface`)

This is a class representing a block and comes with a functions to control it.

* `peek32()`
* `poke32()`
* `uint32_t get_address()` -- Returns 16-Bit address of this block
* `issue_stream_cmd()` -- Does nothing, to be overriden

#### 3.X.2 `nocshell_ctrl` (derives from `rfnoc_block`)

Overrides functions from `rfnoc_block` with sensible functionality:

* `issue_stream_cmd()` -- The default function is to do nothing, unless an upstream block is registered. In this case, the stream command is passed upstream.

Adds the following functions:

* `configure_upstream_flow_control(uint32 cycles, uint32 packets)`
** Description: Configure flow control for talking to the upstream block.
** `cycles`: Number of clock cycles between ACKs
** `packets`: Number of packets between ACKs
* `configure_downstream_flow_control(uint32 buf_size)`
** Description: Configure flow control for talking to the downstream block.
** `buf_size`: The buffer size of the downstream block, in number of packets. When set to zero, flow control is disabled.
* `set_destination_address(uint32_t address, int block_port=0)`
** Description: To configure SID, use this function to set destination address of stream.
** `address` is a 16-bit value which is used to set the SID on outgoing streams.
** `block_port` is a value in [0, 15] for which output this destination address is meant for.
* `set_nsamps_per_packet(size_t spp)`
** Description: Does nothing, to be overridden.



#### 3.X.3 `x300_radio_ctrl`, `e300_radio_ctrl` (derives from `rfnoc_block`)

Control the radio(s). As the name implies, this is actually different for different devices,
because the actual radio controls are different.

##### 3.X.3.1 `x300_radio_ctrl`

The `radio_perifs_t` structs are moved from `x300_impl` into this block.

Overrides:
* `set_nsamps_per_packet(spp)`: Sets this value in the `rx_vita_core_3000` object.

##### 3.X.3.1 `e300_radio_ctrl`

Overrides:
* `set_nsamps_per_packet(spp)`: Sets this value in the `rx_vita_core_3000` object.

#### 3.X.4 Other predefined controller objects (`null_source_ctrl`, derived from `nocshell_ctrl`)

As an example for developers, there will be some other block controllers for computation engines
that we will supply with the standard FPGA image.

#### 3.X.5 Registering own specialized block controller objects

When a developer writes their own computation engine, they can use a class of type `nocshell_ctrl`
to control their block from within an application, or they can derive from `nocshell_ctrl` to create
their own class that controls this block.

* Register blocks similarly to what we do with USRP devices
* Use `dynamic_cast` to turn the result of `get_block()` into your own block


#### 3.X.5 The block controller object instantiation process

1. Check if a specialized controller object was registered, if so, use that to make()
2. If not, check if it has a NoC-Shell bit set. If yes, use `nocshell_ctrl::make()`
3. As a last resort, use `rfnoc_block::make()`

## 3.X Changes to the streamer architecture

Only small things:
* `xxxx_packet_handler` gets function to query SID
** This is a small and simple change, very non-invasive and can go into master right now

