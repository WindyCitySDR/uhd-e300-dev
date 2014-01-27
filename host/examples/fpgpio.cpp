//
// Copyright 2010-2011 Ettus Research LLC
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

// Example for front panel GPIO.
// Bits are set as follows:
// FPGPIO[0] = ATR output 1 at idle
// FPGPIO[1] = ATR output 1 during RX
// FPGPIO[2] = ATR output 1 during TX
// FPGPIO[3] = ATR output 1 during full duplex
// FPGPIO[4] = output
// FPGPIO[5] = input
// FPGPIO[6] = input
// FPGPIO[7] = input
// FPGPIO[8] = input
// FPGPIO[9] = input
// FPGPIO[10] = input
// The example cycles through idle, TX, RX, and full duplex, spending 2 seconds for each.
// Outputs can be physically looped back to inputs for verification testing.

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/convert.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <csignal>
#include <iostream>

#define FPGPIO_DEFAULT_CPU_FORMAT   "fc32"
#define FPGPIO_DEFAULT_OTW_FORMAT   "sc16"
#define FPGPIO_DEFAULT_RX_RATE      1e6
#define FPGPIO_DEFAULT_TX_RATE      1e6
#define FPGPIO_DEFAULT_DWELL_TIME   2.0
#define FPGPIO_NUM_BITS             11

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

std::string to_bit_string(boost::uint16_t val)
{
    std::string out;
    for (int i = FPGPIO_NUM_BITS - 1; i >= 0; i--)
    {
        std::string bit = ((val >> i) & 1) ? "1" : "0";
        out += "  ";
        out += bit;
    }
    return out;
}

void output_reg_values(const std::string bank, const uhd::usrp::multi_usrp::sptr &usrp)
{
    std::cout << (boost::format("Bit       "));
    for (int i = FPGPIO_NUM_BITS - 1; i >= 0; i--)
        std::cout << (boost::format(" %s%d") % (i < 10 ? " " : "") % i);
    std::cout << std::endl;
    std::cout << "CTRL:     " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("CTRL")))) << std::endl;
    std::cout << "DDR:      " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("DDR")))) << std::endl;
    std::cout << "ATR_0X:   " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("ATR_0X")))) << std::endl;
    std::cout << "ATR_RX:   " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("ATR_RX")))) << std::endl;
    std::cout << "ATR_TX:   " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("ATR_TX")))) << std::endl;
    std::cout << "ATR_XX:   " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("ATR_XX")))) << std::endl;
    std::cout << "OUT:      " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("OUT")))) << std::endl;
    std::cout << "READBACK: " << to_bit_string(uint16_t(usrp->get_gpio_attr(bank, std::string("READBACK")))) << std::endl;
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args;
    std::string cpu, otw;
    double rx_rate, tx_rate, dwell;
    const std::string fpgpio = "FP0";

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("repeat", "repeat loop until Ctrl-C is pressed")
        ("cpu", po::value<std::string>(&cpu)->default_value(FPGPIO_DEFAULT_CPU_FORMAT), "cpu data format")
        ("otw", po::value<std::string>(&otw)->default_value(FPGPIO_DEFAULT_OTW_FORMAT), "over the wire data format")
        ("rx_rate", po::value<double>(&rx_rate)->default_value(FPGPIO_DEFAULT_RX_RATE), "rx sample rate")
        ("tx_rate", po::value<double>(&tx_rate)->default_value(FPGPIO_DEFAULT_TX_RATE), "tx sample rate")
        ("dwell", po::value<double>(&dwell)->default_value(FPGPIO_DEFAULT_DWELL_TIME), "dwell time in seconds for each test case")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Front Panel GPIO %s") % desc << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //print out initial unconfigured state of FP GPIO
    std::cout << "Unconfigured GPIO values:" << std::endl;
    output_reg_values(fpgpio, usrp);

    //configure GPIO registers
    uint32_t ctrl = 0;      // default all as manual
    uint32_t ddr = 0;       // default all as input
    uint32_t atr_idle = 0;
    uint32_t atr_rx = 0;
    uint32_t atr_tx = 0;
    uint32_t atr_duplex = 0;

    //set up FPGPIO outputs:
    //FPGPIO[0] = ATR output 1 at idle
    ctrl |= 1 << 0;
    atr_idle |= 1 << 0;
    ddr |= 1 << 0;

    //FPGPIO[1] = ATR output 1 during RX
    ctrl |= 1 << 1;
    ddr |= 1 << 1;
    atr_rx |= 1 << 1;

    //FPGPIO[2] = ATR output 1 during TX
    ctrl |= 1 << 2;
    ddr |= 1 << 2;
    atr_tx |= 1 << 2;

    //FPGPIO[3] = ATR output 1 during full duplex
    ctrl |= 1 << 3;
    ddr |= 1 << 3;
    atr_duplex |= 1 << 3;

    //FPGPIO[4] = output
    ddr |= 1 << 4;

    //set data direction register (DDR)
    usrp->set_gpio_attr(fpgpio, std::string("DDR"), ddr);

    //set ATR registers
    usrp->set_gpio_attr(fpgpio, std::string("ATR_0X"), atr_idle);
    usrp->set_gpio_attr(fpgpio, std::string("ATR_RX"), atr_rx);
    usrp->set_gpio_attr(fpgpio, std::string("ATR_TX"), atr_tx);
    usrp->set_gpio_attr(fpgpio, std::string("ATR_XX"), atr_duplex);

    //set control register
    usrp->set_gpio_attr(fpgpio, std::string("CTRL"), ctrl);

    //print out initial state of FP GPIO
    std::cout << "\nConfigured GPIO values:" << std::endl;
    output_reg_values(fpgpio, usrp);

    //set up streams
    uhd::stream_args_t rx_args(cpu, otw);
    uhd::stream_args_t tx_args(cpu, otw);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_args);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_args);
    uhd::stream_cmd_t rx_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    rx_cmd.stream_now = true;
    usrp->set_rx_rate(rx_rate);
    usrp->set_tx_rate(tx_rate);

    //set up buffers for tx and rx
    const size_t max_samps_per_packet = rx_stream->get_max_num_samps();
    const size_t nsamps_per_buff = max_samps_per_packet;
    std::vector<char> rx_buff(max_samps_per_packet*uhd::convert::get_bytes_per_item(cpu));
    std::vector<char> tx_buff(max_samps_per_packet*uhd::convert::get_bytes_per_item(cpu));
    std::vector<void *> rx_buffs, tx_buffs;
    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
        rx_buffs.push_back(&rx_buff.front()); //same buffer for each channel
    for (size_t ch = 0; ch < tx_stream->get_num_channels(); ch++)
        tx_buffs.push_back(&tx_buff.front()); //same buffer for each channel

    uhd::rx_metadata_t rx_md;
    uhd::tx_metadata_t tx_md;
    tx_md.has_time_spec = false;
    tx_md.start_of_burst = true;
    uhd::time_spec_t stop_time;
    double timeout = 0.01;
    uhd::time_spec_t dwell_time(dwell);
    int loop = 0;
    boost::uint32_t rb, expected;

    //register singal handler
    std::signal(SIGINT, &sig_int_handler);

    while (not stop_signal_called)
    {
        int failures = 0;

        std::cout << "Press Ctrl + C to quit..." << std::endl;

        // test user controlled GPIO and ATR idle by setting bit 4 high for 1 second
        std::cout << "\nTesting user controlled GPIO and ATR idle output..." << std::flush;
        usrp->set_gpio_attr(fpgpio, "OUT", 1 << 4, 1 << 4);
        stop_time = usrp->get_time_now() + dwell_time;
        while (not stop_signal_called and usrp->get_time_now() < stop_time)
        {
            usleep(10000);
        }
        rb = usrp->get_gpio_attr(fpgpio, "READBACK");
        expected = 1 << 4 | 1 << 0;
        if ((rb & expected) != expected)
        {
            ++failures;
            std::cout << "fail" << std::endl;
            if ((rb & (1 << 0)) == 0)
                std::cout << "Bit 0 should be set, but is not" << std::endl;
            if ((rb & (1 << 4)) == 0)
                std::cout << "Bit 4 should be set, but is not" << std::endl;
        } else {
            std::cout << "pass" << std::endl;
        }
        std::cout << std::endl;
        output_reg_values(fpgpio, usrp);
        usrp->set_gpio_attr(fpgpio, "OUT", 0, 1 << 4);
        if (stop_signal_called)
            break;

        // test ATR RX by receiving for 1 second
        std::cout << "\nTesting ATR RX output..." << std::flush;
        rx_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
        rx_stream->issue_stream_cmd(rx_cmd);
        stop_time = usrp->get_time_now() + dwell_time;
        while (not stop_signal_called and usrp->get_time_now() < stop_time)
        {
            try {
                rx_stream->recv(rx_buffs, nsamps_per_buff, rx_md, timeout);
            } catch(...){}
        }
        rb = usrp->get_gpio_attr(fpgpio, "READBACK");
        expected = 1 << 1;
        if ((rb & expected) != expected)
        {
            ++failures;
            std::cout << "fail" << std::endl;
            std::cout << "Bit 1 should be set, but is not" << std::endl;
        } else {
            std::cout << "pass" << std::endl;
        }
        std::cout << std::endl;
        output_reg_values(fpgpio, usrp);
        rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        //clear out any data left in the rx stream
        try {
            rx_stream->recv(rx_buffs, nsamps_per_buff, rx_md, timeout);
        } catch(...){}
        if (stop_signal_called)
            break;

        // test ATR TX by transmitting for 1 second
        std::cout << "\nTesting ATR TX output..." << std::flush;
        stop_time = usrp->get_time_now() + dwell_time;
        tx_md.start_of_burst = true;
        tx_md.end_of_burst = false;
        while (not stop_signal_called and usrp->get_time_now() < stop_time)
        {
            try {
                tx_stream->send(tx_buffs, nsamps_per_buff, tx_md, timeout);
                tx_md.start_of_burst = false;
            } catch(...){}
        }
        rb = usrp->get_gpio_attr(fpgpio, "READBACK");
        expected = 1 << 2;
        if ((rb & expected) != expected)
        {
            ++failures;
            std::cout << "fail" << std::endl;
            std::cout << "Bit 2 should be set, but is not" << std::endl;
        } else {
            std::cout << "pass" << std::endl;
        }
        std::cout << std::endl;
        output_reg_values(fpgpio, usrp);
        tx_md.end_of_burst = true;
        try {
            tx_stream->send(tx_buffs, nsamps_per_buff, tx_md, timeout);
        } catch(...){}
        if (stop_signal_called)
            break;

        // test ATR RX by transmitting and receiving for 1 second
        std::cout << "\nTesting ATR full duplex output..." << std::flush;
        rx_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
        rx_stream->issue_stream_cmd(rx_cmd);
        tx_md.start_of_burst = true;
        tx_md.end_of_burst = false;
        stop_time = usrp->get_time_now() + dwell_time;
        while (not stop_signal_called and usrp->get_time_now() < stop_time)
        {
            try {
                tx_stream->send(rx_buffs, nsamps_per_buff, tx_md, timeout);
                tx_md.start_of_burst = false;
                rx_stream->recv(tx_buffs, nsamps_per_buff, rx_md, timeout);
            } catch(...){}
        }
        rb = usrp->get_gpio_attr(fpgpio, "READBACK");
        expected = 1 << 3;
        if ((rb & expected) != expected)
        {
            ++failures;
            std::cout << "fail" << std::endl;
            std::cout << "Bit 3 should be set, but is not" << std::endl;
        } else {
            std::cout << "pass" << std::endl;
        }
        std::cout << std::endl;
        output_reg_values(fpgpio, usrp);
        rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        tx_md.end_of_burst = true;
        try {
            tx_stream->send(tx_buffs, nsamps_per_buff, tx_md, timeout);
        } catch(...){}
        //clear out any data left in the rx stream
        try {
            rx_stream->recv(rx_buffs, nsamps_per_buff, rx_md, timeout);
        } catch(...){}

        std::cout << std::endl;
        if (failures)
            std::cout << failures << " tests failed" << std::endl;
        else
            std::cout << "All tests passed!" << std::endl;

        if (!vm.count("repeat"))
            break;

        std::cout << (boost::format("\nLoop %d completed")  % ++loop) << std::endl;
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
