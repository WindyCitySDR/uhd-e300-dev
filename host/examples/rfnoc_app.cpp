//
// Copyright 2010-2011,2014 Ettus Research LLC
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

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}


////////////////////////// APPS /////////////////////////////////////////////////////
////////////////////////// APP1: null source -> host ////////////////////////////////
void run_app_null_source_to_host(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &file,
    unsigned long long num_requested_samples,
    double time_requested = 0.0,
    bool bw_summary = false,
    bool stats = false,
    bool null = false,
    boost::uint32_t rate_factor = 12,
    boost::uint32_t lines_per_packet = 50
){
    std::cout << "===== NOTE: This app requires a null source on CE1. =========" << std::endl;
    rate_factor &= 0xFFFF;
    if (lines_per_packet == 0) {
        lines_per_packet = 50;
    } else if (lines_per_packet > 175) {
        lines_per_packet = 175;
    }
    unsigned long long num_total_samps = 0;
    // Create a receive streamer to CE1
    uhd::stream_args_t stream_args("sc16", "sc16");
    stream_args.args["src_addr"] = "1"; // 1 is null source
    stream_args.channels = std::vector<size_t>(1, 0);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    // Get sid for this connection (channel 0 because there's only 1 channel):
    boost::uint32_t data_sid = rx_stream->get_sid(0);
    // Configure null source:
    usrp->get_device()->rfnoc_cmd(
            "ce1", "set_fc",
            20000, // Host buffer: This is pretty big
            0 // No upstream block
    );
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            8, // Register 8: Set SID
            0x02140000 /* 2.20 */ | ((data_sid >> 16) & 0xFFFF)
    );
    std::cout << "Setting lines per packet to " << lines_per_packet << " => Packet size: " << lines_per_packet * 8 << " Bytes, " << lines_per_packet * 2 << " Samples." << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            9, // Register 9: Lines per packet
            lines_per_packet
    );
    std::cout << "Setting divider to " << rate_factor << ", ~" << (160.0 * 8.0 / (rate_factor + 1)) << " MByte/s" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            10, // Register 10: Rate
            rate_factor // Rate in clock cycles (max 16 bits)
    );

    size_t bytes_per_packet = lines_per_packet * 8;
    size_t samples_per_packet = bytes_per_packet / 4;

    uhd::rx_metadata_t md;
    std::vector<std::complex<short> > buff(samples_per_packet);
    std::ofstream outfile;
    if (not null)
        outfile.open(file.c_str(), std::ofstream::binary);

    // Setup streaming
    std::cout << "Sending command to start streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            true
    );
    std::cout << "Done" << std::endl;

    boost::system_time start = boost::get_system_time();
    unsigned long long ticks_requested = (long)(time_requested * (double)boost::posix_time::time_duration::ticks_per_second());
    boost::posix_time::time_duration ticks_diff;
    boost::system_time last_update = start;
    unsigned long long last_update_samps = 0;
    size_t n_packets = 0;

    while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)) {
        boost::system_time now = boost::get_system_time();
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0);
        if (num_rx_samps) {
            n_packets += num_rx_samps / samples_per_packet;
	}

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
            std::cerr << error << std::endl;
        }

        num_total_samps += num_rx_samps;

        if (outfile.is_open())
            outfile.write((const char*)&buff.front(), num_rx_samps * 4);

        if (bw_summary) {
            last_update_samps += num_rx_samps;
            boost::posix_time::time_duration update_diff = now - last_update;
            if (update_diff.ticks() > boost::posix_time::time_duration::ticks_per_second()) {
                double t = (double)update_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
                double r = (double)last_update_samps / t;
                std::cout << boost::format("\t%f Msps") % (r/1e6) << std::endl;
                last_update_samps = 0;
                last_update = now;
            }
        }

        ticks_diff = now - start;
        if (ticks_requested > 0){
            if ((unsigned long long)ticks_diff.ticks() > ticks_requested)
                break;
        }
    } // end while

    // Stop streaming
    std::cout << "Sending command to stop streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            false
    );
    std::cout << "Done" << std::endl;

    // Run recv until nothing is left
    int num_post_samps = 0;
    do {
        num_post_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0);
    } while(num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

    if (outfile.is_open())
        outfile.close();
    if (stats) {
        std::cout << std::endl;
        double t = (double)ticks_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
        std::cout << boost::format("Received %d packets in %f seconds") % n_packets % t << std::endl;
        std::cout << boost::format("Received %d bytes in %f seconds") % (num_total_samps*4) % t << std::endl;
        double r = (double)num_total_samps / t;
        std::cout << boost::format("%f MByte/s") % (r/1e6*4) << std::endl;
    }
}

////////////////////////// APP2: null source -> 8/16 converter -> host ////////////////////
void run_app_null_source_converter_host(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &file,
    unsigned long long num_requested_samples,
    double time_requested = 0.0,
    bool bw_summary = false,
    bool stats = false,
    bool null = false,
    boost::uint32_t rate_factor = 12,
    boost::uint32_t lines_per_packet = 50
){
    std::cout << "===== NOTE: This app requires a null source on CE1 and a converter on CE0. =========" << std::endl;
    rate_factor &= 0xFFFF;
    if (lines_per_packet == 0) {
        lines_per_packet = 50;
    } else if (lines_per_packet > 180) {
        lines_per_packet = 180;
    }

    size_t bytes_per_packet = lines_per_packet * 8;
    size_t samples_per_packet = bytes_per_packet / 4;
    double expected_rate = (160.0 * 8.0 / (rate_factor + 1)) * 2; // *2 'cause of converter

    unsigned long long num_total_samps = 0;

    // Create a receive streamer to CE0
    uhd::stream_args_t stream_args("sc16", "sc16");
    stream_args.args["src_addr"] = "0"; // 0 is converter
    stream_args.channels = std::vector<size_t>(1, 0);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    // Get sid for this connection (channel 0 because there's only 1 channel):
    boost::uint32_t data_sid = rx_stream->get_sid(0);

    // Configure null source:
    // Configure null source:
    usrp->get_device()->rfnoc_cmd(
            "ce1", "set_fc",
            7500/bytes_per_packet, // CE0 has 8k buffer
            0 // No upstream block
    );
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            8, // Register 8: Set SID
            0x02140210 /* 2.20 to 2.16 */
    );
    std::cout << "Setting lines per packet to " << lines_per_packet << " => Packet size: " << lines_per_packet * 8 << " Bytes, " << lines_per_packet * 2 << " Samples." << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            9, // Register 9: Lines per packet
            lines_per_packet
    );
    std::cout << "Setting divider to " << rate_factor << ", ~" << expected_rate << " MByte/s" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            10, // Register 10: Rate
            rate_factor // Rate in clock cycles (max 16 bits)
    );

    // Configure converter
    std::cout << "Converter will send to address " << str(boost::format("0x%08x") % ((data_sid >> 16) & 0xFFFF)) << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce0", "poke",
            8, // Register 8: Set SID
	    (1<<16) /* use SID */ | ((data_sid >> 16) & 0xFFFF) /* send to our streamer */
    );
    usrp->get_device()->rfnoc_cmd(
            "ce0", "set_fc",
            20000, // Host has a large buffer
            2 // Report every 2nd packet to CE0
    );

    uhd::rx_metadata_t md;
    std::vector<std::complex<short> > buff(samples_per_packet*2);
    std::ofstream outfile;
    if (not null)
        outfile.open(file.c_str(), std::ofstream::binary);

    // Setup streaming
    std::cout << "Sending command to start streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            true
    );
    std::cout << "Done" << std::endl;

    boost::system_time start = boost::get_system_time();
    unsigned long long ticks_requested = (long)(time_requested * (double)boost::posix_time::time_duration::ticks_per_second());
    boost::posix_time::time_duration ticks_diff;
    boost::system_time last_update = start;
    unsigned long long last_update_samps = 0;
    size_t n_packets = 0;

    while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)) {
        boost::system_time now = boost::get_system_time();
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0);
        if (num_rx_samps) {
            n_packets += num_rx_samps / samples_per_packet;
	}

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
            std::cerr << error << std::endl;
        }

        num_total_samps += num_rx_samps;

        if (outfile.is_open())
            outfile.write((const char*)&buff.front(), num_rx_samps*4);

        if (bw_summary) {
            last_update_samps += num_rx_samps;
            boost::posix_time::time_duration update_diff = now - last_update;
            if (update_diff.ticks() > boost::posix_time::time_duration::ticks_per_second()) {
                double t = (double)update_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
                double r = (double)last_update_samps * 4.0 / t;
                std::cout << boost::format("\t%f MByte/s") % (r/1e6) << std::endl;
                last_update_samps = 0;
                last_update = now;
            }
        }

        ticks_diff = now - start;
        if (ticks_requested > 0){
            if ((unsigned long long)ticks_diff.ticks() > ticks_requested)
                break;
        }
    } // end while

    // Stop streaming
    std::cout << "Sending command to stop streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            false
    );
    std::cout << "Done" << std::endl;

    // Run recv until nothing is left
    int num_post_samps = 0;
    do {
        num_post_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0);
    } while(num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

    if (outfile.is_open())
        outfile.close();
    if (stats) {
        std::cout << std::endl;
        double t = (double)ticks_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
        std::cout << boost::format("Received %d packets in %f seconds") % n_packets % t << std::endl;
        std::cout << boost::format("Received %d bytes in %f seconds") % (num_total_samps*4) % t << std::endl;
        double r = (double)num_total_samps / t;
        std::cout << boost::format("%f MByte/s") % (r/1e6*4) << "  (Expected: " << expected_rate << " MByte/s)" << std::endl;
    }
}

////////////////////////// APP: host -> null sink ////////////////////
void run_app_host_to_null_sink(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &file,
    unsigned long long num_requested_samples,
    double time_requested = 0.0,
    bool bw_summary = false,
    bool stats = false,
    bool null = false,
    boost::uint32_t bytes_per_packet = 1400,
    boost::uint32_t lines_per_packet = 50
){
    std::cout << "===== NOTE: This app requires a null sink on CE2. =========" << std::endl;
    size_t samples_per_packet = bytes_per_packet / 4;
    bytes_per_packet = samples_per_packet * 4;
    std::cout << "Bytes per packet: " << bytes_per_packet << std::endl;

    unsigned long long num_total_samps = 0;

    // Create a transmit streamer to CE2
    uhd::stream_args_t stream_args("sc16", "sc16");
    stream_args.args["src_addr"] = "2";
    stream_args.channels = std::vector<size_t>(1, 0);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
    // Get sid for this connection (channel 0 because there's only 1 channel):
    boost::uint32_t data_sid = tx_stream->get_sid(0);

    // Configure null sink:
    usrp->get_device()->rfnoc_cmd(
            "ce2", "set_fc",
            0, // No downstream block
            2 // Report every 2nd packet
    );

    uhd::tx_metadata_t md;
    std::vector<std::complex<short> > buff(samples_per_packet);

    boost::system_time start = boost::get_system_time();
    unsigned long long ticks_requested = (long)(time_requested * (double)boost::posix_time::time_duration::ticks_per_second());
    boost::posix_time::time_duration ticks_diff;
    boost::system_time last_update = start;
    unsigned long long last_update_samps = 0;
    size_t n_packets = 0;

    while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)) {
        boost::system_time now = boost::get_system_time();
        size_t num_tx_samps = tx_stream->send(&buff.front(), buff.size(), md, 3.0);
	if (num_tx_samps < buff.size()) {
            std::cout << "Timeout!" << std::endl;
	}
        num_total_samps += num_tx_samps;

        if (bw_summary) {
            last_update_samps += num_tx_samps;
            boost::posix_time::time_duration update_diff = now - last_update;
            if (update_diff.ticks() > boost::posix_time::time_duration::ticks_per_second()) {
                double t = (double)update_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
                double r = (double)last_update_samps * 4.0 / t;
                std::cout << boost::format("\t%f MByte/s") % (r/1e6) << std::endl;
                last_update_samps = 0;
                last_update = now;
            }
        }

        ticks_diff = now - start;
        if (ticks_requested > 0){
            if ((unsigned long long)ticks_diff.ticks() > ticks_requested)
                break;
        }
    } // end while

    if (stats) {
        std::cout << std::endl;
        double t = (double)ticks_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
        std::cout << boost::format("Received %d packets in %f seconds") % n_packets % t << std::endl;
        std::cout << boost::format("Received %d bytes in %f seconds") % (num_total_samps*4) % t << std::endl;
        double r = (double)num_total_samps / t;
        std::cout << boost::format("%f MByte/s") % (r/1e6*4) << std::endl;
    }
}


////////////////////////// APP2: null source -> null sink ////////////////////////////////
void run_app_null_source_to_null_sink(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &file,
    double time_requested = 0.0,
    bool bw_summary = false,
    bool stats = false,
    bool null = false,
    boost::uint32_t rate_factor = 12,
    boost::uint32_t lines_per_packet = 50
){
    std::cout << "=== NOTE: This app requires a null source on CE1 and a null sink on CE2 =======" << std::endl;
    rate_factor &= 0xFFFF;
    if (lines_per_packet == 0) {
        lines_per_packet = 50;
    } else if (lines_per_packet > 175) {
        lines_per_packet = 175;
    }
    if (time_requested == 0.0) {
        time_requested = 10;
        std::cout << "Setting req'd time to " << time_requested << "s" << std::endl;
    }
    size_t bytes_per_packet = lines_per_packet * 8;
    size_t samples_per_packet = bytes_per_packet / 4;

    // Configure null source:
    usrp->get_device()->rfnoc_cmd(
            "ce1", "set_fc",
            7000/bytes_per_packet, // We have 8k buffer
            0 // No upstream block
    );
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            8, // Register 8: Set SID
            0x02140218 /* 2.20 to 2.24*/
    );
    std::cout << "Setting lines per packet to " << lines_per_packet << " => Packet size: " << lines_per_packet * 8 << " Bytes, " << lines_per_packet * 2 << " Samples." << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            9, // Register 9: Lines per packet
            lines_per_packet
    );
    std::cout << "Setting divider to " << rate_factor << ", ~" << (160.0 * 8.0 / (rate_factor + 1)) << " MByte/s" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            10, // Register 10: Rate
            rate_factor // Rate in clock cycles (max 16 bits)
    );
    // Configure null sink
    usrp->get_device()->rfnoc_cmd(
            "ce2", "set_fc",
            0, // No downstream block
            2 // Report every 2nd block
    );

    // Setup streaming
    std::cout << "Sending command to start streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            true
    );
    std::cout << "Done" << std::endl;

    std::cout << "Sleeping for " << time_requested << " s..." << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(time_requested));

    // Stop streaming
    std::cout << "Sending command to stop streaming:" << std::endl;
    usrp->get_device()->rfnoc_cmd(
            "ce1", "poke",
            0x0B, // Register 11: Enable
            false
    );
    std::cout << "Done" << std::endl;

    // Sleep for a little bit to allow gunk to propagate
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}


///////////////////// HELPER FUNCTIONS ////////////////////////////////////////////////////
typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time){
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;

    boost::system_time start = boost::get_system_time();
    boost::system_time first_lock_time;

    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();

    while (true){
        if ((not first_lock_time.is_not_a_date_time()) and
                (boost::get_system_time() > (first_lock_time + boost::posix_time::seconds(setup_time))))
        {
            std::cout << " locked." << std::endl;
            break;
        }

        if (get_sensor_fn(sensor_name).to_bool()){
            if (first_lock_time.is_not_a_date_time())
                first_lock_time = boost::get_system_time();
            std::cout << "+";
            std::cout.flush();
        }
        else{
            first_lock_time = boost::system_time();	//reset to 'not a date time'

            if (boost::get_system_time() > (start + boost::posix_time::seconds(setup_time))){
                std::cout << std::endl;
                throw std::runtime_error(str(boost::format("timed out waiting for consecutive locks on sensor \"%s\"") % sensor_name));
            }

            std::cout << "_";
            std::cout.flush();
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    std::cout << std::endl;

    return true;
}

///////////////////// MAIN ////////////////////////////////////////////////////
int UHD_SAFE_MAIN(int argc, char *argv[])
{
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, file, type, ant, subdev, ref, app;
    size_t total_num_samps, spb;
    double rate, freq, gain, total_time, setup_time;
    boost::uint32_t app_arg1, app_arg2;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("app", po::value<std::string>(&app)->default_value(""), "which app to run")
        ("app_arg1", po::value<boost::uint32_t>(&app_arg1)->default_value(0), "app argument 1")
        ("app_arg2", po::value<boost::uint32_t>(&app_arg2)->default_value(0), "app argument 2")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("time", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<double>(&rate)->default_value(1e6), "rate of incoming samples")
        ("freq", po::value<double>(&freq)->default_value(0.0), "RF center frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "daughterboard antenna selection")
        ("subdev", po::value<std::string>(&subdev), "daughterboard subdevice specification")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "waveform type (internal, external, mimo)")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
        ("progress", "periodically display short-term bandwidth")
        ("stats", "show average bandwidth on exit")
        ("null", "run without writing to file")
        ("continue", "don't abort on a bad packet")
        ("skip-lo", "skip checking LO lock status")
        ("int-n", "tune USRP with integer-N tuning")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
        return ~0;
    }

    bool bw_summary = vm.count("progress") > 0;
    bool stats = vm.count("stats") > 0;
    bool null = vm.count("null") > 0 or vm.count("file") == 0;
    bool continue_on_bad_packet = vm.count("continue") > 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //Lock mboard clocks
    usrp->set_clock_source(ref);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (rate <= 0.0){
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency
    if (vm.count("freq")){	//with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
    }

    //set the rf gain
    if (vm.count("gain")){
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    //set the antenna
    if (vm.count("ant")) usrp->set_rx_antenna(ant);

    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); //allow for some setup time

    //check Ref and LO Lock detect
    if (not vm.count("skip-lo")){
        check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, 0), setup_time);
        if (ref == "mimo")
            check_locked_sensor(usrp->get_mboard_sensor_names(0), "mimo_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);
        if (ref == "external")
            check_locked_sensor(usrp->get_mboard_sensor_names(0), "ref_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // Add your own app here, just pass those args you actually need
    if (app == "null_source_to_host") {
        run_app_null_source_to_host(
            usrp, file, total_num_samps, total_time, bw_summary,
            stats, null,
	    app_arg1, app_arg2
        );
    }
    else if (app == "null_source_converter_host") {
        run_app_null_source_converter_host(
            usrp, file, total_num_samps, total_time, bw_summary,
            stats, null,
	    app_arg1, app_arg2
        );
    }
    else if (app == "null_source_to_null_sink") {
        run_app_null_source_to_null_sink(
            usrp, file, total_time, bw_summary,
            stats, null,
	    app_arg1, app_arg2
        );
    }
    else if (app == "host_to_null_sink") {
        run_app_host_to_null_sink(
            usrp, file, total_time, bw_summary,
            stats, null,
	    app_arg1, app_arg2
        );
    }
    else {
        std::cout << "unknown app" << std::endl;
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
