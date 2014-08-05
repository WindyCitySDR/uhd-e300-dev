//
// Copyright 2014 Ettus Research LLC
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

#include "../lib/usrp/e300/e300_network.hpp"
#include <uhd/device.hpp>

#include <uhd/utils/msg.hpp>

#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <iostream>

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("fpga", po::value<std::string>(), "fpga image to load")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD E3x0 Network Mode %s") % desc << std::endl;
        return EXIT_FAILURE;
    }
    uhd::device_addr_t args;
    if(vm.count("fpga")) {
        args["fpga"] = vm["fpga"].as<std::string>();
    }

    uhd::usrp::e300::network_server::sptr server = uhd::usrp::e300::network_server::make(args);
    server->run();
}
