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

#include <iostream>
#include <map>
#include <fstream>
#include <stdexcept>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <boost/assign.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include "usrp_x3xx_burner_utils.hpp"
#include <uhd/exception.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/safe_call.hpp>

namespace po = boost::program_options;

using namespace uhd;
using namespace uhd::transport;

boost::uint8_t x300_data_in_mem[udp_simple::mtu];
boost::uint8_t intermediary_packet_data[X300_PACKET_SIZE_BYTES];

boost::uint8_t bitswap(uint8_t b){
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;

    return b;
}

bool initialize_connection(udp_simple::sptr udp_transport){
    const x300_fpga_update_data_t *update_data_in = reinterpret_cast<const x300_fpga_update_data_t *>(x300_data_in_mem);

    x300_fpga_update_data_t ack_packet;
    ack_packet.flags = htonx<boost::uint32_t>(X300_FPGA_PROG_FLAGS_ACK | X300_FPGA_PROG_FLAGS_INIT);
    ack_packet.sector = 0;
    ack_packet.size = 0;
    ack_packet.index = 0;
    memset(ack_packet.data, 0, sizeof(ack_packet.data));
    udp_transport->send(boost::asio::buffer(&ack_packet, sizeof(ack_packet)));

    udp_transport->recv(boost::asio::buffer(x300_data_in_mem), UDP_TIMEOUT);
    if((ntohl(update_data_in->flags) & X300_FPGA_PROG_FLAGS_ERROR) != X300_FPGA_PROG_FLAGS_ERROR){
        std::cout << "Found X3x0: " << udp_transport->get_recv_addr() << std::endl;
        return true;
    }
    else return false;
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    memset(intermediary_packet_data, 0, X300_PACKET_SIZE_BYTES);
    std::string ip_addr, fpga_path;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Display this help message.")
        ("addr", po::value<std::string>(&ip_addr)->default_value("192.168.10.2"), "Specify an IP address.")
        ("fpga-path", po::value<std::string>(&fpga_path)->default_value(""), "Specify an FPGA path.")
        ("list", "List all available X3x0 devices.")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")){
        std::cout << "USRP X3x0 Net Burner - " << desc << std::endl;
        return EXIT_FAILURE;
    }
    
    //List connected X3x0 devices
    if(vm.count("list")){
        const x300_fpga_update_data_t *update_data_in = reinterpret_cast<const x300_fpga_update_data_t *>(x300_data_in_mem);
    
        udp_simple::sptr udp_bc_transport;
        x300_fpga_update_data_t config_status_packet;
        config_status_packet.flags = htonx<boost::uint32_t>(X300_FPGA_PROG_CONFIG_STATUS | X300_FPGA_PROG_FLAGS_ACK);
        config_status_packet.sector = 0;
        config_status_packet.size = 0;
        config_status_packet.index = 0;
        memset(config_status_packet.data, 0, sizeof(config_status_packet.data));
        
        std::cout << "Available X3x0 devices:" << std::endl;
        
        BOOST_FOREACH(const if_addrs_t &if_addrs, get_if_addrs()){
            //Avoid the loopback device
            if(if_addrs.inet == boost::asio::ip::address_v4::loopback().to_string()) continue;
            udp_bc_transport = udp_simple::make_broadcast(if_addrs.bcast, BOOST_STRINGIZE(X300_FPGA_PROG_UDP_PORT));
            udp_bc_transport->send(boost::asio::buffer(&config_status_packet, sizeof(config_status_packet)));
            
            udp_bc_transport->recv(boost::asio::buffer(x300_data_in_mem), 1);
            if((ntohl(update_data_in->flags) & X300_FPGA_PROG_FLAGS_ERROR) != X300_FPGA_PROG_FLAGS_ERROR
            and udp_bc_transport->get_recv_addr() != "0.0.0.0"){
                std::cout << " * " << udp_bc_transport->get_recv_addr() << std::endl;
            }
        }
    
        return EXIT_FAILURE;
    }

    size_t fpga_image_size;
    FILE* file;
    if((file = fopen(fpga_path.c_str(), "rb"))){
        fseek(file, 0, SEEK_END);
        fpga_image_size = ftell(file);
        if(fpga_image_size > X300_FPGA_IMAGE_SIZE_BYTES){
            fclose(file);
            throw std::runtime_error(str(boost::format("FPGA size is too large (%d > %d).")
                                         % fpga_image_size % X300_FPGA_IMAGE_SIZE_BYTES));
        }
        rewind(file);
    }
    else
    {
        throw std::runtime_error("Could not find specified FPGA image!");
    }
    
    std::cout << "Attempting to connect to: " << ip_addr << std::endl;
    udp_simple::sptr udp_transport = udp_simple::make_connected(ip_addr, BOOST_STRINGIZE(X300_FPGA_PROG_UDP_PORT));
    if(not initialize_connection(udp_transport)){
        throw std::runtime_error("Flash initialization failed!");
    }

    std::cout << "Burning image: " << fpga_path << std::endl << std::endl;
    std::cout << "Progress: " << std::flush;

    int percentage = -1;
    int last_percentage = -1;

    //Each sector
    for(size_t i = 0; i < fpga_image_size; i += X300_FLASH_SECTOR_SIZE){

        //Print percentage at beginning of first sector after each 10%
        int percentage = int(double(i)/double(fpga_image_size)*100);
        if(percentage != last_percentage and (percentage % 10 == 0)){ //Don't print same percentage twice
            std::cout << int(double(i)/double(fpga_image_size)*100) << "%..." << std::flush;
        }
        last_percentage = percentage;

        //Each packet
        for(size_t j = i; (j < fpga_image_size and j < (i+X300_FLASH_SECTOR_SIZE)); j += X300_PACKET_SIZE_BYTES){
            x300_fpga_update_data_t send_packet;

            send_packet.flags = X300_FPGA_PROG_FLAGS_ACK;
            if(j == i) send_packet.flags |= X300_FPGA_PROG_FLAGS_ERASE; //Erase the sector before writing
            send_packet.flags = htonx<boost::uint32_t>(send_packet.flags);
            
            send_packet.sector = htonx<boost::uint32_t>(X300_FPGA_SECTOR_START + (i/X300_FLASH_SECTOR_SIZE));
            send_packet.index = htonx<boost::uint32_t>((j % X300_FLASH_SECTOR_SIZE) / 2);
            send_packet.size = htonx<boost::uint32_t>(X300_PACKET_SIZE_BYTES / 2);
            memset(intermediary_packet_data,0,X300_PACKET_SIZE_BYTES);
            memset(send_packet.data,0,X300_PACKET_SIZE_BYTES);
            size_t current_pos = ftell(file);
            size_t send_size = 0;

            if(current_pos + 256 > fpga_image_size){
                fread(intermediary_packet_data, sizeof(boost::uint8_t), (fpga_image_size-current_pos), file);
                send_size = (fpga_image_size-current_pos);
            }
            else{
                fread(intermediary_packet_data, sizeof(boost::uint8_t), X300_PACKET_SIZE_BYTES, file);
                send_size = 256;
            }

            for(size_t k = 0; k < X300_PACKET_SIZE_BYTES; k++){
                intermediary_packet_data[k] = bitswap(intermediary_packet_data[k]);
            }

            memcpy(send_packet.data, intermediary_packet_data, X300_PACKET_SIZE_BYTES);

            for(size_t k = 0; k < (X300_PACKET_SIZE_BYTES/2); k++){
                send_packet.data[k] = htonx<boost::uint16_t>(send_packet.data[k]);
            }
           
            udp_transport->send(boost::asio::buffer(&send_packet, sizeof(send_packet)));

            udp_transport->recv(boost::asio::buffer(x300_data_in_mem), UDP_TIMEOUT);
            const x300_fpga_update_data_t *update_data_in = reinterpret_cast<const x300_fpga_update_data_t *>(x300_data_in_mem);

            if((ntohl(update_data_in->flags) & X300_FPGA_PROG_FLAGS_ERROR) == X300_FPGA_PROG_FLAGS_ERROR){
                throw std::runtime_error("Transfer or data verification failed!");
            }
        }
    }
    fclose(file);

    std::cout << "100%" << std::endl << std::endl;
    x300_fpga_update_data_t configure_packet;
    configure_packet.flags = htonx<boost::uint32_t>(X300_FPGA_PROG_CONFIGURE | X300_FPGA_PROG_FLAGS_ACK);
    configure_packet.sector = 0;
    configure_packet.size = 0;
    configure_packet.index = 0;
    memset(configure_packet.data, 0, sizeof(configure_packet.data));
    udp_transport->send(boost::asio::buffer(&configure_packet, sizeof(configure_packet)));

    udp_transport->recv(boost::asio::buffer(x300_data_in_mem), UDP_TIMEOUT);
    const x300_fpga_update_data_t *configure_data_in = reinterpret_cast<const x300_fpga_update_data_t *>(x300_data_in_mem);
    bool successful = false;

    if((ntohl(configure_data_in->flags) & X300_FPGA_PROG_FLAGS_ERROR) == X300_FPGA_PROG_FLAGS_ERROR){
        throw std::runtime_error("Transfer or data verification failed!");
    }
    else{
        std::cout << "Waiting for X3x0 to set new FPGA image and reload." << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

        x300_fpga_update_data_t config_status_packet;
        configure_packet.flags = htonx<boost::uint32_t>(X300_FPGA_PROG_CONFIG_STATUS);
        config_status_packet.sector = 0;
        config_status_packet.size = 0;
        config_status_packet.index = 0;
        memset(config_status_packet.data, 0, sizeof(config_status_packet.data));
        for(int i = 0; i < 5; i++){
            udp_transport->send(boost::asio::buffer(&config_status_packet, sizeof(config_status_packet)));
            udp_transport->recv(boost::asio::buffer(x300_data_in_mem), 1);
            const x300_fpga_update_data_t *config_status_data_in = reinterpret_cast<const x300_fpga_update_data_t *>(x300_data_in_mem);

            if((ntohl(config_status_data_in->flags) & X300_FPGA_PROG_FLAGS_ERROR) != X300_FPGA_PROG_FLAGS_ERROR
               and udp_transport->get_recv_addr() == ip_addr){
                successful = true;
                break;
            }
            successful = false; //If it worked, the break would skip this
        }
    }
    if(successful) std::cout << "Successfully burned FPGA image!" << std::endl;
    else{
        throw std::runtime_error("FPGA image burning failed!");
    }

    return EXIT_SUCCESS;
}
