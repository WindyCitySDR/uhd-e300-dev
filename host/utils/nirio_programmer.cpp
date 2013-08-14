
#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nifpga_interface.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

int main(int argc, char *argv[])
{
    using namespace nirio_interface;
    nirio_status status = NiRio_Status_Success;

    //Setup the program options
    uint32_t interface_num, peek_addr, poke_addr, poke_data;
    std::string fpga_lvbitx_path, flash_path, peek_tokens_str, poke_tokens_str;

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("interface", po::value<uint32_t>(&interface_num)->default_value(0), "The interface number to communicate with.")
        ("fpga", po::value<std::string>(&fpga_lvbitx_path)->default_value(""), "The path to the LVBITX file to download to the FPGA.")
        ("flash", po::value<std::string>(&flash_path)->default_value(""), "The path to the image to download to the flash OR 'erase' to erase the FPGA image from flash.")
        ("en-fpga-master", "Allow FPGA to master the Chinch in the packet network.")
        ("peek", po::value<std::string>(&peek_tokens_str)->default_value(""), "Peek32.")
        ("poke", po::value<std::string>(&poke_tokens_str)->default_value(""), "Poke32.")
        ("status", "Dump status information. WARNING: This required the IoPort2 to be up.")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //Print the help message
    if (vm.count("help")){
        std::cout << boost::format("USRP-NIRIO-Programmer\n\n %s") % desc << std::endl;
        return ~0;
    }

    //Download LVBITX image
    if (fpga_lvbitx_path != "")
    {
        std::string resource_name = boost::str(boost::format("RIO%u") % interface_num);
        printf("Downloading image %s to FPGA as %s...", fpga_lvbitx_path.c_str(), resource_name.c_str());
        fflush(stdout);
        nirio_status_chain(nifpga_interface::nifpga_session::load_lib(), status);
        uint32_t attributes = nifpga_interface::nifpga_session::OPEN_ATTR_SKIP_SIGNATURE_CHECK |
                nifpga_interface::nifpga_session::OPEN_ATTR_FORCE_DOWNLOAD;
        nifpga_interface::nifpga_session fpga_session(resource_name);
        nirio_status_chain(fpga_session.open(fpga_lvbitx_path, NULL, attributes), status);
        //Download BIN to flash or erase
        if (flash_path != "erase") {
            if (flash_path != "") {
                printf("Writing FPGA image %s to flash...", flash_path.c_str());
                fflush(stdout);
                nirio_status_chain(fpga_session.download_bitstream_to_flash(flash_path), status);
                printf("DONE\n");
            }
        } else {
            printf("Erasing FPGA image from flash...");
            fflush(stdout);
            nirio_status_chain(fpga_session.download_bitstream_to_flash(""), status);
            printf("DONE\n");
        }
        fpga_session.close();
        nifpga_interface::nifpga_session::unload_lib();
        printf("DONE\n");
    }

    niriok_proxy dev_proxy;
    fflush(stdout);
    std::string interface_path = niriok_proxy::get_interface_path(interface_num);
    if (interface_path.empty()) {
        printf("ERROR: Could not open a proxy to interface %u. If it exists, try downloading an LVBITX to the FPGA first.\n", interface_num);
        exit(EXIT_FAILURE);
    }

    dev_proxy.open(interface_path);

    //Handle FPGA master mode
    if (vm.count("en-fpga-master")){
        printf("Configuring STC3 and FPGA to master the Chinch...");
        fflush(stdout);

        uint32_t reg_value;

        //Write HBRCR
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceBusInterface), status);
        nirio_status_chain(dev_proxy.peek(0xA4, reg_value), status);
        reg_value |= 0x80000000;
        nirio_status_chain(dev_proxy.poke(0xA4, reg_value), status);

        //Write BIM Caps Register
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceFpga), status);
        nirio_status_chain(dev_proxy.peek(0x1810, reg_value), status);
        reg_value &= 0xFFFFFF00;
        nirio_status_chain(dev_proxy.poke(0x1810, reg_value), status);

        printf("DONE\n");
    }

    if (poke_tokens_str != ""){
        std::stringstream ss;
        std::vector<std::string> poke_tokens;
        boost::split(poke_tokens, poke_tokens_str, boost::is_any_of(":"));
        ss.clear();
        ss << std::hex << poke_tokens[1];
        ss >> poke_addr;
        ss.clear();
        ss << std::hex << poke_tokens[2];
        ss >> poke_data;

        tRioAddressSpace addr_space = poke_tokens[0]=="c"?kRioAddressSpaceBusInterface:kRioAddressSpaceFpga;
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, addr_space), status);
        if (poke_tokens[0]=="z") {
            nirio_status_chain(dev_proxy.poke(poke_addr, (uint32_t)0x70000 + poke_addr), status);
        } else {
            nirio_status_chain(dev_proxy.poke(poke_addr, poke_data), status);
        }
        printf("[POKE] %s:0x%x <= 0x%x (%u)\n", poke_tokens[0]=="c"?"Chinch":(poke_tokens[0]=="z"?"ZPU":"FPGA"), poke_addr, poke_data, poke_data);
    }

    if (peek_tokens_str != ""){
        std::stringstream ss;
        std::vector<std::string> peek_tokens;
        boost::split(peek_tokens, peek_tokens_str, boost::is_any_of(":"));
        ss.clear();
        ss << std::hex << peek_tokens[1];
        ss >> peek_addr;

        tRioAddressSpace addr_space = peek_tokens[0]=="c"?kRioAddressSpaceBusInterface:kRioAddressSpaceFpga;
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, addr_space), status);
        uint32_t reg_val;
        if (peek_tokens[0]=="z") {
            nirio_status_chain(dev_proxy.poke((uint32_t)0x60000 + peek_addr, (uint32_t)0), status);
            do {
                nirio_status_chain(dev_proxy.peek((uint32_t)0x60000 + peek_addr, reg_val), status);
            } while (reg_val != 0);
            nirio_status_chain(dev_proxy.peek((uint32_t)0x70000 + peek_addr, reg_val), status);
        } else {
            nirio_status_chain(dev_proxy.peek(peek_addr, reg_val), status);
        }

        printf("[PEEK] %s:0x%x = 0x%x (%u)\n", peek_tokens[0]=="c"?"Chinch":(peek_tokens[0]=="z"?"ZPU":"FPGA"), peek_addr, reg_val, reg_val);
    }

    //Display attributes
    if (vm.count("status")){
        printf("[Interface %u Status]\n", interface_num);
        uint32_t attr_val;
        nirio_status_chain(dev_proxy.get_attribute(kRioIsFpgaProgrammed, attr_val), status);
        printf("* Is FPGA Programmed? = %s\n", (attr_val==1)?"YES":"NO");

        std::string signature;
        for (int i = 0; i < 4; i++) {
            nirio_status_chain(dev_proxy.peek(0xFFF4, attr_val), status);
            signature += boost::str(boost::format("%08x") % attr_val);
        }
        printf("* FPGA Signature = %s\n", signature.c_str());

        uint32_t reg_val;
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceBusInterface), status);
        nirio_status_chain(dev_proxy.peek(0, reg_val), status);
        printf("* Chinch Signature = %x\n", reg_val);
        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceFpga), status);
        nirio_status_chain(dev_proxy.peek(0, reg_val), status);
        printf("* PCIe FPGA Signature = %x\n", reg_val);

        printf("\n[DMA Stream Status]\n");

        nirio_status_chain(dev_proxy.set_attribute(kRioAddressSpace, kRioAddressSpaceFpga), status);

        printf("----------------------------------------------------------------------------------");
        printf("\nChannel =>     |");
        for (uint32_t i = 0; i < 6; i++) {
            printf("%9d |", i);
        }
        printf("\n----------------------------------------------------------------------------------");
        printf("\nTX Status      |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x200 + (i * 16), reg_val), status);
            printf("%s |", reg_val==0 ? "     Good" : "    Error");
        }
        printf("\nRX Status      |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x400 + (i * 16), reg_val), status);
            printf("%s |", reg_val==0 ? "     Good" : "    Error");
        }
        printf("\nTX Frm Size    |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x204 + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\nRX Frm Size    |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x404 + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\nTX Pkt Count   |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x20C + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\nTX Samp Count  |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x208 + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\nRX Pkt Count   |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x40C + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\nRX Samp Count  |");
        for (uint32_t i = 0; i < 6; i++) {
            nirio_status_chain(dev_proxy.peek(0x408 + (i * 16), reg_val), status);
            printf("%9d |", reg_val);
        }
        printf("\n----------------------------------------------------------------------------------\n");
    }

    exit(EXIT_SUCCESS);
}


