#include <uhd/transport/nirio/rpc/usrprio_rpc_common.hpp>
#include "usrprio/rpc_server.hpp"
#include "usrprio/rpc_logger.hpp"
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <uhd/transport/nirio/status.h>
#include <uhd/transport/nirio/nirio_interface.h>
#include "usrprio/NiFpga.h"
#include "usrprio/niusrprio.h"

using boost::asio::ip::tcp;

static inline uint8_t reverse_bits(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

uint32_t read_bitstream_from_file(
    const std::string& filename,
    boost::scoped_array<uint8_t>& buffer)
{
    using namespace std;

    size_t file_size = 0;
    ifstream file(filename.c_str(), ios::in|ios::binary|ios::ate);
    if (file.is_open())
    {
        file_size = static_cast<size_t>(file.tellg());
        buffer.reset(new uint8_t[file_size + 1]);

        file.seekg(0, ios::beg);
        file.read((char*)buffer.get(), file_size);
        file.close();
    }

    for (size_t i = 0; i < file_size; i++)
        buffer.get()[i] = reverse_bits(buffer.get()[i]);

    return file_size;
}

void handle_usrprio_function_call(
    usrprio_rpc::func_id_t func_id,
    const usrprio_rpc::func_args_reader_t& in_args,
    usrprio_rpc::func_args_writer_t& out_args,
    usrprio_rpc::client_id_t client_id)
{
    switch (func_id) {
        case usrprio_rpc::NIUSRPRIO_INITIALIZE: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_INITIALIZE for client %x") % client_id, LOG_VERBOSE)

            /*
            #define NIUSRPRIO_INITIALIZE_ARGS       \
                void
            */
            nirio_status status = NiRio_Status_Success;
            nirio_status_chain(NiFpga_Initialize(), status);
            nirio_status_chain(niusrprio_Initialize(), status);
            out_args << status;
        } break;

        case usrprio_rpc::NIUSRPRIO_FINALIZE: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_FINALIZE for client %x") % client_id, LOG_VERBOSE)

            /*
            #define NIUSRPRIO_FINALIZE_ARGS       \
                void
            */
            nirio_status status = NiRio_Status_Success;
            nirio_status_chain(NiFpga_Finalize(), status);
            nirio_status_chain(niusrprio_Finalize(), status);
            out_args << status;
        } break;


        case usrprio_rpc::NIUSRPRIO_ENUMERATE: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_ENUMERATE for client %x") % client_id, LOG_VERBOSE)

            usrprio_rpc::usrprio_device_info_vtr device_info_vtr;
            nirio_status status = NiRio_Status_Success;

            boost::uint64_t ndevs;
            nirio_status_chain(niusrprio_getNumberOfDevices(&ndevs), status);
            if (nirio_status_not_fatal(status) && ndevs > 0) {
                std::vector<uint32_t> nodes(static_cast<size_t>(ndevs));
                std::vector<uint64_t> serials(static_cast<size_t>(ndevs));

                nirio_status_chain(niusrprio_getDevicesInformation(ndevs, &nodes[0], &serials[0]), status);
                for(size_t i = 0; i < ndevs && nirio_status_not_fatal(status); i++) {
                    usrprio_rpc::usrprio_device_info info = usrprio_rpc::usrprio_device_info();
                    info.interface_num = nodes[i];
                    info.resource_name = "RIO" + boost::lexical_cast<std::string>(nodes[i]);
                    info.serial_num = boost::lexical_cast<std::string>(serials[i]);
                    //@TODO: The interface path should come from niusrprio / helper
                    info.interface_path = nirio_interface::niriok_proxy::get_interface_path(info.interface_num);

                    if (info.interface_num != ((uint32_t)-1) && !info.interface_path.empty())
                        device_info_vtr.push_back(info);
                }
            }

            out_args << status;
            if (nirio_status_not_fatal(status)) {
                out_args << static_cast<boost::uint32_t>(device_info_vtr.size());
                for (usrprio_rpc::usrprio_device_info_vtr::const_iterator it = device_info_vtr.begin();
                     it != device_info_vtr.end(); it++) {
                    out_args << *it;
                }
            }
        } break;


        case usrprio_rpc::NIUSRPRIO_OPEN_SESSION: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_OPEN_SESSION for client %x") % client_id, LOG_VERBOSE)

            /*
            #define NIUSRPRIO_OPEN_SESSION_ARGS     \
                const std::string& resource,        \
                const std::string& path,            \
                const std::string& signature,       \
                const boost::uint32_t& attribute,   \
                boost::uint32_t& session
            */
            std::string path, signature, resource;
            boost::uint32_t attribute, session;

            in_args >> resource;
            in_args >> path;
            in_args >> signature;
            in_args >> attribute;
            nirio_status status =
                NiFpga_Open(path.c_str(), signature.c_str(), resource.c_str(), attribute, &session);
            out_args << status;
            out_args << session;
        } break;

        case usrprio_rpc::NIUSRPRIO_CLOSE_SESSION: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_CLOSE_SESSION for client %x") % client_id, LOG_VERBOSE)

                    /*
            #define NIUSRPRIO_CLOSE_SESSION_ARGS    \
                const boost::uint32_t& session,     \
                const boost::uint32_t& attribute
            */
            boost::uint32_t attribute, session;

            in_args >> session;
            in_args >> attribute;
            nirio_status status = NiFpga_Close(session, attribute);
            out_args << status;
        } break;

        case usrprio_rpc::NIUSRPRIO_RESET_SESSION: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_RESET_SESSION for client %x") % client_id, LOG_VERBOSE)

                    /*
            #define NIUSRPRIO_RESET_SESSION_ARGS    \
                const boost::uint32_t& session
            */
            boost::uint32_t session;

            in_args >> session;
            nirio_status status = NiFpga_Reset(session);
            out_args << status;
        } break;

        case usrprio_rpc::NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH: {
            RPC_LOG(boost::format("Calling NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH for client %x") % client_id, LOG_VERBOSE)

            /*
            #define NIUSRPRIO_DOWNLOAD_FPGA_TO_FLASH_ARGS    \
                const boost::uint32_t& interface_num,
                const std::string& bitstream_path
            */
            std::string bitstream_path;
            boost::uint32_t interface_num;
            in_args >> interface_num;
            in_args >> bitstream_path;
            nirio_status status = NiRio_Status_Success;

            boost::scoped_array<uint8_t> buffer;
            uint32_t bytes_read = 0;
            if (!bitstream_path.empty()) {
                bytes_read = read_bitstream_from_file(bitstream_path, buffer);
            } else {
                status = NiRio_Status_CorruptBitfile;
            }

            uint64_t usrprio_hdl;
            nirio_status_chain(niusrprio_open(interface_num, &usrprio_hdl), status);
            nirio_status_chain(niusrprio_downloadToFlash(usrprio_hdl, buffer.get(), bytes_read), status);
            nirio_status_chain(niusrprio_close(usrprio_hdl), status);

            out_args << status;
        } break;

        default: {
            out_args << NiRio_Status_FeatureNotSupported;
        }
    }
}

int main(int argc, char *argv[])
{
    //Setup the program options
    std::string log_lvl;

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("log-level", po::value<std::string>(&log_lvl)->default_value("warning"), "Log level")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //Print the help message
    if (vm.count("help")){
        std::cout << boost::format("NI-USRPRIO RPC Server\n\n %s") % desc << std::endl;
        return ~0;
    }

    RPC_SET_LOG_LEVEL(log_lvl);
    RPC_SET_SVR_ID("usrprio_server")

    usrprio_rpc::rpc_server srv("localhost", "50000",
        usrprio_rpc::rpc_server::callback_func_t(&handle_usrprio_function_call));
    srv.run();

    exit(EXIT_SUCCESS);
}
