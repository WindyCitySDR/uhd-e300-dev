#include "usrprio/usrprio_rpc_handler.hpp"
#include "usrprio/rpc_server.hpp"
#include "usrprio/rpc_logger.hpp"
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <uhd/transport/nirio/status.h>

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

    static const boost::uint32_t RPC_LISTEN_PORT = 50000;

    nirio_status status = 0;
    usrprio_rpc::usrprio_rpc_handler handler(status);
    if (nirio_status_not_fatal(status)) {
        usrprio_rpc::rpc_server server(
            RPC_LISTEN_PORT,
            usrprio_rpc::rpc_server::callback_func_t(
                boost::bind(&usrprio_rpc::usrprio_rpc_handler::handle_function_call, &handler, _1, _2, _3, _4)));
        if (server.run()) status = NiRio_Status_SoftwareFault;
    }

    exit(status);
}
