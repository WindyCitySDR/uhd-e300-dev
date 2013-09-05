#include "rpc_logger.hpp"

#ifdef RPC_LOGGING_ENABLED
namespace usrprio_rpc {
    log_level logger::g_log_level = LOG_NONE;
    std::string logger::g_server_id = "rpc_server";
}
#endif
