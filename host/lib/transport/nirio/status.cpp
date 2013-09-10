/*
 * status.cpp
 *
 *  Created on: Sep 9, 2013
 *      Author: ashish
 */

#include <uhd/transport/nirio/status.h>
#include <boost/format.hpp>

namespace nirio_interface {

#define NIRIO_ERR_INFO(CONST_NAME, ERR_CODE, ERR_MSG) \
    nirio_err_info(ERR_CODE, ERR_MSG),

const nirio_err_info nirio_err_info::NIRIO_ERROR_TABLE[] = {
    #include "../../../include/uhd/transport/nirio/nirio_err_template.h"
};

#undef NIRIO_ERR_INFO

const size_t nirio_err_info::NIRIO_ERROR_TABLE_SIZE = sizeof(NIRIO_ERROR_TABLE)/sizeof(*NIRIO_ERROR_TABLE);

const std::string lookup_err_msg(nirio_status code) {
    std::string error_msg = (boost::format("Unknown error. (Error code %d)") % code).str();
    for (size_t i = 0; i < nirio_err_info::NIRIO_ERROR_TABLE_SIZE; i++) {
        if (nirio_err_info::NIRIO_ERROR_TABLE[i].code == code) {
            error_msg = (boost::format("%s (Error code %d)") % nirio_err_info::NIRIO_ERROR_TABLE[i].msg % code).str();
            break;
        }
    }
    return error_msg;
}

void nirio_status_to_exception(const nirio_status& status, const std::string& message) {
    if (nirio_status_fatal(status)) {
        throw uhd::runtime_error((boost::format("%s %s") % message % lookup_err_msg(status)).str());
    }
}

}



