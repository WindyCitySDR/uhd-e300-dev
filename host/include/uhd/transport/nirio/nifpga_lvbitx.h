
#ifndef NIFPGA_LVBITX_H_
#define NIFPGA_LVBITX_H_

#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nirio_resource_manager.h>
#include <boost/smart_ptr.hpp>

namespace nifpga_interface {

class nifpga_lvbitx {
public:
    typedef boost::shared_ptr<nifpga_lvbitx> sptr;

    virtual const char* get_bitfile_path() = 0;
    virtual const char* get_signature() = 0;

    virtual size_t get_input_fifo_count() = 0;
    virtual const char** get_input_fifo_names() = 0;

    virtual size_t get_output_fifo_count() = 0;
    virtual const char** get_output_fifo_names() = 0;

    virtual size_t get_control_count() = 0;
    virtual const char** get_control_names() = 0;

    virtual size_t get_indicator_count() = 0;
    virtual const char** get_indicator_names() = 0;

    virtual void init_register_info(nirio_interface::nirio_register_info_vtr& vtr) = 0;
    virtual void init_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr) = 0;
};
}

#endif /* NIFPGA_LVBITX_H_ */

