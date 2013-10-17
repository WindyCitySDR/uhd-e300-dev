{autogen_msg}

#include "{lvbitx_classname}_lvbitx.hpp"
#include <string>

namespace nifpga_interface {{

using namespace nirio_interface;

const char* {lvbitx_classname}_lvbitx::CONTROLS[] = {{{control_list}
}};

const char* {lvbitx_classname}_lvbitx::INDICATORS[] = {{{indicator_list}
}};

const char* {lvbitx_classname}_lvbitx::OUTPUT_FIFOS[] = {{{out_fifo_list}
}};

const char* {lvbitx_classname}_lvbitx::INPUT_FIFOS[] = {{{in_fifo_list}
}};

{lvbitx_classname}_lvbitx::{lvbitx_classname}_lvbitx(const char* option) :
    fpga_file_name("{lvbitx_path}" + std::string(option) + ".lvbitx")
{{
}}

const char* {lvbitx_classname}_lvbitx::get_bitfile_path() {{
    return fpga_file_name.c_str();
}}

const char* {lvbitx_classname}_lvbitx::get_signature() {{
    return "{lvbitx_signature}";
}}

size_t {lvbitx_classname}_lvbitx::get_input_fifo_count() {{
    return sizeof(INPUT_FIFOS)/sizeof(*INPUT_FIFOS);
}}

const char** {lvbitx_classname}_lvbitx::get_input_fifo_names() {{
    return INPUT_FIFOS;
}}

size_t {lvbitx_classname}_lvbitx::get_output_fifo_count() {{
    return sizeof(OUTPUT_FIFOS)/sizeof(*OUTPUT_FIFOS);
}}

const char** {lvbitx_classname}_lvbitx::get_output_fifo_names() {{
    return OUTPUT_FIFOS;
}}

size_t {lvbitx_classname}_lvbitx::get_control_count() {{
    return sizeof(CONTROLS)/sizeof(*CONTROLS);
}}

const char** {lvbitx_classname}_lvbitx::get_control_names() {{
    return CONTROLS;
}}

size_t {lvbitx_classname}_lvbitx::get_indicator_count() {{
    return sizeof(INDICATORS)/sizeof(*INDICATORS);
}}

const char** {lvbitx_classname}_lvbitx::get_indicator_names() {{
    return INDICATORS;
}}

void {lvbitx_classname}_lvbitx::init_register_info(nirio_interface::nirio_register_info_vtr& vtr) {{ {register_init}
}}

void {lvbitx_classname}_lvbitx::init_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr) {{ {fifo_init}
}}

}}
