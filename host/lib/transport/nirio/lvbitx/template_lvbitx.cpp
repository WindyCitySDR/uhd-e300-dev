{autogen_msg}

#include "{lvbitx_classname}_lvbitx.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <streambuf>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

namespace uhd {{ namespace niusrprio {{

const char* {lvbitx_classname}_lvbitx::CONTROLS[] = {{{control_list}
}};

const char* {lvbitx_classname}_lvbitx::INDICATORS[] = {{{indicator_list}
}};

const char* {lvbitx_classname}_lvbitx::OUTPUT_FIFOS[] = {{{out_fifo_list}
}};

const char* {lvbitx_classname}_lvbitx::INPUT_FIFOS[] = {{{in_fifo_list}
}};

{lvbitx_classname}_lvbitx::{lvbitx_classname}_lvbitx(const char* option) :
    _fpga_file_name("{lvbitx_path}" + std::string(option) + ".lvbitx")
{{
    std::ifstream lvbitx_stream(_fpga_file_name.c_str());
    if (lvbitx_stream.is_open()) {{
        std::string lvbitx_contents;
        lvbitx_stream.seekg(0, std::ios::end);
        lvbitx_contents.reserve(static_cast<size_t>(lvbitx_stream.tellg()));
        lvbitx_stream.seekg(0, std::ios::beg);
        lvbitx_contents.assign((std::istreambuf_iterator<char>(lvbitx_stream)), std::istreambuf_iterator<char>());
        try {{
            boost::smatch md5_match;
            if (boost::regex_search(lvbitx_contents, md5_match, boost::regex("<BitstreamMD5>([a-zA-Z0-9]{{32}})<\\/BitstreamMD5>", boost::regex::icase))) {{
                _bitstream_checksum = std::string(md5_match[1].first, md5_match[1].second);
            }}
        }} catch (boost::exception&) {{
            _bitstream_checksum = "";
        }}
    }}
    boost::to_upper(_bitstream_checksum);
}}

const char* {lvbitx_classname}_lvbitx::get_bitfile_path() {{
    return _fpga_file_name.c_str();
}}

const char* {lvbitx_classname}_lvbitx::get_signature() {{
    return "{lvbitx_signature}";
}}

const char* {lvbitx_classname}_lvbitx::get_bitstream_checksum() {{
    return _bitstream_checksum.c_str();
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

void {lvbitx_classname}_lvbitx::init_register_info(nirio_register_info_vtr& vtr) {{ {register_init}
}}

void {lvbitx_classname}_lvbitx::init_fifo_info(nirio_fifo_info_vtr& vtr) {{ {fifo_init}
}}

}}}}
