// Auto-generated file: DO NOT EDIT!
// Generated from LabVIEW FPGA LVBITX image using "parse-lvbitx.py"

#ifndef NIFPGA_IMAGE_H_
#define NIFPGA_IMAGE_H_

#include <uhd/transport/nirio/nirio_interface.h>
#include <uhd/transport/nirio/nirio_resource_manager.h>

namespace nifpga_image {

static const char* BITFILE = "nifpga_image.lvbitx";
static const char* SIGNATURE = "82F2060A477C76260CE6698B04554731C4816FA7DC9EC80C5DE93C5EB7047626";

static const char* CONTROLS[] = {
    "DiagramReset",
    "ViControl",
    "InterruptEnable",
    "InterruptMask",
    "InterruptStatus",
};

static const char* INDICATORS[] = {
    "ViSignature",
};

static const char* OUTPUT_FIFOS[] = {
    "TX FIFO 0",
    "TX FIFO 1",
    "TX FIFO 2",
    "TX FIFO 3",
    "TX FIFO 4",
    "TX FIFO 5",
};

static const char* INPUT_FIFOS[] = {
    "RX FIFO 0",
    "RX FIFO 1",
    "RX FIFO 2",
    "RX FIFO 3",
    "RX FIFO 4",
    "RX FIFO 5",
};

static void initialize_register_info(nirio_interface::nirio_register_info_vtr& vtr)
{
    using namespace nirio_interface;

    vtr.push_back(nirio_register_info_t(0xfff4, INDICATORS[0], INDICATOR)); //"ViSignature"
    vtr.push_back(nirio_register_info_t(0xfffc, CONTROLS[0], CONTROL)); //"DiagramReset"
    vtr.push_back(nirio_register_info_t(0xfff8, CONTROLS[1], CONTROL)); //"ViControl"
    vtr.push_back(nirio_register_info_t(0xffe4, CONTROLS[2], CONTROL)); //"InterruptEnable"
    vtr.push_back(nirio_register_info_t(0xffec, CONTROLS[3], CONTROL)); //"InterruptMask"
    vtr.push_back(nirio_register_info_t(0xfff0, CONTROLS[4], CONTROL)); //"InterruptStatus"
}

static void initialize_fifo_info(nirio_interface::nirio_fifo_info_vtr& vtr)
{
    using namespace nirio_interface;

    vtr.push_back(nirio_fifo_info_t(0, INPUT_FIFOS[0], INPUT_FIFO, 0xff80, 255, SCALAR_U64, 64, 2)); //"RX FIFO 0"
    vtr.push_back(nirio_fifo_info_t(1, INPUT_FIFOS[1], INPUT_FIFO, 0xff40, 255, SCALAR_U64, 64, 2)); //"RX FIFO 1"
    vtr.push_back(nirio_fifo_info_t(2, INPUT_FIFOS[2], INPUT_FIFO, 0xff00, 255, SCALAR_U64, 64, 2)); //"RX FIFO 2"
    vtr.push_back(nirio_fifo_info_t(3, INPUT_FIFOS[3], INPUT_FIFO, 0xfec0, 255, SCALAR_U64, 64, 2)); //"RX FIFO 3"
    vtr.push_back(nirio_fifo_info_t(4, INPUT_FIFOS[4], INPUT_FIFO, 0xfe80, 255, SCALAR_U64, 64, 2)); //"RX FIFO 4"
    vtr.push_back(nirio_fifo_info_t(5, INPUT_FIFOS[5], INPUT_FIFO, 0xfe40, 255, SCALAR_U64, 64, 2)); //"RX FIFO 5"
    vtr.push_back(nirio_fifo_info_t(6, OUTPUT_FIFOS[0], OUTPUT_FIFO, 0xfe00, 261, SCALAR_U64, 64, 2)); //"TX FIFO 0"
    vtr.push_back(nirio_fifo_info_t(7, OUTPUT_FIFOS[1], OUTPUT_FIFO, 0xfdc0, 261, SCALAR_U64, 64, 2)); //"TX FIFO 1"
    vtr.push_back(nirio_fifo_info_t(8, OUTPUT_FIFOS[2], OUTPUT_FIFO, 0xfd80, 261, SCALAR_U64, 64, 2)); //"TX FIFO 2"
    vtr.push_back(nirio_fifo_info_t(9, OUTPUT_FIFOS[3], OUTPUT_FIFO, 0xfd40, 261, SCALAR_U64, 64, 2)); //"TX FIFO 3"
    vtr.push_back(nirio_fifo_info_t(10, OUTPUT_FIFOS[4], OUTPUT_FIFO, 0xfd00, 261, SCALAR_U64, 64, 2)); //"TX FIFO 4"
    vtr.push_back(nirio_fifo_info_t(11, OUTPUT_FIFOS[5], OUTPUT_FIFO, 0xfcc0, 261, SCALAR_U64, 64, 2)); //"TX FIFO 5"
}

}

#endif /* NIFPGA_IMAGE_H_ */

