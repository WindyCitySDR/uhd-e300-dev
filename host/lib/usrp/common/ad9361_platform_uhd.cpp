//
// Copyright 2013 Ettus Research LLC
//

#include <uhd/utils/msg.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <stdint.h>
#include <ad9361_platform.h>
#include <uhd/types/serial.hpp>
#include "ad9361_ctrl.hpp"
#include <stdio.h>

ad9361_device_t* get_ad9361_device(uint64_t handle)
{
    return reinterpret_cast<ad9361_device_t*>(reinterpret_cast<void*>(handle));
}

#define AD9361_SPI_WRITE_CMD    0x00800000
#define AD9361_SPI_READ_CMD     0x00000000
#define AD9361_SPI_ADDR_MASK    0x003FFF00
#define AD9361_SPI_ADDR_SHIFT   8
#define AD9361_SPI_DATA_MASK    0x000000FF
#define AD9361_SPI_DATA_SHIFT   0
#define AD9361_SPI_SLAVE_NUM    0x1
#define AD9361_SPI_NUM_BITS     24

uint8_t read_ad9361_reg(ad9361_device_t* device, uint32_t reg)
{
    if (device && device->xact_iface) {
        uhd::spi_iface* spi_iface = reinterpret_cast<uhd::spi_iface*>(device->xact_iface);

        uhd::spi_config_t config;
        config.mosi_edge = uhd::spi_config_t::EDGE_FALL;
        config.miso_edge = uhd::spi_config_t::EDGE_FALL;    //TODO (Ashish): FPGA SPI workaround. This should be EDGE_RISE

        uint32_t rd_word = AD9361_SPI_READ_CMD |
                           ((uint32_t(reg) << AD9361_SPI_ADDR_SHIFT) & AD9361_SPI_ADDR_MASK);

        uint32_t val = (spi_iface->read_spi(AD9361_SPI_SLAVE_NUM, config, rd_word, AD9361_SPI_NUM_BITS));
        val &= 0xFF;

        return static_cast<uint8_t>(val);
    } else {
        return 0;
    }
}

void write_ad9361_reg(ad9361_device_t* device, uint32_t reg, uint8_t val)
{
    if (device && device->xact_iface) {
        uhd::spi_iface* spi_iface = reinterpret_cast<uhd::spi_iface*>(device->xact_iface);

        uhd::spi_config_t config;
        config.mosi_edge = uhd::spi_config_t::EDGE_FALL;
        config.miso_edge = uhd::spi_config_t::EDGE_FALL;    //TODO (Ashish): FPGA SPI workaround. This should be EDGE_RISE

        uint32_t wr_word = AD9361_SPI_WRITE_CMD |
                           ((uint32_t(reg) << AD9361_SPI_ADDR_SHIFT) & AD9361_SPI_ADDR_MASK) |
                           ((uint32_t(val) << AD9361_SPI_DATA_SHIFT) & AD9361_SPI_DATA_MASK);
        spi_iface->write_spi(
            AD9361_SPI_SLAVE_NUM, config, wr_word, AD9361_SPI_NUM_BITS);

        //TODO (Ashish): Is this necessary? The FX3 firmware does it right now but for
        read_ad9361_reg(device, reg);
    }
}

typedef union
{
    double d;
    uint32_t x[2];
} ad9361_double_union_t;

void ad9361_double_pack(const double input, uint32_t output[2])
{
    ad9361_double_union_t p = {};
    p.d = input;
    output[0] = p.x[0];
    output[1] = p.x[1];
}

double ad9361_double_unpack(const uint32_t input[2])
{
    ad9361_double_union_t p = {};
    p.x[0] = input[0];
    p.x[1] = input[1];
    return p.d;
}

double ad9361_sqrt(double val)
{
    return std::sqrt(val);
}

void ad9361_msleep(const uint32_t millis)
{
    usleep(millis*1000);
}

int ad9361_floor_to_int(double val)
{
    return static_cast<int>(std::floor(val));
}

int ad9361_ceil_to_int(double val)
{
    return static_cast<int>(std::ceil(val));
}

