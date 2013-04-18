//
// Copyright 2011-2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/serial.hpp>
#include <sys/ioctl.h> //ioctl
#include <fcntl.h> //open, close
#include <poll.h> //poll
#include <boost/thread/thread.hpp> //sleep
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <fstream>

using namespace uhd;

static const int CAT_CS = 64;
static const int CAT_MOSI = 63;
static const int CAT_SCLK = 62;
static const int CAT_MISO = 66;

/***********************************************************************
 * Sysfs GPIO wrapper class
 **********************************************************************/
class e200_gpio{
public:
    e200_gpio(const int num, const std::string &dir) : _num(num){
        UHD_MSG(status) << "open GPIO " << num << " " << dir << std::endl;
        this->set_xport("export");
        this->set_dir(dir);
        _value_file.open(str(boost::format("/sys/class/gpio/gpio%d/value") % num).c_str(), std::ios_base::in | std::ios_base::out);
    }
    ~e200_gpio(void){
        _value_file.close();
        this->set_dir("in");
        this->set_xport("unexport");
    }
    void operator()(const int val){
        _value_file << val << std::endl << std::flush;
    }
    int operator()(void){
        std::string val;
        std::getline(_value_file, val);
        _value_file.seekg(0);
        return int(val.at(0) - '0') & 0x1;
    }
private:
    void set_xport(const std::string &xport){
        std::ofstream export_file(("/sys/class/gpio/" + xport).c_str());
        export_file << _num << std::endl << std::flush;
        export_file.close();
    }
    void set_dir(const std::string &dir){
        std::ofstream dir_file(str(boost::format("/sys/class/gpio/gpio%d/direction") % _num).c_str());
        dir_file << dir << std::endl << std::flush;
        dir_file.close();
    }
    const int _num;
    std::fstream _value_file;
};

/***********************************************************************
 * Aux spi implementation
 **********************************************************************/
class e200_aux_spi_iface_impl : public spi_iface{
public:
    e200_aux_spi_iface_impl(void):
        spi_sclk_gpio(CAT_SCLK, "out"),
        spi_sen_gpio(CAT_CS, "out"),
        spi_mosi_gpio(CAT_MOSI, "out"),
        spi_miso_gpio(CAT_MISO, "in"){}

    boost::uint32_t transact_spi(
        int, const spi_config_t &, //not used params
        boost::uint32_t bits,
        size_t num_bits,
        bool readback
    ){
        boost::uint32_t rb_bits = 0;
        this->spi_sen_gpio(0);

        for (size_t i = 0; i < num_bits; i++){
            this->spi_sclk_gpio(1);
            this->spi_mosi_gpio((bits >> (num_bits-i-1)) & 0x1);
            this->spi_sclk_gpio(0);
            if (readback) rb_bits = (rb_bits << 1) | this->spi_miso_gpio();
        }

        this->spi_mosi_gpio(0);
        this->spi_sen_gpio(1);
        return rb_bits;
    }

private:
    e200_gpio spi_sclk_gpio, spi_sen_gpio, spi_mosi_gpio, spi_miso_gpio;
};

uhd::spi_iface::sptr e200_make_aux_spi_iface(void)
{
    return uhd::spi_iface::sptr(new e200_aux_spi_iface_impl());
}
