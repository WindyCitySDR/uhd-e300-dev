//
// Copyright 2014 Ettus Research LLC
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

#include "e300_eeprom_manager.hpp"
#include <uhd/types/mac_addr.hpp>
#include <uhd/utils/byteswap.hpp>

namespace uhd { namespace usrp { namespace e300 {

static const std::string _bytes_to_string(const uint8_t* bytes, size_t max_len)
{
    std::string out;
    for (size_t i = 0; i < max_len; i++) {
        if (bytes[i] < 32 or bytes[i] > 127) return out;
        out += bytes[i];
    }
    return out;
}

static void _string_to_bytes(const std::string &string, size_t max_len, uint8_t* buffer)
{
    byte_vector_t bytes;
    const size_t len = std::min(string.size(), max_len);
    for (size_t i = 0; i < len; i++){
        buffer[i] = string[i];
    }
    if (len < max_len - 1)
        buffer[len] = '\0';
}

e300_eeprom_manager::e300_eeprom_manager(i2c::sptr i2c) : _i2c(i2c)
{
    read_mb_eeprom();
}

e300_eeprom_manager::~e300_eeprom_manager(void)
{
}

const mboard_eeprom_t& e300_eeprom_manager::read_mb_eeprom(void)
{
    boost::mutex::scoped_lock(_mutex);

    std::vector<boost::uint8_t> bytes;
    bytes.resize(sizeof(mb_eeprom_map_t));
    mb_eeprom_map_t *map_ptr = reinterpret_cast<mb_eeprom_map_t*>(&bytes[0]);
    memset(map_ptr, 0xff, sizeof(mb_eeprom_map_t));

    // get the old contents
    for(size_t i = 0; i < sizeof(mb_eeprom_map_t); i++)
        bytes[i] = _i2c->get_i2c_reg(MB_ADDR, i);

    mb_eeprom_map_t &map = *map_ptr;

    _mb_eeprom["product"] = str(
        boost::format("E%s")
        % (boost::lexical_cast<std::string>(
            uhd::htonx<boost::uint16_t>(map.hw_product))));

    _mb_eeprom["revision"] = boost::lexical_cast<std::string>(
        uhd::htonx<boost::uint16_t>(map.hw_revision));
    _mb_eeprom["serial"] = _bytes_to_string(
        map.serial, MB_SERIAL_LEN);

    byte_vector_t mac_addr(map.mac_addr, map.mac_addr + 6);
    _mb_eeprom["mac-addr"] = mac_addr_t::from_bytes(mac_addr).to_string();

    _mb_eeprom["name"] = _bytes_to_string(
        map.user_name, MB_NAME_LEN);

    return _mb_eeprom;
}

void e300_eeprom_manager::write_mb_eeprom(const mboard_eeprom_t& eeprom)
{
    boost::mutex::scoped_lock(_mutex);
    _mb_eeprom = eeprom;
    std::vector<boost::uint8_t> bytes;
    bytes.resize(sizeof(mb_eeprom_map_t));


    mb_eeprom_map_t *map_ptr = reinterpret_cast<mb_eeprom_map_t*>(&bytes[0]);
    memset(map_ptr, 0xff, sizeof(mb_eeprom_map_t));

    // get the old contents
    for(size_t i = 0; i < sizeof(mb_eeprom_map_t); i++)
        bytes[i] = _i2c->get_i2c_reg(MB_ADDR, i);

    mb_eeprom_map_t &map = *map_ptr;

    if (_mb_eeprom.has_key("product")) {
        // TODO: This is a hack, do this nice
        if (_mb_eeprom["product"] == "E300")
            map.hw_product = uhd::htonx<boost::uint16_t>(300);
        else
            map.hw_product = uhd::htonx<boost::uint16_t>(310);
    }
    if (_mb_eeprom.has_key("revision")) {
        map.hw_revision = uhd::htonx<boost::uint16_t>(
            boost::lexical_cast<boost::uint16_t>(_mb_eeprom["revision"]));
    }
    if (_mb_eeprom.has_key("serial")) {
        _string_to_bytes(_mb_eeprom["serial"], MB_SERIAL_LEN, map.serial);
    }
    if (_mb_eeprom.has_key("mac-addr")) {
        byte_vector_t mac_addr = mac_addr_t::from_string(_mb_eeprom["mac-addr"]).to_bytes();
        std::copy(mac_addr.begin(), mac_addr.end(), map.mac_addr);
    }

    //store the name
    if (_mb_eeprom.has_key("name")) {
        _string_to_bytes(_mb_eeprom["name"], MB_NAME_LEN, map.user_name);
    }

    for(size_t i = 0; i < sizeof(mb_eeprom_map_t); i++)
        _i2c->set_i2c_reg(MB_ADDR, i, bytes[i]);

}


}}} // namespace
