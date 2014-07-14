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

#ifndef INCLUDED_UHD_TYPES_SERIAL_HPP
#define INCLUDED_UHD_TYPES_SERIAL_HPP

#include <uhd/config.hpp>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

namespace uhd{

    /*!
     * Represents a stream ID (SID).
     *
     * A stream ID (SID) is an identifier for data.
     * It is a 32-Bit value which consistst of 16 Bits
     * for the source address and 16 Bits for the destination
     * address.
     * Every address is split into two parts: A remote part
     * and a local part, which each are 8 Bits.
     * A typical representation is in an IPv4-fashion, e.g.
     * 2.3.0.6.
     *
     * The format is:
     *     REMOTE_SRC.LOCAL_SRC.REMOTE_DST.LOCAL_DST
     */
    class UHD_API sid_t
    {
    public:
        sid_t(boost::uint32_t sid);
        sid_t();

        //! Return a string like this: 2.12.0.18 (decimal numbers)
        std::string to_pp_string() const;
        //! Return a string like this: 02.B.00.12 (hexadecimal numbers)
        std::string to_pp_string_hex() const;

        //! Returns true if this actually holds a valid SID
        bool is_set() const { return _set; };

        // Getters
        //! Alias for get_sid()
        inline boost::uint32_t get() const { return get_sid(); };
        inline boost::uint32_t get_sid() const { return _set ? _sid : 0; };
        //! Return the source address of this SID
        //  (the first 16 Bits)
        inline boost::uint32_t get_src_address() const {
            return (_sid >> 16) & 0xFFFF;
        }
        //! Return the destination address of this SID
        //  (the last 16 Bits)
        inline boost::uint32_t get_dst_address() const {
            return _sid & 0xFFFF;
        }

        //! Return remote part of source address
        inline boost::uint32_t get_remote_src_address() const {
            return (get_src_address() >> 8) & 0xFF;
        }
        //! Return local part of source address
        inline boost::uint32_t get_local_src_address() const {
            return get_src_address() & 0xFF;
        }
        //! Return remote part of destination address
        inline boost::uint32_t get_remote_dst_address() const {
            return (get_dst_address() >> 8) & 0xFF;
        }
        //! Return local part of destination address
        inline boost::uint32_t get_local_dst_address() const {
            return get_dst_address() & 0xFF;
        }

        // Setters

        //! Alias for set_sid()
        void set(boost::uint32_t new_sid) { set_sid(new_sid); };
        void set_sid(boost::uint32_t new_sid);
        //! Return the source address of this SID
        //  (the first 16 Bits)
        void set_src_address(boost::uint32_t new_addr);
        //! Return the destination address of this SID
        //  (the last 16 Bits)
        void set_dst_address(boost::uint32_t new_addr);
        //! Return remote part of source address
        void set_remote_src_address(boost::uint32_t new_addr);
        //! Return local part of source address
        void set_local_src_address(boost::uint32_t new_addr);
        //! Return remote part of destination address
        void set_remote_dst_address(boost::uint32_t new_addr);
        //! Return local part of source address
        void set_local_dst_address(boost::uint32_t new_addr);

        // Overloaded operators

        sid_t operator = (boost::uint32_t new_sid) {
            set_sid(new_sid);
            return *this;
        }

        sid_t operator = (sid_t &sid) {
            set_sid(sid.get_sid());
            return *this;
        }

        bool operator == (const sid_t &sid) const {
            return (not _set and not sid.is_set()) or (_sid == sid.get_sid());
        }

        bool operator == (boost::uint32_t sid) const {
            return _set and _sid == sid;
        }

        // overloaded type casts are tricky, but for now we'll need them
        // for backward compatibility. consider them deprecated.

        //! If the SID is not set, always returns zero.
        //  Use is_set() to check if the return value is valid.
        operator boost::uint32_t() const {
            return get();
        }

        operator bool() const {
            return _set;
        }

    private:
        boost::uint32_t _sid;
        bool _set;
    };

    inline std::ostream& operator<< (std::ostream& out, const sid_t &sid) {
        out << sid.to_pp_string();
        return out;
    }

} //namespace uhd

#endif /* INCLUDED_UHD_TYPES_SERIAL_HPP */
// vim: sw=4 et:
