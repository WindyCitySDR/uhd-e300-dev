#
# Copyright 2013 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# This file included, use CMake directory variables
########################################################################

########################################################################
# Include subdirectories (different than add)
########################################################################
INCLUDE_SUBDIRECTORY(lvbitx)

########################################################################
# Append to the list of sources for lib uhd
########################################################################

LIBUHD_APPEND_SOURCES(
    ${CMAKE_CURRENT_SOURCE_DIR}/nifpga_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/nirio_fifo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/nirio_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/nirio_resource_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NiFpga/NiFpga.c
    ${CMAKE_CURRENT_SOURCE_DIR}/NiFpga/niusrprio.c
)

IF(WIN32)
    LIBUHD_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/nirio_driver_iface_win.cpp)
    #@TODO: Ashish: Setupapi should be removed when enumeration is done using niusrprio / helper.
    LIBUHD_APPEND_LIBS(Setupapi)
ELSE()
    LIBUHD_APPEND_SOURCES(${CMAKE_CURRENT_SOURCE_DIR}/nirio_driver_iface_linux.cpp)
ENDIF()
