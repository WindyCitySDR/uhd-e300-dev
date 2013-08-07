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
# Conditionally configure the USRP-E200 support
########################################################################
find_package(UDev)

LIBUHD_REGISTER_COMPONENT("E200" ENABLE_E200 OFF "ENABLE_LIBUHD" OFF)

IF(ENABLE_E200)
    LIST(APPEND E200_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/e200_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/e200_io_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/e200_fifo_config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/e200_sysfs_hooks.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/e200_network.cpp
    )
    LIBUHD_APPEND_SOURCES(${E200_SOURCES})
    IF(UDEV_FOUND)
        INCLUDE_DIRECTORIES(${UDEV_INCLUDE_DIR})
        LIBUHD_APPEND_LIBS(${UDEV_LIBS})
        SET_SOURCE_FILES_PROPERTIES(
            ${E200_SOURCES}
            PROPERTIES COMPILE_DEFINITIONS "E200_NATIVE=1"
        )
    ENDIF(UDEV_FOUND)
ENDIF(ENABLE_E200)
