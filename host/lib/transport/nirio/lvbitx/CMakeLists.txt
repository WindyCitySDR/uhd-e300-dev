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

MACRO(LIBUHD_LVBITX_GEN_SOURCE lvbitx)
    GET_FILENAME_COMPONENT(lvbitxprefix ${lvbitx} NAME_WE)
    SET(OUTPUT_PATH_OPT --output-path=${CMAKE_CURRENT_BINARY_DIR})
    SET(IMAGES_PATH_OPT --uhd-images-path=${lvbitx_install_dir})

    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${lvbitxprefix}_lvbitx.hpp 
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${lvbitxprefix}_lvbitx.cpp 
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/template_lvbitx.hpp
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/template_lvbitx.cpp
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${lvbitx}
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/process-lvbitx.py ${OUTPUT_PATH_OPT} ${IMAGES_PATH_OPT} ${CMAKE_CURRENT_SOURCE_DIR}/${lvbitx}
        COMMENT "Generating ${CMAKE_CURRENT_BINARY_DIR}/${lvbitxprefix}_lvbitx.cpp"
    )

    #make libuhd depend on the output file
    LIBUHD_APPEND_SOURCES(${CMAKE_CURRENT_BINARY_DIR}/${lvbitxprefix}_lvbitx.cpp)
ENDMACRO(LIBUHD_LVBITX_GEN_SOURCE)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

########################################################################
# Add generation code here
########################################################################

SET(lvbitx_install_dir /usr/local/share/uhd/images)

LIBUHD_LVBITX_GEN_SOURCE(x300.lvbitx)
