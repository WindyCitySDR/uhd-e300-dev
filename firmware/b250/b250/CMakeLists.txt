#
# Copyright 2010-2012 Ettus Research LLC
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../../host/lib/usrp/b250)

list(APPEND b250_sources b250_init.c b250_main.c)

########################################################################
set(GEN_OUTPUTS_BIN_SIZE 0x7fff)

add_executable(b250_main.elf ${b250_sources})
target_link_libraries(b250_main.elf usrpfw)
GEN_OUTPUTS(b250_main.elf)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/b250_main.bin
    DESTINATION share/uhd/images
    RENAME usrp_b250_fw.bin
)
