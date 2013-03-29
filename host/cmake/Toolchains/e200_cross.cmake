########################################################################
# Toolchain file for cross building for ARM Cortex A8 w/ NEON
# Usage:  cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchains/arm_cortex_a8_\
#cross.cmake -DENABLE_E100=ON -DENABLE_USRP_E_UTILS=TRUE -DENABLE_ORC=ON \
#-DCAMKE_INSTALL_PREFIX=./install ../
########################################################################
set( CMAKE_SYSTEM_NAME Linux )

set( CMAKE_C_COMPILER  /usr/local/oecore-x86_64/sysroots/x86_64-oesdk-linux/usr/bin/armv7a-vfp-neon-oe-linux-gnueabi/arm-oe-linux-gnueabi-gcc )
set( CMAKE_CXX_COMPILER  /usr/local/oecore-x86_64/sysroots/x86_64-oesdk-linux/usr/bin/armv7a-vfp-neon-oe-linux-gnueabi/arm-oe-linux-gnueabi-g++ )

set( CMAKE_CXX_FLAGS "-march=armv7-a -mtune=cortex-a9 -mfpu=neon -mfloat-abi=softfp --sysroot=/usr/local/oecore-x86_64/sysroots/armv7a-vfp-neon-oe-linux-gnueabi"  CACHE STRING "" FORCE )
set( CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE ) #same flags for C sources
set( CMAKE_LDFLAGS_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE ) #same flags for C sources

set( CMAKE_FIND_ROOT_PATH /usr/local/oecore-x86_64/sysroots/x86_64-oesdk-linux/
/usr/local/oecore-x86_64/sysroots/armv7a-vfp-neon-oe-linux-gnueabi/ )

#set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY )
set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )

set ( ORC_INCLUDE_DIRS /usr/local/oecore-x86_64/sysroots/armv7a-vfp-neon-oe-linux-gnueabi/usr/include/orc-0.4 )
set ( ORC_LIBRARY_DIRS /usr/local/oecore-x86_64/sysroots/armv7a-vfp-neon-oe-linux-gnueabi/usr/lib )
