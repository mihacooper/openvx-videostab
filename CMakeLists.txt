cmake_minimum_required(VERSION 2.8)

include($ENV{CMAKE_TOOLS_DIR}/CMakeFuncs.txt)

set(CMAKE_CXX_FLAGS $ENV{CXX_FLAGS})
set(CMAKE_C_FLAGS $ENV{C_FLAGS})

if($ENV{MAKE_DEBUG_MODE})
	set(CMAKE_VERBOSE_MAKEFILE true)
endif()

add_subdirectory(openvx-lib)
add_subdirectory(sample)
