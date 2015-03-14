#
# Copyright (c) 2011-2014 The Khronos Group Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and/or associated documentation files (the
# "Materials"), to deal in the Materials without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Materials, and to
# permit persons to whom the Materials are furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Materials.
#
# THE MATERIALS ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
#

set( TARGET_NAME_1 openvx-debug-lib )
set( TARGET_NAME_2 openvx-debug )

include_directories( BEFORE
                     ${CMAKE_CURRENT_SOURCE_DIR}
                     ${OPENVX_SOURCE_DIR}/include
                     ${OPENVX_SOURCE_DIR}/kernels/debug )
					 
add_library (${TARGET_NAME_1} vx_debug_lib.c)
set(LIBRARY_OUTPUT_PATH $ENV{LIBRARY_DIRECTORY})

FIND_SOURCES( vx_debug_lib.c )

add_library (${TARGET_NAME_2} SHARED ${SOURCE_FILES})
set(LIBRARY_OUTPUT_PATH $ENV{LIBRARY_DIRECTORY})

target_link_libraries( ${TARGET_NAME_2} openvx-helper openvx-debug_k-lib openvx )
