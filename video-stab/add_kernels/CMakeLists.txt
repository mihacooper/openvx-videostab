set( TARGET_NAME vx_add_kernels )

include_directories( BEFORE
                     ${CMAKE_CURRENT_SOURCE_DIR} 
					 $ENV{OPENVX_SOURCE_DIR}/include
					 $ENV{OPENVX_SOURCE_DIR}/debug)

FIND_SOURCES()

find_package(OpenCV REQUIRED core imgproc video highgui calib3d)

add_definitions( -DVX_ADD_KERNELS_LIBRARY )

add_library(${TARGET_NAME} SHARED ${SOURCE_FILES})

set(LIBRARY_OUTPUT_PATH $ENV{LIBRARY_DIRECTORY})

target_link_libraries( ${TARGET_NAME} ${OpenCV_LIBS} openvx )
