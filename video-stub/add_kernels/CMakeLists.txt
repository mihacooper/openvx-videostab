set( TARGET_NAME vx_add_kernels )

include_directories( BEFORE
                     ${CMAKE_CURRENT_SOURCE_DIR} 
					 $ENV{OPENVX_SOURCE_DIR}/include
					 $ENV{OPENVX_SOURCE_DIR}/debug)

FIND_SOURCES()

add_definitions( -DVX_IAP_SAMPLE_KERNELS_LIBRARY )

add_library(${TARGET_NAME} SHARED ${SOURCE_FILES})

set(LIBRARY_OUTPUT_PATH $ENV{BIN_DIRECTORY})

target_link_libraries( ${TARGET_NAME} openvx )
