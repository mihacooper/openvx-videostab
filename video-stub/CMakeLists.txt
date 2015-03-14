set( TARGET_NAME vx_videostub )

add_subdirectory(add_kernels)

include_directories( BEFORE
                     ${CMAKE_CURRENT_SOURCE_DIR} 
					 $ENV{OPENVX_SOURCE_DIR}/include 
					 $ENV{OPENVX_SOURCE_DIR}/debug )

FIND_SOURCES()

find_package( OpenCV REQUIRED )

add_executable (${TARGET_NAME} ${SOURCE_FILES})

set(EXECUTABLE_OUTPUT_PATH $ENV{BIN_DIRECTORY})

target_link_libraries( ${TARGET_NAME} ${OpenCV_LIBS} openvx vx_add_kernels)
