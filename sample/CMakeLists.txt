set( TARGET_NAME vx_sample )

include_directories( BEFORE
                     ${CMAKE_CURRENT_SOURCE_DIR} 
					 $ENV{OPENVX_SOURCE_DIR}/include )

FIND_SOURCES()

add_executable (${TARGET_NAME} ${SOURCE_FILES})
set(EXECUTABLE_OUTPUT_PATH $ENV{BIN_DIRECTORY})
