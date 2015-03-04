#!/bin/bash

############################################
# enviroment variables
############################################

export MAKE_DEBUG_MODE=0 #  1 or 0
if [ $1 == "debug" ]; then
	MAKE_DEBUG_MODE=1
fi

export BIN_DIRECTORY=${PWD}/bin
export BUILD_DIRECTORY=${PWD}/build
export CMAKE_TOOLS_DIR=${PWD}/cmake-tools
export CMAKE_FLAGS="-DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8"
export CXX_FLAGS="-fPIC -Wno-unused-but-set-variable -Wno-unused-variable"
export C_FLAGS="$CXX_FLAGS" #-std=c99

############################################
# cmake
############################################

rm -rf $BUILD_DIRECTORY;rm -rf $BIN_DIRECTORY; mkdir $BUILD_DIRECTORY; cd $BUILD_DIRECTORY;
cmake $CMAKE_FLAGS ../

############################################
# make
############################################

if [ $MAKE_DEBUG_MODE -eq 1 ]; then
	make
else
	make -j4
fi

