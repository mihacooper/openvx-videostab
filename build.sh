#!/bin/bash

############################################
# enviroment variables
############################################

export MAKE_DEBUG_MODE=0 
export ONLY_MAKE=0
export ONLY_CMAKE=0

for par in $@
do
	if [ $par == "debug" ]; then
		MAKE_DEBUG_MODE=1
	fi
	
	if [ $par == "make" ]; then
		ONLY_MAKE=1
	fi

	if [ $par == "cmake" ]; then
		ONLY_CMAKE=1
	fi
done

export BIN_DIRECTORY=${PWD}/bin
export BUILD_DIRECTORY=${PWD}/build
export CMAKE_TOOLS_DIR=${PWD}/cmake-tools
export CMAKE_FLAGS="-DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8"
export CXX_FLAGS="-fPIC -Wno-unused-but-set-variable -Wno-unused-variable -Wno-write-strings"
export C_FLAGS="$CXX_FLAGS" #-std=c99

############################################
# cmake
############################################

rm -rf $BIN_DIRECTORY; mkdir $BIN_DIRECTORY;
if ! [ -d $BUILD_DIRECTORY ]; then
	mkdir $BUILD_DIRECTORY
	ONLY_MAKE=0
fi
cd $BUILD_DIRECTORY;

if [ $ONLY_MAKE -eq 0 ]; then
	cmake $CMAKE_FLAGS ../
fi

############################################
# make
############################################

if [ $ONLY_CMAKE -eq 0 ]; then
	if [ $MAKE_DEBUG_MODE -eq 1 ]; then
		make
	else
		make -j4
	fi
fi

