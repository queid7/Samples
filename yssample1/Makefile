# Usage:
# for the first time: make install_dependencies; make
# from next time, just make or make debug 
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=yssample1
BIN_FOLDER=../build_$(EXE)
BIN_TO_SRC=../../$(EXE)
SRC_TO_WORK=../../work
SRC_TO_EXE=../../work/$(EXE)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory
	cp $(BIN_FOLDER)/build_linux/OgreFltk $(SRC_TO_EXE)

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1
	cp $(BIN_FOLDER)/build_linux_debug/OgreFltk $(SRC_TO_EXE)_debug

run: 
	cd $(SRC_TO_WORK);./$(EXE)
cgdb:
	make debug
	cd $(SRC_TO_WORK);cgdb ./$(EXE)_debug
doc:
	vi ../../doc/index.wiki

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt
	

# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	cd ../../../taesooLib;make install_dependencies_ubuntu
	make build_dependencies

build_dependencies:
	cd ../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta;cmake .;make
	cd ../../../taesooLib/Samples/classification/liblbfgs-1.10;./configure;make
	cd ../../../taesooLib;make build_dependencies
