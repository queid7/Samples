# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug  or make mpi
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=console
BIN_FOLDER=../build_$(EXE)
BIN_TO_SRC=../../$(EXE)
SRC_TO_WORK=../../work
SRC_TO_EXE=../../work/$(EXE)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	#cd ../yssample1;make build_dependencies
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory
	cp $(BIN_FOLDER)/build_linux/OgreFltk $(SRC_TO_EXE)

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1
	cp $(BIN_FOLDER)/build_linux_debug/OgreFltk $(SRC_TO_EXE)_debug

mpi: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	if ! test -f $(BIN_FOLDER)/build_linux_mpi/Makefile ; then mkdir -p $(BIN_FOLDER)/build_linux_mpi;cd $(BIN_FOLDER)/build_linux_mpi;export CC=mpicc;export CXX=mpiCC;cmake -G "Eclipse CDT4 - Unix Makefiles" -D UseMPI:BOOL=TRUE $(BIN_TO_SRC) ; fi
	#cd ../yssample1;make build_dependencies
	cd $(BIN_FOLDER)/build_linux_mpi;export CC=mpicc;export CXX=mpiCC;make --no-print-directory VERBOSE=1
	cp $(BIN_FOLDER)/build_linux_mpi/OgreFltk $(SRC_TO_EXE)_mpi
	cp $(BIN_FOLDER)/build_linux_mpi/OgreFltk $(SRC_TO_EXE)

	# after $ make mpi
	# $ cd ../yssample1;make build_dependencies
	# again $ make mpi

run: 
	#cd ../../work;./$(EXE) driverDynamic.lua ../Samples/QP_controller/lua/
	cd ../../work;./$(EXE) driverDynamicConsoleMuscle.lua ../Samples/ysscripts/samples/
console:
	cd ../../work;./$(EXE) console.lua ../Samples/QP_controller/lua/

testopt:
	cd ../../work;./$(EXE) MovingWindowOptimizerMuscle.lua ../Samples/ysscripts/samples/

mpitest:
	cd ../../work;mpiexec -np 4  -machinefile machines_test ./$(EXE)_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

mpirun:
	#cd ../../work;mpiexec -np 16  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/
	cd ../../work;mpiexec -np 16  ./$(EXE)_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

mpirun4:
	#cd ../../work;mpiexec -np 4  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/
	cd ../../work;mpiexec -np 4  ./$(EXE)_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

mpirun1:
	#cd ../../work;mpiexec -np 1  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/
	cd ../../work;mpiexec -np 1  ./$(EXE)_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

doc:
	vim ../../doc/index.wiki
cgdb:
	make debug
	cd ../../work;cgdb -- --args ./$(EXE)_debug driverDynamic.lua ../Samples/QP_controller/lua/
cgdb_testopt:
	cd ../../work;cgdb -- --args ./$(EXE)_debug MovingWindowOptimizer.lua ../Samples/QP_controller/lua/IPC_based/

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt
	
# launch vim
vim: 
	vim -c "set tags=./TAGS,../../BaseLib/TAGS,../../MainLib/TAGS,../../PhysicsLib/TAGS" lua/cart_pole.lua
gvim:
	gvim  -f -c "set tags=./TAGS,../../BaseLib/TAGS,../../MainLib/TAGS,../../PhysicsLib/TAGS" OgreFltk.cpp &

# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	cd ../yssample1;make build_dependencies

install_dependencies_ubuntu:
	sudo apt-get install libatlas-base-dev libdevil-dev libfreeimage-dev liblua5.1-dev libxml2-dev

install_dependencies_mpi:
	sudo apt-get install libopenmpi-dev openmpi-bin


# Rocks server (cent os)
#
# 1.add CentOSCentOS-Base.repo (with root account)
# $ vi /etc/yum.conf
# Add line: plugins=1
# $ cd /etc/yum.repos.d/
# $ wget https://lists.sdsc.edu/pipermail/npaci-rocks-discussion/attachments/20110519/c88f4c3c/CentOS-Base.repo
# $ vi /etc/yum.repos.d/CentOS-Base.repo
# replace gpgkey=...-CentOS-5 ot gpgkey=...-CentOS-6
#
# 2.add EPEL repo (with root account)
# (http://www.rackspace.com/knowledge_center/article/installing-rhel-epel-repo-on-centos-5x-or-6x)
# wget http://dl.fedoraproject.org/pub/epel/6/x86_64/epel-release-6-8.noarch.rpm
# wget http://rpms.famillecollet.com/enterprise/remi-release-6.rpm
# sudo rpm -Uvh remi-release-6*.rpm epel-release-6*.rpm
#
# $ yum repolist
# then you can see added repos
#
# 3.install stow (manage source code installs)
# $ yum install stow
# $ mkdir /usr/local/stow
#
# 4.install using yum
# $ sudo yum install cmake
# $ sudo yum install freeimage-devel.x86_64
# $ sudo yum install fltk-devel.x86_64
# $ sudo yum install lua-devel.x86_64
# $ sudo yum install atlas-sse3-devel.x86_64
# $ sudo yum install libxml2-devel.x86_64
# $ sudo yum install boost-openmpi-devel.x86_64
# $ sudo yum install lapack-devel.x86_64
# $ sudo yum install gsl-devel.x86_64
#
# 5.install from source Code
# - devil
# $ wget http://downloads.sourceforge.net/openil/DevIL-1.7.8.tar.gz
# $ tar -xzvf DevIL-1.7.8.tar.gz
# do [common things] with following change:
# $ ./configure --prefix=/usr/local/stow/<program name> --enable-ILU --enable-ILUT
#
# - ogre
# $ wget http://downloads.sourceforge.net/project/ogre/ogre/1.8/1.8.1/ogre_src_v1-8-1.tar.bz2?r=http%3A%2F%2Fwww.ogre3d.org%2Fdownload%2Fsource&ts=1387408521&use_mirror=jaist
# $ tar -xjvf ogre_src_v1-8-1.tar.bz2
# $ cd ogre_src_v1-8-1
# $ cmake
# $ make
# add following line to the cmake_install.cmake
# 	SET(CMAKE_INSTALL_PREFIX "/usr/local/stow/ogre_src_v1")
# $ sudo make install
# $ cd /usr/local/stow
# $ sudo stow ogre_src_v1
#
# [common things]
# $ cd <program name>
# $ ./configure --prefix=/usr/local/stow/<program name>
# $ make
# $ make install
# $ cd /usr/local/stow
# $ sudo stow <program name>
#
# 6. additional compile
#	cd ../QP_controller/qpOASES-3.0beta;cmake . ;make
#	cd ../../dependencies/clapack-3.2.1-CMAKE;mkdir -p build_;cd build_;cmake ..;make
#
# 7. additional setting & final build
# $ cd <your proj dir>
# $ vi Samples/Common_baselib.cmake
# replace set(LUA_LIB "lua5.1") to set(LUA_LIB "lua")
# $ sudo ldconfig /usr/local/lib
# $ make mpi
#
# 8. test
# $ cd <your proj dir>/Samples/console
# $ make mpirun1
# or
# $ qsub jobconsole.sh
#

