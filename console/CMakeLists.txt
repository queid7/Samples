# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.6)

project (console)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../taesooLib/Samples")
include(Common_baselib) 
add_subdirectory(../../../taesooLib/BaseLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/BaseLib/build_${CMAKE_BUILD_TYPE}" )

add_definitions(-DSEGA)				
add_definitions(-DNO_GUI -DNO_OGRE)  
set(NO_GUI TRUE)
if (UseMPI)
	add_definitions(-DUSE_MPI)
	message("MPI enabled")
	add_subdirectory(../../../taesooLib/MainLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/MainLib/build_console_mpi_${CMAKE_BUILD_TYPE}" )
else()
	add_subdirectory(../../../taesooLib/MainLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/MainLib/build_console_${CMAKE_BUILD_TYPE}" )
endif()

include(Common_mainlib) 
add_subdirectory(../../../taesooLib/PhysicsLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/PhysicsLib/build_console_${CMAKE_BUILD_TYPE}" )
add_subdirectory(../../../taesooLib/ClassificationLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/ClassificationLib/build_console_${CMAKE_BUILD_TYPE}" )

include_directories (
	"${CMAKE_SOURCE_DIR}/liblbfgs-1.10/include"
	.
	../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/include
	../../../taesooLib/Samples/QP_controller/
	../../../taesooLib/Samples/classification/liblbfgs-1.10/include
	../../../taesooLib/Samples/scripts
	)
set(src
  "./OgreFltk.cpp"
  "./RigidBodyWin.cpp"
  "./RigidBodyWin.h"
  "../../../taesooLib/Samples/QP_controller/eiquadprog.hpp"
  "../../../taesooLib/Samples/QP_controller/quadprog.cpp"
  "../../../taesooLib/Samples/QP_controller/quadprog.h"
  "../../../taesooLib/Samples/QP_controller/quadprog_qpOASES.cpp"
  "../../../taesooLib/Samples/classification/cma/cmaes.c"
  "../../../taesooLib/Samples/classification/cma/CMAwrap.cpp"
  "../../../taesooLib/Samples/classification/cma/CMAwrap.h"
  )


set(luna_script "${CMAKE_SOURCE_DIR}")

add_custom_command(
	OUTPUT "${luna_script}/luna_QP.cpp"
	DEPENDS "${luna_script}/luna_QP.lua" "${LUNA_GEN}"
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_QP.lua"
  )

add_executable(OgreFltk
	${src}
	"${CMAKE_SOURCE_DIR}/luna_QP.cpp"
  )

if(WIN32)
  set(QPOASES_LIB
	  # build using cd qp<TAB>;cmake  . ;make
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/Release/qpOASES.lib"
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/Release/qpOASESextras.lib"
	  )
else()
  set(QPOASES_LIB
	  # build using cd qp<TAB>;cmake  . ;make
  "${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/libs/libqpOASES.a"
  "${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/libs/libqpOASESextras.a"
	"${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/classification/liblbfgs-1.10/lib/.libs/liblbfgs.a"
	dl
	  )
  endif()
target_link_libraries(
	OgreFltk 
	BaseLib 
	MainLib 
	PhysicsLib 
	ClassificationLib
	#${FLTK_LIBS}
	${IMAGE_LIBS}
	${OGRE_LIBS}
	${QPOASES_LIB}
	-lIL
	)
if(WIN32)
  add_custom_command(TARGET OgreFltk
                       POST_BUILD
                       COMMAND lua "${CMAKE_CURRENT_SOURCE_DIR}/make" install)
endif()
