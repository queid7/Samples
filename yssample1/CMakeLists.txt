# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.8)

project (yssample1)

# include ../Common.cmake
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../taesooLib/Samples")
include(Common_baselib) 
add_subdirectory(../../../taesooLib/BaseLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/BaseLib/build_${CMAKE_BUILD_TYPE}" )
add_subdirectory(../../../taesooLib/MainLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/MainLib/build_${CMAKE_BUILD_TYPE}" )
include(Common_mainlib) 
add_subdirectory(../../../taesooLib/PhysicsLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/PhysicsLib/build_${CMAKE_BUILD_TYPE}" )
add_subdirectory(../../../taesooLib/ClassificationLib "${CMAKE_SOURCE_DIR}/../../../taesooLib/ClassificationLib/build_${CMAKE_BUILD_TYPE}" )

include_directories (	
	"${CMAKE_SOURCE_DIR}/liblbfgs-1.10/include"
	.
	../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/include
	../../../taesooLib/Samples/classification/liblbfgs-1.10/include
	../../../taesooLib/Samples/scripts
	)

set(src
  "./OgreFltk.cpp"
  "./RigidBodyWin.cpp"
  "./RigidBodyWin.h"
  "./cma/cmaes.c"
  "./cma/CMAwrap.cpp"
  "./cma/CMAwrap.h"
quadprog.cpp
quadprog.h
quadprog_qpOASES.cpp
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
	"${CMAKE_SOURCE_DIR}/luna_classification.cpp" # important. without this, luna_gen doesn't run.
	"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
  )

set(QPOASES_LIB 
  # build using cd qp<TAB>;cmake  . ;make
  "${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/libs/libqpOASES.a"
  "${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/libs/libqpOASESextras.a"
	"${CMAKE_SOURCE_DIR}/../../../taesooLib/Samples/classification/liblbfgs-1.10/lib/.libs/liblbfgs.a"
  )
target_link_libraries(OgreFltk 
	BaseLib 
	MainLib 
	PhysicsLib 
	ClassificationLib
	${FLTK_LIBS}
	${IMAGE_LIBS}
	${OGRE_LIBS}
	${QPOASES_LIB}
	dl
	-lIL
	)
if(WIN32)
  add_custom_command(TARGET OgreFltk
                       POST_BUILD
                       COMMAND lua "${CMAKE_CURRENT_SOURCE_DIR}/make" install)
endif()
