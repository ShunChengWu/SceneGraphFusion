cmake_minimum_required(VERSION 3.13)
project(GraphSLAM)
###########
# Options #
###########
OPTION(BUILD_GRAPHPRED "compile with scene graph prediction" OFF)
OPTION(BUILD_GUI "compile with GUI" OFF)
OPTION(USE_RENDEREDVIEW "Enable generating rendered view for ScanNet and 3RScan dataset" ON)
###################
# Project setting #
###################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
#################################
# Specify the CMake module path #
#################################
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
SET(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR} ${CMAKE_PREFIX_PATH})

#################################
# Add additional compiler flags #
#################################
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()
IF(NOT MSVC_IDE)
	SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
	SET(CMAKE_CXX_FLAGS "-fPIC ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()
############
# External #
############
SET(WITH_OPENCV TRUE)
SET(WITH_EIGEN TRUE)
SET(WITH_TINYPLY TRUE)
SET(WITH_JSON TRUE)
INCLUDE(Usetinyply)
INCLUDE(UseJson)
INCLUDE(UseOpenMP)
if(APPLE)
	set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_osx.a)
else()
	set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_linux.a)
endif()
################
# Include dirs #
################
INCLUDE_DIRECTORIES(ORUtils/include)
######################
# Add subdirectories #
######################
IF(BUILD_GUI)
	ADD_SUBDIRECTORY(libGUI3D)
	ADD_SUBDIRECTORY(libGraphSLAMGUI)
ENDIF(BUILD_GUI)
ADD_SUBDIRECTORY(libDataLoader)
ADD_SUBDIRECTORY(libGraphSLAM)
ADD_SUBDIRECTORY(app)

