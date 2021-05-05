cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

UNSET(COMPILE_WITH_CUDA)
###############################################################################
### -- 3rd party
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(JSONCPP jsoncpp) # For Standford2D3DS

#find_package(OpenCV REQUIRED)
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIB_DIRS})

###########################################################################################
SET(targetname ScanNetLoader)
file(GLOB_RECURSE srcs *.cpp)
file(GLOB_RECURSE headers *.h *.hpp)
file(GLOB_RECURSE protos *.proto)

SET(sources
	${srcs}
	${protos}
)

SET(headers
	${headers}
)

ADD_LIBRARY(${targetname} ${sources} ${headers})
#target_link_libraries(${targetname}
#		PUBLIC ${JSONCPP_LIBRARIES}
#		PUBLIC ${OpenCV_LIBS}
#)

#target_include_directories(${targetname} PUBLIC ${PROJECT_SOURCE_DIR}/ThirdParty/stb)
#target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
#target_compile_definitions(${targetname} PUBLIC -DWITH_OPENCV PUBLIC -DWITH_REALSENSE2)
#set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 11)
#target_include_directories(${targetname} PUBLIC ./include)
#target_include_directories(${targetname} PUBLIC ./Loader)

