###################
# UseOpenCV.cmake #
###################
find_package(OpenCV 4.0.0 QUIET)
OPTION(WITH_OPENCV "Build with OpenCV support?" ${OpenCV_FOUND})

IF(WITH_OPENCV)
    MESSAGE("WITH_OPENCV: " ${WITH_OPENCV})
  IF(NOT OpenCV_FOUND)
      INCLUDE(ConfigureTimeDependency)
      add_configure_time_dependency(OpenCV)
      find_package(OpenCV REQUIRED)
  ENDIF()
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_OPENCV)
ELSE()
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_OPENCV)
ENDIF()
