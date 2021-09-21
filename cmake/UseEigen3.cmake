###################
# UseEigen3.cmake #
###################
find_package(Eigen3 3.4 QUIET)
OPTION(WITH_EIGEN "Build with libEIGEN support?" OFF)
#MESSAGE("EIGEN!" ${WITH_EIGEN})
IF(WITH_EIGEN)
  IF(NOT Eigen3_FOUND)
    INCLUDE(ConfigureTimeDependency)
    add_configure_time_dependency(Eigen3)
    find_package(Eigen3 REQUIRED)
  ENDIF()

  IF(TARGET Eigen3::Eigen)
    GET_TARGET_PROPERTY(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  ENDIF()
  include_directories(${EIGEN3_INCLUDE_DIR})
  MESSAGE("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_EIGEN)
ENDIF()


