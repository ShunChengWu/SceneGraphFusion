###################
# UseEigen3.cmake #
###################
find_package(Eigen3 3.3.90 QUIET)
OPTION(WITH_EIGEN "Build with libEIGEN support?" OFF)
#MESSAGE("EIGEN!" ${WITH_EIGEN})
IF(WITH_EIGEN)
#  find_package(Eigen3)
#  MESSAGE("USE EIGEN " ${Eigen_FOUND} )
  IF(NOT Eigen3_FOUND)
    INCLUDE(ConfigureTimeDependency)
    add_configure_time_dependency(Eigen3)
    find_package(Eigen3 REQUIRED)
  ENDIF()
  include_directories(${EIGEN3_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_EIGEN)
ENDIF()


