####################
# LinkEigen3.cmake #
####################

if(EIGEN3_INCLUDE_DIR)
  target_include_directories(${targetname} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_INCLUDE_DIR)