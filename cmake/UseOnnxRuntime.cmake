###################
# UseEigen3.cmake #
###################

FIND_PACKAGE(OnnxRuntime)

OPTION(WITH_ONNX "Build with libEIGEN support?" ${OnnxRuntime_FOUND})

IF(WITH_ONNX)
  include_directories(${OnnxRuntime_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_ONNX)
ENDIF()


