####################
# LinkEigen3.cmake #
####################
IF(NOT TARGET OnnxRuntime)
  MESSAGE(WARNING "Trying to link onnxruntime but it was not found.")
else()
  TARGET_LINK_LIBRARIES(${targetname} PUBLIC OnnxRuntime)
endif()