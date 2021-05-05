
FIND_PATH(JSON_INCLUDE_DIR NAMES json11.hpp
        PATHS
        ${PROJECT_SOURCE_DIR}/external/include
        ${PROJECT_SOURCE_DIR}/external/include/${CMAKE_LIBRARY_ARCHITECTURE}
        ${CMAKE_BINARY_DIR}/include/
        ${CMAKE_BINARY_DIR}/include/${CMAKE_LIBRARY_ARCHITECTURE}
        /usr/include/
        /usr/local/include/json
        NO_DEFAULT_PATH
        )
FIND_LIBRARY(JSON_LIBRARY
        NAMES
        libjson11.a
        PATHS
        ${PROJECT_SOURCE_DIR}/external/lib/
        ${PROJECT_SOURCE_DIR}/external/lib/${CMAKE_LIBRARY_ARCHITECTURE}
        ${CMAKE_BINARY_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE}
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Json DEFAULT_MSG
        JSON_LIBRARY JSON_INCLUDE_DIR
        )

IF (JSON_INCLUDE_DIR AND JSON_LIBRARY)
    SET( JSON_FOUND 1 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ELSE ()
    SET( JSON_FOUND 0 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ENDIF ()

MARK_AS_ADVANCED(
#    JSON_FOUND
        JSON_LIBRARY
        JSON_INCLUDE_DIR
)
