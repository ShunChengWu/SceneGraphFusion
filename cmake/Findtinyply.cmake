
FIND_PATH(TINYPLY_INCLUDE_DIR NAMES tinyply.h
        PATHS
        ${PROJECT_SOURCE_DIR}/external/include
        ${CMAKE_BINARY_DIR}/include/
        ${CMAKE_BINARY_DIR}/include/${CMAKE_LIBRARY_ARCHITECTURE}
        /usr/include/
        /usr/local/include/
        NO_DEFAULT_PATH
        )
FIND_LIBRARY(TINYPLY_LIBRARY
        NAMES
        libtinyply.a
        PATHS
        ${PROJECT_SOURCE_DIR}/external/lib
        ${CMAKE_BINARY_DIR}/lib/
        ${CMAKE_BINARY_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE}
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tinyply DEFAULT_MSG
        TINYPLY_LIBRARY TINYPLY_INCLUDE_DIR
        )

IF (TINYPLY_INCLUDE_DIR AND TINYPLY_LIBRARY)
    SET( TINYPLY_FOUND 1 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ELSE ()
    SET( TINYPLY_FOUND 0 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ENDIF ()

MARK_AS_ADVANCED(
        #    TINYPLY_FOUND
        TINYPLY_LIBRARY
        TINYPLY_INCLUDE_DIR
)