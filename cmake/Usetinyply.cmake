###
FIND_PACKAGE(tinyply QUIET)

OPTION(WITH_TINYPLY "Build with tinyply" ${TINYPLY_FOUND})

IF(WITH_TINYPLY)
    IF(NOT tinyply_FOUND)
        INCLUDE(ConfigureTimeDependency)
        add_configure_time_dependency(tinyply)
        FIND_PACKAGE(tinyply REQUIRED)
    ENDIF()
    ADD_DEFINITIONS(-DCOMPILE_WITH_TINYPLY)

    IF(NOT TARGET libtinyply)
        ADD_LIBRARY(libtinyply
                INTERFACE
                )
        TARGET_LINK_LIBRARIES(libtinyply
                INTERFACE ${TINYPLY_LIBRARY}
                )
        TARGET_INCLUDE_DIRECTORIES(libtinyply
                INTERFACE ${TINYPLY_INCLUDE_DIR}
                )
        TARGET_COMPILE_DEFINITIONS(libtinyply INTERFACE COMPILE_WITH_TINYPLY)
    ENDIF()
ENDIF()