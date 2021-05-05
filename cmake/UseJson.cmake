#################
# UseJson.cmake #
#################
FIND_PACKAGE(Json QUIET)

OPTION(WITH_JSON "Build with Json" ${JSON_FOUND})

IF(WITH_JSON)
    if(NOT JSON_FOUND)
        INCLUDE(ConfigureTimeDependency)
        add_configure_time_dependency(Json)
        FIND_PACKAGE(Json REQUIRED)
    ENDIF()

    ADD_DEFINITIONS(-DCOMPILE_WITH_JSON)
    MESSAGE("JSON_INCLUDE_DIR: " ${JSON_INCLUDE_DIR})
    INCLUDE_DIRECTORIES(${JSON_INCLUDE_DIR})
#    Create a target for easy linking library
    IF(NOT TARGET libJson)
        ADD_LIBRARY(libJson
                INTERFACE
                )
        TARGET_LINK_LIBRARIES(libJson
                INTERFACE ${JSON_LIBRARY}
                )
        TARGET_INCLUDE_DIRECTORIES(libJson
                INTERFACE ${JSON_INCLUDE_DIR}
                )
        TARGET_COMPILE_DEFINITIONS(libJson INTERFACE COMPILE_WITH_JSON)
    ENDIF()

ENDIF()