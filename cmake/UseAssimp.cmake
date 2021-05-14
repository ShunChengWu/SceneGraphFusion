find_package(Assimp QUIET)
OPTION(WITH_ASSIMP "Build with Assimp support?" ${Assimp_FOUND})

IF(WITH_ASSIMP)
    MESSAGE(STATUS "WITH Assimp")
    find_package(Assimp REQUIRED)
    INCLUDE_DIRECTORIES(${Assimp_INCLUDE_DIRS})
ENDIF()