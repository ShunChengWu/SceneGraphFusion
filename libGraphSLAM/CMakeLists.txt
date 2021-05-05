SET(targetname libGraphSLAM)

INCLUDE(UseEigen3)
INCLUDE(UseOpenCV)

if(APPLE)
    set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_osx.a)
else()
    set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_linux.a)
endif()

FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.h")
IF(BUILD_GRAPHPRED)
    INCLUDE(UseOnnxRuntime)
    FILE(GLOB src_gp "graphPredictor/*.cpp")
    FILE(GLOB h_gp   "graphPredictor/*.h")
    SET(sources ${sources} ${src_gp})
    SET(headers ${headers} ${h_gp})
ENDIF(BUILD_GRAPHPRED)



ADD_LIBRARY(${targetname} ${sources} ${headers})
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC  ${INSEG_LIB}
)
TARGET_INCLUDE_DIRECTORIES(${targetname}
        PUBLIC ../lib)
IF(BUILD_GRAPHPRED)
    INCLUDE(LinkOnnxRuntime)
    target_compile_definitions(${targetname} PUBLIC COMPILE_WITH_GRAPHPRED)
ENDIF(BUILD_GRAPHPRED)
INCLUDE(Linktinyply)
INCLUDE(LinkJson)
