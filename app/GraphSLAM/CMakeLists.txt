INCLUDE(UseOpenCV)
INCLUDE(UseEigen3)
IF(BUILD_GUI)
    SET(targetname exe_GraphSLAM_GUI)
    ADD_EXECUTABLE(${targetname} main.cc)
    TARGET_LINK_LIBRARIES(${targetname}
            PUBLIC libGraphSLAM
            PUBLIC libDataLoader
            )
    INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenCV.cmake)
    TARGET_LINK_LIBRARIES(${targetname}
            PUBLIC libGraphSLAMGUI
    )
    TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_PSLAM_GUI)
ENDIF(BUILD_GUI)

SET(targetname exe_GraphSLAM)
ADD_EXECUTABLE(${targetname} main.cc)
TARGET_LINK_LIBRARIES(${targetname} PUBLIC libGraphSLAM libDataLoader)
INCLUDE(LinkEigen3)
INCLUDE(LinkOpenCV)
