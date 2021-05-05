OPTION(USE_RENDEREDVIEW "Enable the rendered view form ScanNet mesh" ON)

IF(USE_RENDEREDVIEW)
    INCLUDE(UseAssimp)
ENDIF()

SET(targetname libGraphSLAMGUI)
SET(sources
        GraphSLAMGUI.cpp
        SurfelDrawer.cpp
        ImageDrawer.cpp
        CameraDrawer.cpp
        TrajectoryDrawer.cpp
        )
SET(headers
        GraphSLAMGUI.h
        SurfelDrawer.h
        ImageDrawer.h
        CameraDrawer.h
        TrajectoryDrawer.h
        )
FILE(GLOB src_graphDrawer  "./graphDrawer/*.cpp")
FILE(GLOB head_graphDrawer  "./graphDrawer/*.h")

INCLUDE(UseOpenCV)
INCLUDE(UseEigen3)

ADD_LIBRARY(${targetname}
        ${sources} ${headers}
        ${src_graphDrawer} ${head_graphDrawer}
)
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC libDataLoader
        PUBLIC libGraphSLAM
        PUBLIC GUI3D # need to put this the last. otherwise some weird bugs appear. (not able to show text and lines, etc.)
)

IF(USE_RENDEREDVIEW)
    INCLUDE(LinkAssimp)
ENDIF()