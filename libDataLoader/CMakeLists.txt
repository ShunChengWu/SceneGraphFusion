SET(targetname libDataLoader)

INCLUDE(UseOpenCV)
INCLUDE(UseEigen3)

SET(sources
        src/dataloader_scannet.cpp
        src/dataloader_3rscan.cpp
        ScanNetLoader/sensorData.cpp
)
SET(headers
        include/dataLoader/dataset_loader.h
        include/dataLoader/dataset_loader_facotry.h
)

ADD_LIBRARY(${targetname} ${sources} ${headers})
TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ./include)