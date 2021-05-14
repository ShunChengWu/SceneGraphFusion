IF(WITH_ASSIMP)
    target_link_libraries(${targetname} PUBLIC ${ASSIMP_LIBRARIES}         -ldl)
    TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_ASSIMP)
ENDIF()