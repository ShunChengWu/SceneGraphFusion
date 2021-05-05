
# Adds steps to download, configure and install dependencies at configure-time
function(add_configure_time_dependency name)
    if (NOT TARGET update-${name})
        MESSAGE("-build " ${name})
#        SET(${name}_internal_cache 1 CACHE INTERNAL "${NAME} cache")
        configure_file(${CMAKE_SOURCE_DIR}/cmake/External_${name}.CMakeLists.txt.in ${CMAKE_BINARY_DIR}/external/${name}/CMakeLists.txt)
        execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name} )
        if(result)
            message(FATAL_ERROR "CMake step for ${name} failed: ${result}")
        endif()
        execute_process(COMMAND ${CMAKE_COMMAND} --build .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name} )
        if(result)
            message(FATAL_ERROR "Build step for ${name} failed: ${result}")
        endif()

        # Add update target to get new sources and recompile
        add_custom_target(update-${name}
                COMMAND ${CMAKE_COMMAND} --build . --target ${name}-update
                COMMAND ${CMAKE_COMMAND} --build . --target all
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}
                )
    ELSE()
        MESSAGE("-skip " ${name})
    ENDIF()
endfunction()