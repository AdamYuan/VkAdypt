cmake_minimum_required(VERSION 3.15)
project(shader)

set(SHADER_SRC
        quad.vert
        ray_tracer.frag
        )

set(SHADER_HEADER
        accelerated_scene.glsl
        camera.glsl
        )
list(TRANSFORM SHADER_HEADER PREPEND ${CMAKE_CURRENT_LIST_DIR}/)

# Compile
find_program(GLSLC "glslc")
IF (GLSLC)
    foreach (X IN LISTS SHADER_SRC)
        add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/include/spirv/${X}.u32
                COMMAND glslc ${X} -o include/spirv/${X}.u32 -O -Os -mfmt=num
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                DEPENDS ${CMAKE_CURRENT_LIST_DIR}/${X} ${SHADER_HEADER}
                )
    endforeach ()
ENDIF ()
foreach (X IN LISTS SHADER_SRC)
    list(APPEND SHADER_U32 include/spirv/${X}.u32)
endforeach ()

add_custom_target(shader_u32 ALL DEPENDS ${SHADER_U32})

add_library(shader INTERFACE)
add_dependencies(shader shader_u32)
target_include_directories(shader INTERFACE include)
