cmake_minimum_required(VERSION 3.15)
project(shader)

set(SHADER_SRC 

	quad.vert

	ray_tracer.frag
	)

find_program(GLSLC "glslc")
IF(GLSLC)
	add_executable(shader_gen gen/main.cpp gen/stb_include.h)
	function(add_spirv_shader INPUT_FILE)
		add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/inc/${INPUT_FILE}
			COMMAND shader_gen ${CMAKE_CURRENT_LIST_DIR} ${INPUT_FILE} inc/${INPUT_FILE}
			WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
			DEPENDS ${CMAKE_CURRENT_LIST_DIR}/${INPUT_FILE}
			)
		add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/include/spirv/${INPUT_FILE}.u32
			COMMAND glslc inc/${INPUT_FILE} -o include/spirv/${INPUT_FILE}.u32 -O -Os -mfmt=num
			WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
			DEPENDS ${CMAKE_CURRENT_LIST_DIR}/inc/${INPUT_FILE}
			)
	endfunction()

	foreach(X IN LISTS SHADER_SRC)
		add_spirv_shader(${X})
	endforeach()
ENDIF()

set(SHADER_U32) 
foreach(X IN LISTS SHADER_SRC)
	list(APPEND SHADER_U32 include/spirv/${X}.u32)
endforeach()

add_custom_target(
	shader_u32 ALL DEPENDS
	${SHADER_U32}
	)
add_library(shader INTERFACE)
target_include_directories(shader INTERFACE include)
