cmake_minimum_required(VERSION 3.15)
project(Adypt)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif ()

# Windows' math include does not define constants by default.
# Set this definition so it does.
# Also set NOMINMAX so the min and max functions are not overwritten with macros.
if(CMAKE_SYSTEM_NAME STREQUAL Windows)
	add_definitions(-D_USE_MATH_DEFINES)
	add_definitions(-DNOMINMAX)
ENDIF()

# Hide the console window in visual studio projects - Release
if(MSVC)
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
endif()

add_subdirectory(dep)
add_subdirectory(shader)

add_executable(Adypt
	# MAIN PROGRAM
	src/main.cpp
	src/Application.hpp
	src/Application.cpp
	src/ImGuiRenderer.hpp
	src/ImGuiRenderer.cpp
	src/Camera.hpp
	src/Camera.cpp
	src/Config.hpp

	src/RayTracer.hpp
	src/RayTracer.cpp

	src/QuadSpirv.hpp

	# UI
	src/UIHelper.hpp
	src/UIHelper.cpp

	# BVH
	src/SBVH.hpp
	src/SBVHBuilder.cpp
	src/WideBVH.hpp
	src/WideBVH.cpp
	src/WideBVHBuilder.cpp
	src/BVHConfig.hpp
	src/BVHConfig.cpp

	src/AcceleratedScene.hpp
	src/AcceleratedScene.cpp

	# UTIL
	src/Math.hpp
	src/Shape.hpp
	src/Scene.hpp
	src/Scene.cpp
	)

# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
# 	target_link_libraries(Adypt PRIVATE OpenMP::OpenMP_CXX)
# endif()
find_package(Threads REQUIRED)
target_link_libraries(Adypt PRIVATE dep shader Threads::Threads)

install(TARGETS Adypt RUNTIME DESTINATION)
