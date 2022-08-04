cmake_minimum_required(VERSION 3.15)
project(Adypt)

set(CMAKE_CXX_STANDARD 17)

# Windows' math include does not define constants by default.
# Set this definition so it does.
# Also set NOMINMAX so the min and max functions are not overwritten with macros.
if (CMAKE_SYSTEM_NAME STREQUAL Windows)
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-DNOMINMAX)
ENDIF ()

# Hide the console window in visual studio projects - Release
if (MSVC)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORTED)
    if (IPO_SUPPORTED)
        message("Enable IPO")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif ()
    if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
    endif ()
endif ()

add_subdirectory(dep)
add_subdirectory(shader)

add_executable(Adypt
        # MAIN PROGRAM
        src/main.cpp
        src/Application.hpp
        src/Application.cpp
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
        src/BinaryBVHBase.hpp
        src/FlatBinaryBVH.hpp
        src/AtomicBinaryBVH.hpp

        src/SBVHBuilder.hpp
        src/SBVHBuilder.cpp
        src/PSSBVHBuilder.cpp
        src/PSSBVHBuilder.hpp
        src/ParallelSBVHBuilder.cpp
        src/ParallelSBVHBuilder.hpp

        src/WideBVH.hpp
        # src/WideBVHBuilder.cpp
        src/BVHConfig.hpp
        src/BVHConfig.cpp

        src/AcceleratedScene.hpp
        src/AcceleratedScene.cpp

        # UTIL
        src/Math.hpp
        src/Shape.hpp
        src/Scene.hpp
        src/Scene.cpp
        src/ParallelSort.hpp
        )

# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
# 	target_link_libraries(Adypt PRIVATE OpenMP::OpenMP_CXX)
# endif()
find_package(Threads REQUIRED)
target_link_libraries(Adypt PRIVATE dep shader Threads::Threads)

install(TARGETS Adypt RUNTIME DESTINATION)
