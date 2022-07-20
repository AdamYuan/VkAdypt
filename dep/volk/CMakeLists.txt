cmake_minimum_required(VERSION 3.0)
cmake_policy(PUSH)
cmake_policy(SET CMP0048 NEW) # project(... VERSION ...) support

project(volk VERSION
        # VOLK_GENERATE_VERSION
        170
        # VOLK_GENERATE_VERSION
        LANGUAGES C
        )

# CMake 3.12 changes the default behaviour of option() to leave local variables
# unchanged if they exist (which we want), but we must work with older CMake versions.
if (NOT DEFINED VOLK_STATIC_DEFINES)
    option(VOLK_STATIC_DEFINES "Additional defines for building the volk static library, e.g. Vulkan platform defines" "")
endif ()

# -----------------------------------------------------
# Static library

add_library(volk STATIC volk.h volk.c)
target_include_directories(volk PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<INSTALL_INTERFACE:include>
        )
if (VOLK_STATIC_DEFINES)
    target_compile_definitions(volk PUBLIC ${VOLK_STATIC_DEFINES})
endif ()
if (NOT WIN32)
    target_link_libraries(volk PUBLIC dl)
endif ()

# -----------------------------------------------------
# Interface library

add_library(volk_headers INTERFACE)
add_library(volk::volk_headers ALIAS volk_headers)
target_include_directories(volk_headers INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        )
if (NOT WIN32)
    target_link_libraries(volk_headers INTERFACE dl)
endif ()

cmake_policy(POP)
