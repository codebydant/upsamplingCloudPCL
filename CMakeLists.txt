# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_BUILD_TYPE_INIT Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

project(upsampling_cloud VERSION 1.1.0 LANGUAGES CXX)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the CMP0074 policy to old behavior (disable warnings) (CMake 3.12.0-rc1)
if(${CMAKE_VERSION} MATCHES 3.12.0)
    cmake_policy(SET CMP0074 OLD)

    if(POLICY CMP0048)
        cmake_policy(SET CMP0048 NEW)
    endif(POLICY CMP0048)
endif()

# #############################################################################
# PACKAGES
# #############################################################################
find_package(PCL 1.8 REQUIRED QUIET)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "   ERROR: PCL minimum required version 1.8. Not found")
endif()

fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
)

fetch_project(
    NAME argparse
    URL https://github.com/p-ranav/argparse/archive/v2.6.tar.gz
)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "src/main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

# #############################################################################
# HEADERS
# #############################################################################
target_include_directories(${PROJECT_NAME} PRIVATE
    ${PCL_INCLUDE_DIRS}
)

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PCL_LIBRARIES}
    cloudparse
    argparse
)

# #############################################################################
# COMPILATION FLAGS: MMX, SSE(1, 2, 3, 3S, 4.1, 4.2), CLMUL, RdRand, VT-x, x86-64
# #############################################################################
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp
    -mmmx
    -msse
    -msse2
    -msse3
    -mssse3
    -msse4.2
    -msse4.1
    -mno-sse4a
    -mno-avx
    -mno-avx2
    -mno-fma
    -mno-fma4
    -mno-f16c
    -mno-xop
    -mno-bmi
    -mno-bmi2
    -mrdrnd
    -mno-3dnow
    -mlzcnt
    -mfsgsbase
    -mpclmul
)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
