cmake_minimum_required(VERSION 3.15)

project(ga_dev)

option(USE_DOUBLE_PRECISION "Enable Bullet Physic library to use double precision" ON)

if(USE_DOUBLE_PRECISION)
    message("-- Build with Double Precision")
    add_compile_definitions(BT_USE_DOUBLE_PRECISION)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # C++ Compiler configs
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    include(CheckPIESupported)
    check_pie_supported()

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

list(APPEND CMAKE_PREFIX_PATH /usr/local/include ${CMAKE_SOURCE_DIR}/../external/ntrt/external/env)

find_package(ntrt REQUIRED)
find_package(Bullet REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_subdirectory(libGA)
add_subdirectory(structureStudy)
add_subdirectory(movingBall_sim)
add_subdirectory(geneticAlgs)