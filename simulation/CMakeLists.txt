cmake_minimum_required(VERSION 3.15)

cmake_policy(SET CMP0069 NEW) # IPO flags behaviour
cmake_policy(SET CMP0083 NEW) # PIE related flags (for linking phase)

project(robot_simulation VERSION 0.1.0 LANGUAGES CXX)

# ====================== BUILD CONFIGURATION ====================== #
option(USE_DOUBLE_PRECISION "Enable Bullet Physic library to use double precision" ON)
option(SIN_CONTROLLER "Select whether to use a sinusoidal controller instead of the one managed by a NN" OFF)
option(TEST_SINGLE_MODULE "Enable debug mode for measuring the effects of NN controller on a single module" OFF)
option(STEP_INPUT "Enable the execution of a special control signal for testing purposes - Requires TEST_SINGLE_MODULE set to ON" OFF)
option(NO_ACTIVE_CABLES "Select whether to not add the actuation cables" OFF)
option(ENABLE_IPO "Enable Inter-Procedural Optimization" ON)
option(ENABLE_CCACHE "Enable CCache to improve repeated compilations" ON)
option(DIRECT_ASSOC "Enable DIRECT_ASSOC to associate controller outputs to modules based on module order" OFF)
option(NO_SERVO "Enable NO_SERVO to simulate the robot without the parallelepiped representing the servo" OFF)

# select which type of build perform
set(default_build_type "RelWithDebInfo")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# needed to be compatible with Bullet library
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

    # program and compiler optimizations
    include(CheckIPOSupported)
    check_ipo_supported(RESULT IPO_SUPPORT)
    if(IPO_SUPPORT AND ENABLE_IPO)
        message("-- Inter-Procedural Optimization (IPO) enabled")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()

    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM AND ENABLE_CCACHE)
        message("-- CCache enabled")
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    endif()

    # add support folders in IDE
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# ===================  DEPENDENCIES MANAGEMENT ==================== #

find_package(ntrt REQUIRED)
find_package(Bullet REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# ==================== BUILD THE APPLICATIONS ===================== #

# config where simulation executables will be held
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/apps)
message("-- Simulation applications will be created in: ${EXECUTABLE_OUTPUT_PATH}")

list(APPEND ROB_SIM_INCLUDE
  ${EIGEN3_INCLUDE_DIR}
  ${NTRT_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)
# libraries to link with the applications
list(APPEND ROB_SIM_LIBS
    ntrt::core
    ntrt::controllers
    ntrt::tgcreator
    ntrt::terrain
    ntrt::sensors
    Eigen3::Eigen
)


add_executable(robotSimulation
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotSimulation.cpp
)
if(SIN_CONTROLLER)
    message("-- Build with Sinusoidal Controller")
    target_compile_definitions(robotSimulation PUBLIC SIN_CONTROLLER)
else()
    # default behaviour
    message("-- Build with NN Controller")
endif()
if(TEST_SINGLE_MODULE)
    message("-- Build with debug information for single module tests")
    target_compile_definitions(robotSimulation PUBLIC TEST_SINGLE_MODULE)
    target_compile_definitions(robotSimulation PUBLIC DEBUG_INFO)

    ## NOTE: enable the following to compute the command vs compression model
    if(STEP_INPUT)
        target_compile_definitions(robotSimulation PUBLIC STEP_INPUT)
    endif()
endif()
if(NO_SERVO)
    target_compile_definitions(robotSimulation PUBLIC NO_SERVO)
endif()
target_include_directories(robotSimulation PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotSimulation PUBLIC ${ROB_SIM_LIBS})


add_executable(robotDirection
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotDirection.cpp
)
if(NO_SERVO)
    target_compile_definitions(robotDirection PUBLIC NO_SERVO)
endif()
target_include_directories(robotDirection PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotDirection PUBLIC ${ROB_SIM_LIBS})


add_executable(robotRotationAngle
    robotModel.cpp
    robotRotationAngle.cpp
)
if(NO_SERVO)
    target_compile_definitions(robotRotationAngle PUBLIC NO_SERVO)
endif()
target_include_directories(robotRotationAngle PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotRotationAngle PUBLIC ${ROB_SIM_LIBS})


add_executable(robotSimulationGR
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    targetModel.cpp
    robotSimulationGR.cpp
)
if(DIRECT_ASSOC)
    target_compile_definitions(robotSimulationGR PUBLIC DIRECT_ASSOC)
endif()
if(NO_SERVO)
    target_compile_definitions(robotSimulationGR PUBLIC NO_SERVO)
endif()
target_include_directories(robotSimulationGR PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotSimulationGR PUBLIC ${ROB_SIM_LIBS})


set(DATA_DIR nn_data)
if(NOT EXISTS ${DATA_DIR})
    message("-- Create default directory for controller log data")
    file(MAKE_DIRECTORY ${DATA_DIR})
endif()


add_executable(robotSimulationGRLog
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    targetModel.cpp
    robotSimulationGR.cpp
)
target_compile_definitions(robotSimulationGRLog PUBLIC CONTROLLER_INFO)
if(DIRECT_ASSOC)
    target_compile_definitions(robotSimulationGRLog PUBLIC DIRECT_ASSOC)
endif()
if(NO_SERVO)
    target_compile_definitions(robotSimulationGRLog PUBLIC NO_SERVO)
endif()
target_include_directories(robotSimulationGRLog PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotSimulationGRLog PUBLIC ${ROB_SIM_LIBS})


add_executable(robotSimulationSGR
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    robotControllerSGR.cpp
    obstacleModel.cpp
    targetModel.cpp
    robotSimulationSGR.cpp
)
if(NO_SERVO)
    target_compile_definitions(robotSimulationSGR PUBLIC NO_SERVO)
endif()
target_include_directories(robotSimulationSGR PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotSimulationSGR PUBLIC ${ROB_SIM_LIBS})


add_executable(robotSimulationSGRLog
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    robotControllerSGR.cpp
    obstacleModel.cpp
    targetModel.cpp
    robotSimulationSGR.cpp
)
target_compile_definitions(robotSimulationSGRLog PUBLIC CONTROLLER_INFO)
if(NO_SERVO)
    target_compile_definitions(robotSimulationSGRLog PUBLIC NO_SERVO)
endif()
target_include_directories(robotSimulationSGRLog PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotSimulationSGRLog PUBLIC ${ROB_SIM_LIBS})


add_executable(robotVisualization
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotVisualization.cpp
)
if(SIN_CONTROLLER)
    target_compile_definitions(robotVisualization PUBLIC SIN_CONTROLLER)
endif()
if(TEST_SINGLE_MODULE)
    target_compile_definitions(robotVisualization PUBLIC TEST_SINGLE_MODULE)
    target_compile_definitions(robotVisualization PUBLIC DEBUG_INFO)

    ## NOTE: enable the following to compute the command vs compression model
    if(STEP_INPUT)
        target_compile_definitions(robotVisualization PUBLIC STEP_INPUT)
    endif()
    if(NO_ACTIVE_CABLES)
        target_compile_definitions(robotVisualization PUBLIC NO_ACTIVE_CABLES)
    endif()
endif()
if(NO_SERVO)
    target_compile_definitions(robotVisualization PUBLIC NO_SERVO)
endif()
target_include_directories(robotVisualization PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotVisualization PUBLIC ${ROB_SIM_LIBS})


add_executable(robotVisualizationGR
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    targetModel.cpp
    robotVisualizationGR.cpp
)
if(DIRECT_ASSOC)
    target_compile_definitions(robotVisualizationGR PUBLIC DIRECT_ASSOC)
endif()
if(NO_SERVO)
    target_compile_definitions(robotVisualizationGR PUBLIC NO_SERVO)
endif()
target_include_directories(robotVisualizationGR PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotVisualizationGR PUBLIC ${ROB_SIM_LIBS})


add_executable(robotVisualizationSGR
    neuralNet.cpp
    neuralNetGR.cpp
    robotModel.cpp
    robotController.cpp
    robotControllerGR.cpp
    robotControllerSGR.cpp
    obstacleModel.cpp
    targetModel.cpp
    robotVisualizationSGR.cpp
)
if(NO_SERVO)
    target_compile_definitions(robotVisualizationSGR PUBLIC NO_SERVO)
endif()
target_include_directories(robotVisualizationSGR PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotVisualizationSGR PUBLIC ${ROB_SIM_LIBS})


add_executable(robotStatic
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotStatic.cpp
)
target_compile_definitions(robotStatic PUBLIC STATIC)
if(TEST_SINGLE_MODULE)
    target_compile_definitions(robotStatic PUBLIC TEST_SINGLE_MODULE)
    target_compile_definitions(robotStatic PUBLIC DEBUG_INFO)
else()
    target_compile_definitions(robotStatic PUBLIC ADD_MARKERS)
endif()
if(NO_SERVO)
    target_compile_definitions(robotStatic PUBLIC NO_SERVO)
endif()

## NOTES:
## - this mode is no more useful, kept for backup
## - if this definitions is enabled, before performing measurements
## remember to set actuator rod (sphere + light rod + half rod) mass (density) to 0,
## so that the module stays in place where it is built.
#target_compile_definitions(robotStatic PUBLIC MEASURE_CABLES)

target_include_directories(robotStatic PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotStatic PUBLIC ${ROB_SIM_LIBS})


add_executable(robotMeasurements
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotMeasurements.cpp
)
target_compile_definitions(robotMeasurements PUBLIC DEBUG_INFO)
target_compile_definitions(robotMeasurements PUBLIC COM_COLLECTION)
target_compile_definitions(robotMeasurements PUBLIC STEP_INPUT)
message("-- Build with module conf EVOLUTION support")
target_compile_definitions(robotMeasurements PUBLIC EVOLVE_MODULE)
if(NO_SERVO)
    target_compile_definitions(robotMeasurements PUBLIC NO_SERVO)
endif()
target_include_directories(robotMeasurements PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotMeasurements PUBLIC ${ROB_SIM_LIBS})


add_executable(robotCOM
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotCOM.cpp
)
target_compile_definitions(robotCOM PUBLIC DEBUG_INFO)
target_compile_definitions(robotCOM PUBLIC COM_COLLECTION)

## NOTE: enable the following to compute the command vs compression model
if(STEP_INPUT)
    message("-- Build with STEP controller for testing purposes")
    target_compile_definitions(robotCOM PUBLIC STEP_INPUT)
endif()
if(NO_ACTIVE_CABLES)
    message("-- Build with NO ACTIVE CABLES for testing purposes")
    target_compile_definitions(robotCOM PUBLIC NO_ACTIVE_CABLES)
endif()
if(NO_SERVO)
    target_compile_definitions(robotCOM PUBLIC NO_SERVO)
endif()
target_include_directories(robotCOM PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotCOM PUBLIC ${ROB_SIM_LIBS})


add_executable(robotTrajectory
    neuralNet.cpp
    robotModel.cpp
    robotController.cpp
    robotTrajectory.cpp
)
if(SIN_CONTROLLER)
    target_compile_definitions(robotTrajectory PUBLIC SIN_CONTROLLER)
endif()
if(NO_SERVO)
    target_compile_definitions(robotTrajectory PUBLIC NO_SERVO)
endif()
target_include_directories(robotTrajectory PUBLIC ${ROB_SIM_INCLUDE})
target_link_libraries(robotTrajectory PUBLIC ${ROB_SIM_LIBS})
# ================================================================= #
