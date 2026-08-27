cmake_minimum_required(VERSION 3.16)
project(evtol_sim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

set(EVTOL_SOURCES
    src/controller/pid_controller.cpp
    src/dynamics/evtol_model.cpp
    src/sensor/altitude_sensor.cpp
    src/flight_state/flight_state.cpp
)

add_library(evtol_lib STATIC ${EVTOL_SOURCES})
target_include_directories(evtol_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/controller
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_state
)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE evtol_lib GTest::gtest_main)

add_executable(test_dynamics tests/test_dynamics.cpp)
target_link_libraries(test_dynamics PRIVATE evtol_lib GTest::gtest_main)

add_executable(test_sensor tests/test_sensor.cpp)
target_link_libraries(test_sensor PRIVATE evtol_lib GTest::gtest_main)

add_executable(test_flight_state tests/test_flight_state.cpp)
target_link_libraries(test_flight_state PRIVATE evtol_lib GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_pid)
gtest_discover_tests(test_dynamics)
gtest_discover_tests(test_sensor)
gtest_discover_tests(test_flight_state)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

set(PYBIND11_FINDPYTHON ON)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(evtol_core src/bindings/python_bindings.cpp)
target_link_libraries(evtol_core PRIVATE evtol_lib)
target_include_directories(evtol_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set_target_properties(evtol_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
