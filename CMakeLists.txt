cmake_minimum_required(VERSION 3.16)
project(planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall -g")

# Directories
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(EXAMPLES_DIR ${CMAKE_SOURCE_DIR}/examples)
set(TESTS_DIR ${CMAKE_SOURCE_DIR}/tests)

include_directories(${INCLUDE_DIR})

# Main executable
file(GLOB SRC_FILES ${SRC_DIR}/*.cpp)
add_executable(planner ${SRC_FILES})

# Example executables
file(GLOB EXAMPLE_MAIN_SRC ${EXAMPLES_DIR}/main_*.cpp)
file(GLOB EXAMPLE_HELPER_SRC ${EXAMPLES_DIR}/*.cpp)
list(REMOVE_ITEM EXAMPLE_HELPER_SRC ${EXAMPLE_MAIN_SRC})

foreach(EXAMPLE ${EXAMPLE_MAIN_SRC})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE} ${EXAMPLE_HELPER_SRC} ${SRC_FILES})
    target_include_directories(${EXAMPLE_NAME} PRIVATE ${INCLUDE_DIR})
endforeach()

# Test executable
file(GLOB TEST_SRC ${TESTS_DIR}/*.cpp)
add_executable(test ${TEST_SRC} ${EXAMPLE_HELPER_SRC} ${SRC_FILES})
target_include_directories(test PRIVATE ${INCLUDE_DIR})

# Clean target
add_custom_target(clean-all COMMAND ${CMAKE_COMMAND} -E rm -rf ${BUILD_DIR} ${TMP_DIR})
