cmake_minimum_required(VERSION 3.16)
project(MotionPlanningRulebooks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

# Core Library Setup
file(GLOB SRC_FILES "src/*.cpp")
list(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

add_library(core_lib STATIC ${SRC_FILES})
target_include_directories(core_lib PUBLIC include)
target_link_libraries(core_lib PUBLIC m)

# Main Executable
add_executable(planner src/main.cpp)
target_link_libraries(planner PRIVATE core_lib)

# Example Helpers
file(GLOB EXAMPLE_ALL "examples/*.cpp")
file(GLOB EXAMPLE_MAINS "examples/main_*.cpp")
set(EXAMPLE_HELPERS ${EXAMPLE_ALL})
if(EXAMPLE_MAINS)
    list(REMOVE_ITEM EXAMPLE_HELPERS ${EXAMPLE_MAINS})
endif()

add_library(example_helpers STATIC ${EXAMPLE_HELPERS})
target_include_directories(example_helpers PUBLIC include examples)
target_link_libraries(example_helpers PUBLIC core_lib)

# Example Executables
# Grid Example
add_executable(grid examples/main_grid.cpp)
target_link_libraries(grid PRIVATE example_helpers)

# Navigation Example
add_executable(navigation examples/main_navigation.cpp)
target_link_libraries(navigation PRIVATE example_helpers)

# Test Executable
file(GLOB TEST_SRC "tests/*.cpp")

add_executable(test_suite ${TEST_SRC})
target_include_directories(test_suite PRIVATE include tests examples)
target_link_libraries(test_suite PRIVATE example_helpers)