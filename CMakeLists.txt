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

add_custom_target(examples)

foreach(EXAMPLE_MAIN IN LISTS EXAMPLE_MAINS)
    get_filename_component(FILE_NAME ${EXAMPLE_MAIN} NAME_WE) 
    
    string(REPLACE "main_" "" TARGET_NAME ${FILE_NAME}) 

    add_executable(${TARGET_NAME} ${EXAMPLE_MAIN})
    target_link_libraries(${TARGET_NAME} PRIVATE example_helpers)
    
    add_dependencies(examples ${TARGET_NAME})
endforeach()

file(GLOB TEST_SRC "tests/*.cpp")

add_executable(test_suite ${TEST_SRC})
target_include_directories(test_suite PRIVATE include tests examples)
target_link_libraries(test_suite PRIVATE example_helpers)