# Copyright The odrive-can-driver Contributors

cmake_minimum_required(VERSION 3.21)
project(odrive_can_consumer LANGUAGES C)

include(CTest)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../.." odrive_can_driver)

add_executable(odrive_can_consumer main.c)
target_link_libraries(odrive_can_consumer PRIVATE odrive_can_driver::odrive_can_driver)
set_target_properties(odrive_can_consumer PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)

if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_test(NAME odrive_can_consumer COMMAND odrive_can_consumer)
endif()
