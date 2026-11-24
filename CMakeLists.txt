# Copyright The odrive-can-driver Contributors

cmake_minimum_required(VERSION 3.21)
project(odrive_can_driver LANGUAGES C)

add_library(odrive_can_driver STATIC
    c/src/fw_v0_5_1.c
    c/src/response_cache.c
    c/src/driver.c)
add_library(odrive_can_driver::odrive_can_driver ALIAS odrive_can_driver)
target_include_directories(odrive_can_driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/c/include")
target_compile_features(odrive_can_driver PUBLIC c_std_17)
set_target_properties(odrive_can_driver PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO)

if(PROJECT_IS_TOP_LEVEL AND NOT CMAKE_CROSSCOMPILING)
    include(CTest)
    set(CMAKE_C_STANDARD 17)
    set(CMAKE_C_STANDARD_REQUIRED YES)
    set(CMAKE_C_EXTENSIONS NO)

    add_executable(odrive_can_periodic c/examples/periodic.c)
    add_executable(odrive_can_driver_poll c/examples/driver_poll.c)
    target_link_libraries(odrive_can_periodic PRIVATE odrive_can_driver::odrive_can_driver)
    target_link_libraries(odrive_can_driver_poll PRIVATE odrive_can_driver::odrive_can_driver)

    if(BUILD_TESTING)
        add_test(NAME odrive_can_periodic_example COMMAND odrive_can_periodic)
        add_test(NAME odrive_can_driver_poll_example COMMAND odrive_can_driver_poll)
        add_subdirectory(c/tests)
    endif()
endif()
