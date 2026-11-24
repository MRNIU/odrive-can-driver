# Copyright The odrive-can-driver Contributors

set(odrive_can_fixtures "${PROJECT_SOURCE_DIR}/tests/fixtures")

function(odrive_can_add_test test_name)
    set(test_target "odrive_can_test_${test_name}")
    add_executable("${test_target}" "test_${test_name}.c")
    target_link_libraries("${test_target}" PRIVATE odrive_can_driver::odrive_can_driver)
    add_test(NAME "${test_target}" COMMAND "${test_target}" ${ARGN})
endfunction()

odrive_can_add_test(cache)
odrive_can_add_test(protocol "${odrive_can_fixtures}/fw_v0_5_1.tsv")
odrive_can_add_test(driver)
odrive_can_add_test(driver_traces "${odrive_can_fixtures}/driver_traces.tsv")
