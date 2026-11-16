# Host-side tests for the MZPico floppy firmware. A separate CMake project
# from the firmware (which needs the Pico SDK and an ARM toolchain):
#
#   cmake -S tests -B build-tests -G Ninja && cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure
#
# The firmware's own sources are compiled unmodified against the simulation in
# tests/sim (virtual clock, two cores, a simulated drive and disk).
cmake_minimum_required(VERSION 3.16)
project(mzpico_floppy_tests C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(FW ${CMAKE_CURRENT_LIST_DIR}/../src)
enable_testing()

# --- the firmware under test + the simulation it runs on ---
add_library(fwsim STATIC
  ${FW}/bus/fdc.cpp
  ${FW}/floppy.cpp
  ${FW}/mfm_decode.cpp
  ${FW}/mfm_encode.cpp
  ${FW}/mfm_splice.cpp
  sim/sim_core.cpp
  sim/sim_drive.cpp
  sim/disk_image.cpp
  sim/board.cpp
)
target_include_directories(fwsim PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/hal/include
  ${CMAKE_CURRENT_LIST_DIR}/sim
  ${FW} ${FW}/bus ${FW}/flux)
target_compile_definitions(fwsim PUBLIC MZP_HOST_SIM=1)
# The firmware is written for arm-none-eabi -O3; keep host warnings from
# drowning the test output, but never hide errors.
target_compile_options(fwsim PRIVATE -w)

add_library(testfw STATIC sim/testfw.cpp)
target_include_directories(testfw PUBLIC sim)

function(fw_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} fwsim testfw)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 900)
endfunction()

fw_test(test_mfm unit/test_mfm.cpp)
fw_test(test_read_window unit/test_read_window.cpp)
fw_test(test_fdc fdc/test_fdc.cpp)
