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

# --- an emulated MZ-800 running MZF programs against the FDC ---
add_library(z80cpu STATIC third_party/z80/z80.c)
target_include_directories(z80cpu PUBLIC third_party/z80)
target_compile_options(z80cpu PRIVATE -w)
add_executable(mz800_run mz800/mz800_run.cpp mz800/machine.cpp)
target_link_libraries(mz800_run fwsim z80cpu)

# --- FDTEST.MZF on the emulated MZ-800 (needs z88dk; see mzf/) ---
include(${CMAKE_CURRENT_LIST_DIR}/../mzf/mzf.cmake)
function(mz800_test name)
  add_test(NAME ${name} COMMAND mz800_run ${ARGN})
  set_tests_properties(${name} PROPERTIES TIMEOUT 900)
endfunction()
if(MZF_FDTEST)
  # Positive runs: the controller passes FDTEST on good media.
  mz800_test(fdtest_master_read_write ${MZF_FDTEST} --quiet --disk0 master --keys 03Y --expect "RESULT: PASS")
  mz800_test(fdtest_data_disk_drive1 ${MZF_FDTEST} --quiet --disk0 dd --disk1 dd --keys 11 --expect "RESULT: PASS")
  mz800_test(fdtest_dense_media ${MZF_FDTEST} --quiet --disk0 dense --keys 01 --expect "RESULT: PASS")
  mz800_test(fdtest_slow_spindle ${MZF_FDTEST} --quiet --disk0 master --rev0 218700 --keys 03Y --expect "RESULT: PASS")
  # Negative runs: FDTEST must notice a controller-visible fault.
  mz800_test(fdtest_reports_write_protect ${MZF_FDTEST} --quiet --disk0 master --wp0 --keys 03Y --expect "RESULT: FAIL")
  mz800_test(fdtest_reports_empty_drive ${MZF_FDTEST} --quiet --disk0 none --keys 01 --limit 120 --expect "NOT READY")

  # Local tier: real disk images, if present. They are copyrighted and never
  # committed; point MZP_MEDIA_DIR at a directory holding them (default: the
  # repository root, where the bring-up images live).
  set(MZP_MEDIA_DIR ${CMAKE_CURRENT_LIST_DIR}/.. CACHE PATH "directory with local .dsk images")
  file(GLOB local_dsk ${MZP_MEDIA_DIR}/*.dsk)
  foreach(d ${local_dsk})
    get_filename_component(n ${d} NAME_WE)
    string(MAKE_C_IDENTIFIER ${n} n)
    mz800_test(fdtest_local_${n} ${MZF_FDTEST} --quiet --disk0 edsk:${d} --keys 01 --expect "RESULT: PASS")
  endforeach()
endif()
