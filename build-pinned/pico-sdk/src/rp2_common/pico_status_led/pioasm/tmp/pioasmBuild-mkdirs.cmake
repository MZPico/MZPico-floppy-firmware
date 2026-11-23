# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/matyamar/src/MZPico-floppy-firmware/external/pico-sdk/tools/pioasm"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pioasm"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pioasm-install"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/tmp"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/src/pioasmBuild-stamp"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/src"
  "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/src/pioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/src/pioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/matyamar/src/MZPico-floppy-firmware/build-pinned/pico-sdk/src/rp2_common/pico_status_led/pioasm/src/pioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
