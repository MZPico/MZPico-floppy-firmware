# MZ-800 programs for testing the floppy controller on the machine itself.
#
#   cmake -S mzf -B build-mzf && cmake --build build-mzf    -> build-mzf/FDTEST.MZF
#
# Needs z88dk's zcc as `z88dk.zcc` (the snap's name). Without the snap, use the
# pinned Docker image: PATH=$PWD/tools/ci/z88dk:$PATH cmake -S mzf -B build-mzf
cmake_minimum_required(VERSION 3.10)
project(mzpico_floppy_mzf NONE)
include(${CMAKE_CURRENT_LIST_DIR}/mzf.cmake)
