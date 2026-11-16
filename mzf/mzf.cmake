# z88dk +mz build of the MZF programs; included by mzf/CMakeLists.txt and by
# the host test project (tests/), which runs them on its emulated MZ-800.
find_program(ZCC_EXECUTABLE z88dk.zcc)
set(MZF_DIR ${CMAKE_CURRENT_LIST_DIR})
set(MZF_FILES "")
if(NOT ZCC_EXECUTABLE)
  message(STATUS "z88dk.zcc not found: MZF programs not built (see mzf/CMakeLists.txt)")
  return()
endif()

# Same target and flags as MZPico-800-Manager: Sharp MZ, org 1200h, stack D000h.
set(MZF_ZCC_FLAGS +mz -create-app -pragma-define:REGISTER_SP=0xd000 -m)

function(mzf_program NAME)
  string(TOLOWER ${NAME} lower)
  set(out ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.MZF)
  set(srcs ${ARGN})
  add_custom_command(
    OUTPUT ${out}
    COMMAND ${ZCC_EXECUTABLE} ${MZF_ZCC_FLAGS} -o ${lower} ${srcs}
    COMMAND ${CMAKE_COMMAND} -E rename ${lower}.mzt ${out}
    DEPENDS ${srcs}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "z88dk: ${NAME}.MZF")
  add_custom_target(mzf_${lower} ALL DEPENDS ${out})
  set(MZF_FILES ${MZF_FILES} ${out} PARENT_SCOPE)
endfunction()

mzf_program(FDTEST ${MZF_DIR}/fdtest/fdtest.c)
set(MZF_FDTEST ${CMAKE_CURRENT_BINARY_DIR}/FDTEST.MZF)
