# Regenerated on EVERY build (see the gitver custom target): writes gitver.h
# only when the hash actually changed, so nothing recompiles needlessly but
# the reported version can never go stale -- which is the whole point: a test
# session was once diagnosed against the wrong firmware because nothing could
# say which build was running.
execute_process(
  COMMAND git -C ${GIT_DIR} rev-parse --short HEAD
  OUTPUT_VARIABLE GIT_HASH OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT GIT_HASH)
  set(GIT_HASH "unknown")
endif()
execute_process(
  COMMAND git -C ${GIT_DIR} status --porcelain --untracked-files=no
  OUTPUT_VARIABLE GIT_DIRTY OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(GIT_DIRTY)
  set(GIT_HASH "${GIT_HASH}+dirty")
endif()
set(CONTENT "#define FW_GIT_HASH \"${GIT_HASH}\"\n")
if(EXISTS ${OUT})
  file(READ ${OUT} OLD)
else()
  set(OLD "")
endif()
if(NOT CONTENT STREQUAL OLD)
  file(WRITE ${OUT} ${CONTENT})
endif()
