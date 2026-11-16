#!/bin/sh
# Fail if core1's bus path reaches into flash.
#
# core1 has ~560 ns to answer a Z80 I/O cycle (CLAUDE.md, "Nothing goes in
# core1's listen loop"). Its functions are RAM_FUNC, but a RAM function can
# still call into flash -- a libgcc division helper (`% 96` did: two
# __aeabi_idivmod calls per register access) -- or load a switch's jump table
# from flash .rodata. On an XIP cache miss either costs ~0.5 us. Issue #12.
#
# Checks, in the disassembly of the listen loop and the register handlers:
#   - every BL/BLX target is in RAM (0x2xxxxxxx), and is not a linker veneer
#     (a RAM stub that jumps on into flash: how the idivmod calls appeared)
#   - no literal-pool word points into flash (0x10xxxxxx)
#
#   tools/ci/check_core1_ram.sh build/mzpico_floppy.elf
set -eu
ELF=${1:-build/mzpico_floppy.elf}
DIS=$(mktemp)
trap 'rm -f "$DIS"' EXIT
arm-none-eabi-objdump -d "$ELF" > "$DIS"

fail=0
for sym in _ZL11listen_loopv _Z13fdc_reg_writehh _Z12fdc_reg_readh _ZL18fdc_reg_read_innerh; do
  body=$(awk -v s="<$sym>:" '$2 == s {on=1; next} on && /^$/ {exit} on' "$DIS")
  if [ -z "$body" ]; then
    echo "check_core1_ram: $sym not found in $ELF" >&2
    fail=1
    continue
  fi
  bad_calls=$(printf '%s\n' "$body" | grep -E '\sbl(x)?\s' | grep -E -v '\sbl(x)?\s+2[0-9a-f]{7}\s' || true)
  veneers=$(printf '%s\n' "$body" | grep -E '\sbl(x)?\s.*_veneer>' || true)
  bad_calls="$bad_calls$veneers"
  bad_words=$(printf '%s\n' "$body" | grep -E '\.word\s+0x1[0-9a-f]{7}' || true)
  if [ -n "$bad_calls$bad_words" ]; then
    echo "check_core1_ram: $sym reaches into flash:" >&2
    printf '%s\n%s\n' "$bad_calls" "$bad_words" | sed '/^$/d; s/^/  /' >&2
    fail=1
  else
    echo "check_core1_ram: $sym OK"
  fi
done
exit $fail
