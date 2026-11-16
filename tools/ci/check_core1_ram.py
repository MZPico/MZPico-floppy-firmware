#!/usr/bin/env python3
"""Fail if core1's bus path reaches into flash.

core1 has ~560 ns to answer a Z80 I/O cycle (CLAUDE.md, "Nothing goes in
core1's listen loop"). Its functions are RAM_FUNC, but a RAM function can still
call into flash -- a libgcc division helper (`% 96` did: two __aeabi_idivmod
calls per register access) -- or load a switch's jump table from flash
.rodata. On an XIP cache miss either costs ~0.5 us. Issue #12.

Starting from the listen loop and the register handlers, and following every
function they can call (transitively), each function reached must:
  - itself live in RAM (0x2xxxxxxx);
  - call only RAM functions, and no linker veneer (a RAM stub that jumps on
    into flash: how the idivmod calls appeared);
  - load no literal-pool word that points into flash (0x10xxxxxx).

    tools/ci/check_core1_ram.py build/mzpico_floppy.elf
"""
import re
import subprocess
import sys

ROOTS = ["_ZL11listen_loopv", "_Z13fdc_reg_writehh", "_Z12fdc_reg_readh"]

FUNC_RE = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:$")
CALL_RE = re.compile(r"\sblx?\s+([0-9a-f]+) <([^>+]+)(\+0x[0-9a-f]+)?>")
WORD_RE = re.compile(r"\.word\s+0x(1[0-9a-f]{7})\b")


def disassemble(elf):
    out = subprocess.run(["arm-none-eabi-objdump", "-d", elf], check=True,
                         capture_output=True, text=True).stdout
    funcs = {}
    name = None
    for line in out.splitlines():
        m = FUNC_RE.match(line)
        if m:
            name = m.group(2)
            funcs[name] = (int(m.group(1), 16), [])
        elif name and line.strip():
            funcs[name][1].append(line)
        elif not line.strip():
            name = None
    return funcs


def main():
    elf = sys.argv[1] if len(sys.argv) > 1 else "build/mzpico_floppy.elf"
    funcs = disassemble(elf)
    problems = []
    seen = set()
    todo = list(ROOTS)
    while todo:
        f = todo.pop()
        if f in seen:
            continue
        seen.add(f)
        if f not in funcs:
            problems.append(f"{f}: not found in {elf}")
            continue
        addr, body = funcs[f]
        if addr >> 28 != 0x2:
            problems.append(f"{f}: lives at 0x{addr:08x}, not in RAM")
        for line in body:
            m = CALL_RE.search(line)
            if m:
                target, callee, off = int(m.group(1), 16), m.group(2), m.group(3)
                if target >> 28 != 0x2 or callee.endswith("_veneer"):
                    problems.append(f"{f}: calls flash: {line.strip()}")
                elif callee != f and not off:
                    todo.append(callee)  # follow it: RAM alone proves nothing
            if WORD_RE.search(line):
                problems.append(f"{f}: flash literal: {line.strip()}")
    for p in problems:
        print(f"check_core1_ram: {p}", file=sys.stderr)
    if not problems:
        print(f"check_core1_ram: OK -- {len(seen)} function(s) reachable from "
              f"core1's bus path, all in RAM: {', '.join(sorted(seen))}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
