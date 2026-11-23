#!/usr/bin/env python3
"""Fail if core1's bus path reaches into flash.

core1 has ~560 ns to answer a Z80 I/O cycle (CLAUDE.md, "Nothing goes in
core1's listen loop"). Its functions are RAM_FUNC, but a RAM function can still
call into flash -- a libgcc division helper (`% 96` did: two __aeabi_idivmod
calls per register access) -- or load a switch's jump table from flash
.rodata. On an XIP cache miss either costs ~0.5 us. Issue #12.

Starting from the listen loop, the register handlers and the bus-RESET
interrupt handler, and following every
function they can call (transitively), each function reached must:
  - itself live in RAM (0x2xxxxxxx);
  - call only RAM functions, and no linker veneer (a RAM stub that jumps on
    into flash: how the idivmod calls appeared) -- including tail calls (a
    `b` to another function) and no indirect call (`blx rN`), which cannot
    be followed;
  - load no literal-pool word that points into flash (0x10xxxxxx).

    tools/ci/check_core1_ram.py build/mzpico_floppy.elf
"""
import re
import subprocess
import sys

# The bus-RESET edge interrupt runs on core1 too, between bus cycles: a
# flash fetch there delays core1's next answer just the same.
# core0's publish-lock regions (#61): while core0 holds the lock a core1
# command write spins on it, so these must be RAM-only and call nothing --
# not even another RAM function (a call could block or grow the hold time).
# Each locked region is a lambda in fdc_disk_task (plus one helper).
LOCKED_RE = re.compile(r"^_ZZ13fdc_disk_taskvENKUlvE\d*_clEv(\.\w+\.\d+)?$")
LOCKED_EXTRA = ["_ZL24fdc_end_for_missing_unitv"]

ROOTS = ["_ZL11listen_loopv", "_Z13fdc_reg_writehh", "_Z12fdc_reg_readh",
         "_ZN12_GLOBAL__N_1L17fdc_bus_reset_isrEv"]

FUNC_RE = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:$")
CALL_RE = re.compile(r"\sblx?\s+([0-9a-f]+) <([^>+]+)(\+0x[0-9a-f]+)?>")
WORD_RE = re.compile(r"\.word\s+0x(1[0-9a-f]{7})\b")
# A branch to another symbol is a tail call; a register call is unknowable.
BRANCH_RE = re.compile(r"\sb(?:\.[nw])?\s+([0-9a-f]+) <([^>+]+)(\+0x[0-9a-f]+)?>")
INDIRECT_RE = re.compile(r"\sbl?x\s+(r\d+|ip|lr)\b")


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
            b = BRANCH_RE.search(line)
            if b and b.group(2) != f:  # a jump into ANOTHER function
                target, callee = int(b.group(1), 16), b.group(2)
                if target >> 28 != 0x2 or callee.endswith("_veneer"):
                    problems.append(f"{f}: tail-calls flash: {line.strip()}")
                elif not b.group(3):
                    todo.append(callee)
            if INDIRECT_RE.search(line) and "bx\tlr" not in line and " bx lr" not in line:
                problems.append(f"{f}: indirect call (cannot be checked): {line.strip()}")
            if WORD_RE.search(line):
                problems.append(f"{f}: flash literal: {line.strip()}")
    locked = [n for n in funcs if LOCKED_RE.match(n)] + \
             [n for n in LOCKED_EXTRA if n in funcs]
    if not locked:
        problems.append("no publish-lock regions found (renamed? update LOCKED_RE)")
    for f in locked:
        addr, body = funcs[f]
        if addr >> 28 != 0x2:
            problems.append(f"{f}: publish-lock region at 0x{addr:08x}, not in RAM")
        for line in body:
            if CALL_RE.search(line) or INDIRECT_RE.search(line) and "bx\tlr" not in line:
                problems.append(f"{f}: publish-lock region calls out: {line.strip()}")
            b = BRANCH_RE.search(line)
            if b and b.group(2) != f:
                problems.append(f"{f}: publish-lock region jumps out: {line.strip()}")
            if WORD_RE.search(line):
                problems.append(f"{f}: publish-lock region flash literal: {line.strip()}")
    for p in problems:
        print(f"check_core1_ram: {p}", file=sys.stderr)
    if not problems:
        print(f"check_core1_ram: OK -- {len(seen)} function(s) reachable from "
              f"core1's bus path, all in RAM: {', '.join(sorted(seen))}; "
              f"{len(locked)} publish-lock region(s) RAM-only with no calls")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
