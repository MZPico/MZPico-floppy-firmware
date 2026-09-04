#!/usr/bin/env python3
"""Build tools/mzf/BUSTEST.MZF -- the MZ-800 side of the bus health check.

Load it on the MZ-800 (monitor `L`, or BASIC LOAD of a machine-code file) with
the firmware's bus-test counters freshly reset, and it runs 524,288 I/O cycles
of the full loopback: OUT the next sequence byte to 0xDF, IN the firmware's
echo from 0xDF, OUT that echo to 0xDE for the firmware to verify. Then it
prints DONE and returns to the monitor. The verdict lives on the firmware
side: `mzp_probe.py PORT bustest` reports cycles, sequence gaps (dropped I/O
cycles), garbage (corrupted bytes), per-line duty and the READ-direction
error count. A clean bus shows err=0 with rx ~= 524288 and rd_reports the
same.

    mzp_probe.py PORT bustestreset     # BEFORE loading -- CP/M's EINT writes
                                       # on 0xDF pollute the sequence checker
    (load + run BUSTEST.MZF on the MZ-800, ~12 s)
    mzp_probe.py PORT bustest          # verdict

Side effects, by design of the test: it writes every value to 0xDF (EINT)
and 0xDE (density), so do not run it with CP/M live; the firmware's density
declaration is left at whatever the last echo's bit 0 was -- a bus RESET or
power cycle clears it. Run this on a cold machine, from the monitor.

Z80 source (MZ-700/800 monitor calls: 0015h = MSG, prints DE-string to 0Dh):

    ORG  1200h
    LD   C,0            ; running sequence byte -- must be continuous
    LD   DE,MSG1
    CALL 0015h
    LD   B,8            ; passes
OUTER:
    LD   DE,0           ; 65536 iterations per pass (DEC to zero)
INNER:
    LD   A,C
    OUT  (0DFh),A       ; write the sequence byte
    PUSH AF
    IN   A,(0DFh)       ; read the firmware's echo (raw ^ A5)
    OUT  (0DEh),A       ; hand it back for READ-direction verification
    POP  AF
    INC  C
    DEC  DE
    LD   A,D
    OR   E
    JR   NZ,INNER
    DJNZ OUTER
    LD   DE,MSG2
    CALL 0015h
    JP   0000h          ; monitor restart (L jumps to us; RET has no caller)
MSG1: DB "MZPICO BUS TEST 524K CYCLES",0Dh
MSG2: DB "DONE - NOW RUN: MZP_PROBE BUSTEST",0Dh
"""
import struct, sys, os

ORG = 0x1200
code = bytearray()

def emit(*b):
    code.extend(b)

# Labels resolved by hand below; assert the offsets after assembly.
emit(0x0E, 0x00)              # 1200 LD C,0
emit(0x11, 0x00, 0x00)        # 1202 LD DE,MSG1  (patched)
emit(0xCD, 0x15, 0x00)        # 1205 CALL 0015h
emit(0x06, 0x08)              # 1208 LD B,8
OUTER = len(code)             # 120A
emit(0x11, 0x00, 0x00)        # 120A LD DE,0
INNER = len(code)             # 120D
emit(0x79)                    # 120D LD A,C
emit(0xD3, 0xDF)              # 120E OUT (DFh),A
emit(0xF5)                    # 1210 PUSH AF
emit(0xDB, 0xDF)              # 1211 IN A,(DFh)
emit(0xD3, 0xDE)              # 1213 OUT (DEh),A
emit(0xF1)                    # 1215 POP AF
emit(0x0C)                    # 1216 INC C
emit(0x1B)                    # 1217 DEC DE
emit(0x7A)                    # 1218 LD A,D
emit(0xB3)                    # 1219 OR E
jr_at = len(code)
emit(0x20, (INNER - (jr_at + 2)) & 0xFF)   # 121A JR NZ,INNER
djnz_at = len(code)
emit(0x10, (OUTER - (djnz_at + 2)) & 0xFF) # 121C DJNZ OUTER
emit(0x11, 0x00, 0x00)        # 121E LD DE,MSG2  (patched)
emit(0xCD, 0x15, 0x00)        # 1221 CALL 0015h
emit(0xC3, 0x00, 0x00)        # 1224 JP 0000h (monitor restart)
MSG1 = len(code)
code.extend(b"MZPICO BUS TEST 524K CYCLES\r")
MSG2 = len(code)
code.extend(b"DONE - NOW RUN: MZP_PROBE BUSTEST\r")

# Patch the two LD DE immediates with absolute addresses.
struct.pack_into("<H", code, 3, ORG + MSG1)
struct.pack_into("<H", code, 0x1F, ORG + MSG2)

# Sanity on the hand-computed layout.
assert code[0x1A] == 0x20 and code[0x1B] == 0xF1, code[0x1A:0x1C].hex()
assert code[0x1C] == 0x10 and code[0x1D] == 0xEC, code[0x1C:0x1E].hex()
assert code[0x1E] == 0x11 and code[0x24] == 0xC3

# MZF (MZ-700/800 tape image) header: 128 bytes.
hdr = bytearray(128)
hdr[0] = 0x01                              # attribute: OBJ (machine code)
name = b"BUSTEST"
hdr[1:1 + len(name)] = name
hdr[1 + len(name)] = 0x0D                  # filename terminator
struct.pack_into("<H", hdr, 18, len(code)) # size
struct.pack_into("<H", hdr, 20, ORG)       # load address
struct.pack_into("<H", hdr, 22, ORG)       # exec address
comment = b"MZPico bus loopback test"
hdr[24:24 + len(comment)] = comment

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mzf", "BUSTEST.MZF")
with open(out, "wb") as f:
    f.write(hdr)
    f.write(code)
print(f"wrote {out}: {len(code)} code bytes at {ORG:04X}, MZF {128 + len(code)} bytes")
print("code:", code[:0x25].hex(" "))
