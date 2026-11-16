# 9Z_504M.ROM floppy routines (ports 0xD8-0xDF)

This note maps the confirmed floppy-controller I/O in `9Z_504M.dis.asm` and gives probable routine roles.

The sources this note is drawn from — the boot ROM image `9Z_504M.ROM`, its
disassembly `9Z_504M.dis.asm`, and the Disk BASIC listing `MZ2Z046.ASM` — are
copyrighted Sharp material. They are kept locally (git-ignored) and are **not
distributed with this repository**; addresses below refer to the ROM itself.

## Port map (MZ-800 Disk BASIC convention)

- `0xD8`: WD status (read) / WD command (write), active-low bus values
- `0xD9`: WD track register
- `0xDA`: WD sector register
- `0xDB`: WD data register
- `0xDC`: drive/motor select control latch
- `0xDD`: side/page select latch
- `0xDE`: density latch (not directly touched in this disassembly block)
- `0xDF`: no direct I/O found in this ROM block

## Confirmed I/O points in disassembly

- Control latch writes (`OUT (0xDC),A`): around `E4F1`, `E51C`, `E548`, `E5FD`
- Status polls (`IN A,(0xD8)`): around `E502`, `E55A`, `E57A`, `E5C1`, `E5D5`
- Command writes (`OUT (0xD8),A`): around `E52F`, `E64E`, `E653`
- Data register write (`OUT (0xDB),A`): around `E61F`
- Side/page write (`OUT (0xDD),A`): around `E627`
- Track/sector writes (`OUT (0xD9),A`, `OUT (0xDA),A`): around `E62E-E632`

## Probable routine grouping (from behavior)

- `E4DC...E548`: controller and motor/control-latch setup, status pre-check
- `E568`: wait loop variant on status bit state (one polarity)
- `E587`: wait loop variant on status bit state (opposite polarity)
- `E61B...E628`: push transfer byte to data register and set side/page
- `E62B...E632`: load track/sector registers
- `E64E`: issue command to 0xD8 (with active-low inversion)

## Call flow (who calls what)

- `E45F` floppy operation entry:
	- initializes per-request state
	- calls `E530` (reset transient state)
	- calls `E5A7` (main floppy sector R/W engine)
	- validates and reports result
- `E602` setup routine:
	- called by `E5A7`
	- calls `E4DC` (select/ready check)
	- loads track/sector/buffer counters from the parameter block (`IX`)
- `E5A7` main sector-transfer loop:
	- calls `E696` (derive geometry/loop counters)
	- calls `E5FB` (drive/motor transfer-phase select)
	- calls `E61B` (data+side setup)
	- calls `E528` (status mask pre-check)
	- calls `E62B` (program track/sector)
	- calls `E64E` (write command to 0xD8 and wait)
	- loops data transfer using `INI`
	- calls `E63C` (sector advance)
	- calls `E658` (status->error mapping)
- low-level helpers:
	- `E555` is the shared command+status primitive used by `E528` and `E548`
	- `E568` / `E587` are complementary wait predicates used by command paths
	- `E653` issues force interrupt via 0xD8

## Cross-reference source

The semantic names above match the floppy definitions in `MZ2Z046.ASM`:

- port constants near `CD_REG/ST_REG/TR_REG/SC_REG/DA_REG/MO_REG/PG_REG/DE_REG`
- floppy helper routines around `A3851`, `A385D`, `A38A8`, `A38E3`, `A38F0`, `A38F7`, `A3906`

Address mapping between that listing and this ROM is consistent with:

- `ROM_E_address = MZ2Z046_A_address + 0xB000`
