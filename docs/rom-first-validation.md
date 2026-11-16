# ROM-first validation sequence (Phase 3.2 focus)

> **Historical (pre-3.2).** This checklist was written before READ SECTOR worked
> and describes firmware of that time — e.g. the index-synchronized read start
> it targets was later removed (reads now start at any rotational phase with a
> one-revolution + one-sector-span window). `tools/rom_first_validation.sh` is
> equally historical. For the current design see [../CLAUDE.md](../CLAUDE.md)
> and [wd-fidelity.md](wd-fidelity.md); for a current on-machine check use
> `FDTEST.MZF` ([../mzf/fdtest/README.md](../mzf/fdtest/README.md)) and start
> every bench session with `mzp_probe.py PORT health`.

This checklist is for validating that the MZ-800 ROM floppy routines run reliably first, before adding more feature scope.

## Scope

- Target path: RESTORE -> SEEK -> READ-SEC(M) -> FORCE-INT
- Relevant references:
  - 9Z_504M disassembly flow at E5A7 family
  - Disk BASIC flow at A39D3 polling loop
- Firmware target: index-synchronized read start and verify start

## Prerequisites

1. Firmware builds and flashes successfully.
2. Board wiring is known-good, including the RDATA pull-up bodge.
3. A known-good bootable/readable disk is inserted.
4. USB probe path is available via tools/mzp_probe.py.

## Bench sequence (host side)

Run:

1. ./tools/rom_first_validation.sh <serial-port> [track] [sector] [side] [density]
2. Example: ./tools/rom_first_validation.sh /dev/ttyACM0 0 1 0 0

Expected bench signals:

1. fdcread returns found=1 and has_data=1 for known-good sector.
2. streamscan reports decodable sectors with valid CRCs.
3. fdcdump shows coherent state transitions without stuck busy/drq.

## Live ROM sequence (MZ-800 side)

1. Cold boot test:
   - Power cycle MZ-800 and interface board.
   - Attempt disk directory/boot path that exercises ROM floppy routines.
2. Warm retry test:
   - Reset and repeat the same operation.
3. Repeatability test:
   - Perform at least 10 runs total (mixed cold and warm starts).

## Pass criteria

All must pass:

1. No FD loading error during the test set.
2. Directory/boot read path succeeds repeatedly (target: 10/10).
3. No pathological busy wait behavior observed in ROM path.
4. Post-run fdclog remains consistent with expected command pattern:
   - Type I setup commands
   - Type II read command family
   - Force interrupt usage

## Fail signatures and likely next adjustments

1. Frequent abort_after around short timeout with no DRQ:
   - Increase probability of early sector hit after index by tuning read-start alignment strategy.
2. DRQ appears but transfer fails mid-sector:
   - Re-check xfer handoff and status bit transitions around DRQ drop.
3. Verify regressions after sync changes:
   - Tune verify index-sync budget separately from read budget.

## Suggested test log template

Record per run:

1. Run id and cold/warm type
2. Disk identity
3. Operation attempted
4. Result (pass/fail)
5. If fail: latest fdcdump line and fdclog tail

A short CSV or text table is enough for comparison across firmware revisions.
