# Phase 3 plan — real-time WD FDC bridge to the MZ-800

Goal: the MZ-800 ROM talks to ports 0xD8–0xDE and sees a faithful **WD179x /
MB8877** floppy controller; the Pico backs it with the **physical drive in real
time** (no cache). Spec: [wd-fidelity.md](wd-fidelity.md). This is the phase that
puts the board between a real MZ-800 and a real drive.

## Architecture (two cores, mirroring the chip)

- **Core 1 — host/register side (hard real-time):** services MZ-800 I/O cycles
  on ports 0xD8–0xDE; holds the WD register + command state machine; computes
  STATUS; drives `~EXWAIT` (bus timing) and `~INT` (INTRQ); handles the inverted
  data bus. Must never block.
- **Core 0 — disk side:** the "data separator + write channel." Streaming MFM
  decode from the live flux FIFO (find marks, decode bytes, track rotational
  position), byte FIFO to core1 on reads; timed write-splice on writes; drives
  STEP/DIR/HDSEL/MOTOR/WGATE.
- **Shared state:** command request + a small byte ring + status flags
  (BUSY/DRQ/INTRQ/errors) between cores. core0's IRQ-disable during flux does not
  stall core1.

PIO/SM budget: bus servicing (a few SMs) on one PIO block, the flux read/write SM
on the other; both RP2040 PIO blocks used.

## What we reuse (and its license implications)

| Need | Source | Action |
|------|--------|--------|
| Bus servicing PIO + loop (shared C0..C7, EN/OE, EXWAIT) | reference `src/bus_io.pio`, `bus.hpp`, `device.cpp` (**DELUXE** path) | **adapt** — remap pins, match this board's EN0/EN1/RD1 topology |
| WD register/command state machine | reference `src/mz_devices/fdc.cpp` (GPL v2+) | **refactor** — drop EDSK byte-source; add async BUSY/DRQ + physical backend |
| Streaming flux sampling | Adafruit `arch_rp2` `start_polled_capture()` / `mfm_io_sample_flux()` (dropped in Phase 1) | **re-vendor** — feeds the streaming decoder |
| MFM symbol/CRC/mark logic | `flux/mfm_impl.h` (already vendored) | **reuse** — re-plumb to pull from the live FIFO |
| Drive mechanics (seek/step/side/motor) | `floppy.cpp` (Phase 1) | **reuse** for Type I commands |

Note: adapting `fdc.cpp` (GPL v2+) and the reference bus code makes those
components GPL — the firmware's licensing must reflect that.

## Board-specific bus handshake (the critical path)

Shared **C0..C7** (GPIO0–7) carries address then data via two SN74LVC245s:
`~EN0`(26)=address buffer OE, `~EN1`(27)=data buffer OE, `~RD1`(28)=data buffer
direction. Control: `~IORQ`(8), `~RD`(9), `~WR`(10), `~EXWAIT`(11), `~INT`(12),
`RESET`(29). Per I/O cycle core1 must:

1. On `~IORQ` active, assert `~EN0`, read the port address off C0..C7.
2. If it's **not** 0xD8–0xDE → release, ignore. If it is → assert `~EXWAIT`
   immediately (stall the Z80 while we work).
3. **Read cycle** (`~RD`): compute the register byte, **invert it** (ROM `CPL`s),
   switch buffers (`~EN0` off, `~EN1` on, `~RD1`=drive-out), drive C0..C7, release
   `~EXWAIT`, hold until the Z80 latches, then release the bus.
4. **Write cycle** (`~WR`): read the byte off C0..C7 (via data buffer), **invert
   it**, hand to the WD state machine, release `~EXWAIT`.

Never assert `~EN0` and `~EN1` together. `~EXWAIT` is µs-scale (register timing
only) — never held across seek/rotational latency (that would stall Z80 refresh;
BUSY-polling covers latency instead, per the ROM driver).

## Milestones (each independently testable on a real MZ-800)

- **3.0 — Bus bring-up ("hello MZ-800"). ✅ DONE (hardware-verified).** The board
  answers I/O reads/writes on the FDC ports (0xD8–0xDE) over the shared C0..C7 bus:
  `OUT`/`INP` from the MZ-800 round-trip bit-for-bit (with the ROM's data-bus
  inversion), even and odd ports both register, and the `busdump` bit-activity
  monitor shows all 8 C-lines toggling. *Validated the entire bus handshake — the
  riskiest bit.* Design: PIO (pio1) side-sets ONLY the address buffer (EN0);
  **core1 drives the data buffer (EN1/RD1) directly and only for our ports**, so
  non-FDC I/O by other MZ-800 devices is never touched. `~EXWAIT` times the
  register access only. A hardware bug found during bring-up: a solder bridge
  between U2 pin 18 (C0) and pin 19 (~EN1) corrupted both bit0 and the data-buffer
  enable — cleared on the board. Bench isolation tools added to `mzp_probe.py`
  (`gpioread`, `pintest`) but note the shared C-net means true pad-isolation needs
  the MZ-800 powered.
- **3.1 — Type I + status. ✅ DONE (hardware-verified).** RESTORE/SEEK/STEP drive
  the real head; STATUS returns real BUSY/TRK00/WRITE-PROTECT/NOT-READY. The
  MZ-800 ROM issues `RESTORE → SEEK → READ-SEC(M)` — i.e. it accepts our Type I
  and advances to sector reads. Async model: core1 decodes commands + serves the
  WD register file (with per-port inversion: 0xD8–0xDB CPL'd, 0xDC–0xDE not);
  core0 runs motor/select/side/seek in the background while the CPU polls BUSY
  (no wait-state stall). Control ports decoded from `MZ2Z046.ASM`: 0xDC b7=motor
  b1..0=drive, 0xDD b0=side, 0xDE b0=density. **Drive gotchas found:** this drive
  needs the spindle actually spinning before it will step to TRK00 (spin_motor
  with a 300 ms pre-sleep), and a motor-off delay keeps it spinning between the
  ROM's retries. `fdc.{cpp,hpp}`; probe cmds `fdcdump`/`fdclog`/`fdcseek`. No
  INTRQ yet — the ROM polls STATUS for Type I.
- **3.2 — READ SECTOR (single + multiple). ✅ DONE (hardware-verified) — CP/M 4.1
  boots to the prompt from a physical disk.** Getting there took three things
  beyond the read engine itself: READ ADDRESS (3.4), the LOST DATA / undrained
  DRQ timeouts below, and working interrupts (see "Interrupts" below) — the BIOS
  is a different client from the boot ROM and exercises all three. On a Type II
  read (0x8X/0x9X)
  core1 sets BUSY at once and hands core0 a request; core0 captures a bounded
  flux window and decodes the requested sector by ID via `mfm_read_sector()`,
  publishing it to a one-sector transfer buffer and raising **DRQ**. core1
  serves DATA reads from that buffer, auto-advancing the Sector register in
  multi-record mode; Force Interrupt (0xD0/0xD8) terminates. Multi-record reuses
  the captured window, so sectors 2..N cost only a decode (~2 ms).
  - **Capture and decode must be strictly sequential.** The flux capture has no
    DMA — the PIO RX FIFO holds 4 words × 2 half-samples = 8 transitions, about
    **24 µs** at DD rates. Any work between drains longer than that silently
    drops flux, MFM sync never holds, and no sector is ever found. An earlier
    decode-while-capturing design failed exactly this way while `scan` /
    `streamscan` / `fdcread` (all capture-then-decode) passed on the same disk.
  - **No index sync, no pre-arm.** A ~1.05-revolution window contains every
    sector wherever it starts, so index alignment buys nothing and pre-arming
    reintroduced the interleaving hazard from the other side.
  - **Host budget (measured, `rddiag`):** the boot ROM polls STATUS ~25,000
    times over **~285 ms** after a READ before issuing Force Interrupt — one
    poll every 11.4 µs, a ~40 T-state Z80 loop. Capture ~215 ms + decode ~2 ms
    fits, but not by much; this is the constraint that sizes the window.
  - **Undrained DRQ must time out.** We buffer a whole sector, so unlike a real
    WD we *can* wait forever — and a host that is entitled to ignore the bytes
    then deadlocks us with BUSY+DRQ held. Two budgets: a sector read that goes
    unread is a real overrun (raise LOST DATA), while an undrained READ ADDRESS
    is normal and must complete silently — the WD also loads the ID's track byte
    into the Sector register, so "issue READ ADDRESS, wait for BUSY, read Sector"
    is a legitimate idiom. Observed before this: CP/M 4.1 looping RESTORE /
    READ ADDRESS forever, 27031 STATUS polls per attempt, xfer=0/6 every time.
  - Diagnostics: `rddiag` reports capture bytes/time, decode time, the sector
    IDs actually present on the track when a read RNFs, the host's poll count
    and span, `~INT` state (mode / EINT armed / line asserted / assert count),
    and LOST DATA events.

- **Interrupts. ✅ DONE (hardware-verified — this is what got CP/M 4.1 to the
  prompt).** `~INT` is DRQ-driven and EINT-gated, not WD INTRQ-driven; the full
  model, the clearing rules, and why the EINT gate is a safety requirement are
  in [wd-fidelity.md](wd-fidelity.md#interrupts--drq-driven-not-intrq-driven).
  Three things had to be right at once:
  - The refresh call next to the **first DRQ of a sector** must run in every
    mode. It was gated on the diagnostic force-on mode, so an EINT-armed host
    armed, issued READ, and waited for an edge that never came.
  - The refresh helper must **drive the line unconditionally**, with no
    "already in that state" short-circuit. Both cores call it; a compare-and-skip
    on a plain flag is a read-modify-write race whose bad direction is `~INT`
    stuck low with nothing pending = hard machine hang.
  - **Bus RESET** (GPIO29, active **high** — the one active-high control line)
    must disarm EINT. It is a latch in our state; a resetting host cannot clear
    it, so a reset mid-interrupt otherwise brought the machine back up into a
    permanent one.
- **3.3 — WRITE SECTOR (in-place splice).** Locate the sector by ID (streaming
  decode), then WGATE + write the MFM-encoded data field in the ~22-byte Gap 2
  window (~700 µs slack). Test: MZ-800 writes a file; read back (host `gw`/scan).
  **Not implemented — now the top priority.** The stub **fails honestly**: a
  Type II write completes immediately with **WRITE PROTECT** (b6) set in the
  Type II STATUS layout, so the host sees an unambiguous error. It previously
  cleared BUSY with no error bits, i.e. reported *success* — defensible when the
  boot ROM was the only client (it never writes), but a data-loss trap now that
  CP/M 4.1 boots to a prompt and can: the host's in-RAM directory and buffers
  would diverge from the disk, and the damage would surface later on a disk the
  user believes is good. `rddiag` counts refused writes.
- **3.4 — Type III / format / density / multi-drive.**
  - **READ ADDRESS (0xCX). ✅ DONE (hardware-verified).** Returns the first ID
    field in the captured flux as the six bytes a real WD hands over DRQ —
    track, side, sector, size code, CRC1, CRC2 — using the CRC bytes actually
    recorded on the disk, not a recomputation, so a host that checks them sees
    what the chip would have given it. Also loads the ID's track byte into the
    Sector register (WD quirk; the CP/M BIOS reads head position from there).
    This is what stopped the BIOS stepping blindly and giving up.
  - **Still to do:** READ TRACK, WRITE TRACK (format), density applied to
    `~DEN_RDY` (decoded already — see wd-fidelity.md), drive-select via
    `~DS0/DS1`.
- **3.5 — Robustness.** Error/retry paths, write precomp if inner tracks are
  marginal, edge-case timing, soak testing.

## Primary risks / unknowns

1. ~~**Bus real-time response** (M3.0)~~ — ✅ **resolved.** The port-0xDF
   loopback (`mzp_probe.py bustest`) runs >1M writes and >1M reads with zero
   mismatches, zero late cycles and zero FIFO backlog, all eight C-lines at
   exactly 50.00 % duty. The PIO re-arms the address buffer only after a ~122 ns
   guard so it can never contend with core1's drive; core1 skips (and counts)
   any cycle that ends before it can service it.
2. **No `~M1` pin on this board.** The reference uses M1 to distinguish I/O from
   opcode-fetch/interrupt-acknowledge. We decode on `~IORQ`+address only →
   theoretical risk of responding during an IORQ-based INTA cycle. Assess against
   the MZ-800's interrupt scheme; likely benign but confirm. **Still open.**
3. ~~**Streaming decode keeping bit-sync in real time** (M3.2)~~ — ✅ **moot.**
   Decode does not run concurrently with capture (see 3.2 above); it runs once,
   afterwards, over a complete window.
4. ~~**`~EXWAIT` budget**~~ — ✅ **resolved,** and it turned out not to be the
   binding constraint: core1 comfortably services a register access inside the
   ~560 ns Z80 I/O window without needing wait states.
5. ~~**`0xDC`–`0xDE` control-bit meanings**~~ — ✅ **decoded** (0xDC b7 motor,
   b1..0 drive; 0xDD b0 side; 0xDE b0 density), with one correction below.
6. **CP/M BIOS is an unserved client.** Everything verified to date is the boot
   ROM's command repertoire. The BIOS issues its own sequence and currently
   trips `Read/Write Error (Abort/Retry/Ignore)` during startup. Diagnose with
   `fdclog` at the error prompt (names the opcode) plus `rddiag`; if the 16-deep
   ring has already wrapped, a sticky first-failure latch is the next tool to
   add. **Open — this is the blocker for a clean boot.**
7. **Read latency headroom.** Capture (~215 ms) consumes most of the host's
   ~285 ms patience. Comfortable margin needs DMA'd flux capture so decode can
   run concurrently and a sector is found as soon as its ID passes the head
   (~100 ms average). Not required today; required if WRITE SECTOR or a slower
   host path proves tighter. **Open.**

## Machine quirks that cost real time (read this before debugging)

- **Side-select is inverted.** The MZ-800's floppy card drives side-select the
  opposite way round from the PC/Shugart convention `FloppyDrive::side()` uses:
  **the ROM's side 0 is physical head 1.** The bridge inverts it by default
  (`fdc_set_side_invert`, default *on*); `mzp_probe.py sideswap 0` restores the
  raw mapping for A/B testing. Without the inversion the machine cannot boot.
  This is why every MZ-800 CP/M image has its 16×256 IPL track at cyl0/**side1**
  as a PC reads it, while the boot ROM reads logical track 0 = cyl0/side0.
- **The boot ROM's disk geometry** (from `9Z_504M.dis.asm`): logical track =
  `cylinder*2 + side` (`FD_PREPARE_TRACK_AND_SIDE`, E61B); block → track/sector
  is `track = block/16`, `sector = block%16 + 1` (E696), i.e. **16 sectors of
  256 bytes** per track, numbered 1..16 (`CP $11` wrap). The read loop is
  `LD B,$00` + `INI` — exactly 256 bytes per sector, regardless of the sector's
  actual size, then Force Interrupt.
- **"FD:Not master"** is a 7-byte compare of the loaded block at `$CF00`
  against E4CA = `03 49 50 4C 50 52 4F` (`0x03 "IPLPRO"`), stored complemented
  on disk because `INI` keeps the raw inverted bus byte. It is a *content*
  verdict, not an I/O error — a correct response to a non-master disk.

## Post-mortem: what actually blocked 3.2 (superseding the 2026-07-24 notes)

Three unrelated faults compounded, and each produced misleading evidence that
justified the wrong fix. Recorded because the wrong conclusions were written
into the code as "workarounds":

1. **Data-bus bit7 (D7) was a marginal solder joint on U2.** Every WD command
   whose *true* opcode has bit7 set lost that bit through the ROM's `CPL`:
   `0x94` arrived as `0x14`, `0xD8` as `0x58`. This spawned the
   `FDC_ALIAS_14_TO_94_AFTER_1B` / `FDC_ALIAS_58_TO_D8_AFTER_94` aliases, the
   belief that "the ROM issues Verify seeks" (it does not — `0x14` *was* `0x94`,
   and `mfm_verify_track()` was written for a command never sent), and the
   "intermittent command sequence" (the joint was flaky, not the ROM). Found
   with the port-0xDF loopback: 100 % of mismatches were one bit, one direction.
2. **The ~10 ms host timeout never existed.** Measured at ~285 ms once commands
   decoded correctly. The old figure timed a read that wasn't happening. Every
   design decision derived from it — index sync, pre-arm, "batch capture can
   never work" — was unnecessary, and all of it has been removed.
3. **Reads returned RNF because decode ran between flux drains.** See 3.2 above.
   The bench paths passed because none of them interleave.

Method note: the bus-test counters were `uint16_t` and wrapped every 0.5 s under
a tight `OUT` loop, fabricating a rate-dependent error curve that looked exactly
like a latency problem. All cycle counters are 32-bit now. Prefer a *signature*
(which bits, what xor, what delta) over a percentage.
