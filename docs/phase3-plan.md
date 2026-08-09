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
- **3.3 — WRITE SECTOR (in-place splice). ✅ DONE (hardware-verified) — CP/M 4.1
  on the MZ-800 writes files to a physical disk and reads them back correct.** Locate the sector by ID with a *live* MFM decoder, then
  WGATE + write the MFM-encoded data field into the ~22-byte Gap 2 window
  (~700 µs slack). Code: [src/mfm_splice.cpp](../src/mfm_splice.cpp) (the
  splice), `mfm_encode_data_field` (the payload), the write block in
  `fdc_disk_task` and the `0xAX/0xBX` branch of `fdc_reg_write`.
  - **Why a live decoder and not the buffered one.** A read may capture a whole
    revolution and decode it afterwards; a write cannot — by the time a buffer
    has been decoded the sector has long since passed the head. So the search
    pulls one transition at a time straight off the capture FIFO and decodes ID
    fields as they arrive. Per transition that is a subtract, a compare and a
    shift (~0.2 µs against a ~4 µs DD interval), which is why it keeps up where
    the batch decoder — O(n²), rescanning from the start — never could. That
    distinction is the whole content of the "no decode while capturing" rule in
    `fdc_capture_window()`; it bans the *buffered* decoder, not live decoding.
  - **Only the data field is rewritten** — 12 zeros, the 3-byte 0xA1 sync, the
    DAM, the data, its CRC and 3 gap bytes to carry the write-off splice. ID
    fields, Gap 2 and the rest of Gap 3 stay as recorded, so sector size,
    interleave and numbering all survive, and nothing outside one sector's data
    can be touched by a bug in this path.
  - **Splice timing.** The gap is measured from the last bit of the ID CRC, with
    the flux still queued in the FIFO subtracted off (that is disk which has
    already passed the head). The bias is deliberately +2 bytes: late merely
    eats presync we rewrite anyway, early destroys the ID field.
  - **Buffer first, then write.** core1 collects the whole sector from the host
    over DRQ, then core0 splices. A host that stalls mid-sector therefore stalls
    *before* any flux is committed (→ LOST DATA, nothing written) instead of
    half way through a data field. A real WD, streaming host bytes straight to
    the media, answers that case by writing zeros — i.e. by corrupting the
    sector. Cost: the sector's length must be known before the first host byte
    arrives, which is what the per-cylinder size cache (`FDC_SIZE_CACHE_N`,
    filled from the ID fields every read decodes) is for. A wrong guess is
    caught, not acted on, by the size compare at splice time.
  - **Three refusals, all before flux.** Physical `~WPT`, the firmware master
    switch (`mzp_probe.py wren 0`), and the WD ID **track compare** — which
    matters far more here than on a read: a read from the wrong cylinder returns
    garbage the host can retry, a write there destroys a track that was fine.
  - **Test path (do this before letting the MZ-800 write):**
    `mzp_probe.py PORT wrverify <track> <sector>` writes a pseudo-random pattern
    through the real splice, reads it back through the real read path and
    compares byte for byte. `wrdiag` reports the last splice and the cumulative
    written / RNF / wrong-cylinder / wrong-size / too-late counters. **Use a
    scratch disk.**
  - **Measured, bench:** 16/16 `wrverify` writes across two disks, search
    45-62 ms (a couple of sector periods from an arbitrary rotational start).
    A `scan` of a written track returns all nine sectors ID+data CRC clean --
    the neighbours either side of the spliced sector are untouched, which is the
    check that actually matters (the failure mode of a mistimed splice is a
    damaged *adjacent* sector, not a bad written one). NB: a scan captures ~1.05
    revolutions, so its last row is a wrapped partial re-read of the first
    sector and its "BAD" data CRC is a window artefact, not damage.
  - **Measured, MZ-800 under CP/M 4.1:** files written to two physical disks and
    read back correct; `wrdiag` after each session reported 13/13 and 3/3 writes
    completed with **zero** RNF, wrong-cylinder, wrong-size, too-late or LOST
    DATA. That is the result that matters: the host positions the head with
    STEP/FORCE-INT bursts it never waits on (see the head-position decision in
    ../CLAUDE.md), so wrong-cylinder was the plausible way for this to destroy
    data, and the `op_done != op_seq` gate plus the ID track compare held. Zero
    LOST DATA also says the host kept up with DRQ across a whole sector, and the
    ~60 ms splice search fits inside its patience.
- **3.4 — Type III / format / density / multi-drive.**
  - **READ ADDRESS (0xCX). ✅ DONE (hardware-verified).** Returns the first ID
    field in the captured flux as the six bytes a real WD hands over DRQ —
    track, side, sector, size code, CRC1, CRC2 — using the CRC bytes actually
    recorded on the disk, not a recomputation, so a host that checks them sees
    what the chip would have given it. Also loads the ID's track byte into the
    Sector register (WD quirk; the CP/M BIOS reads head position from there).
    This is what stopped the BIOS stepping blindly and giving up.
  - **WRITE TRACK (0xFX) — format. ✅ DONE (hardware-verified).** Formatting
    track 70 as 9x512 interleave 2 and scanning it back returns all nine
    sectors, in the interleaved physical order 1,6,2,7,3,8,4,9,5, every one
    with a valid ID **and** data CRC — which is the proof that the missing-clock
    sync marks, the once-per-run CRC preset and the `F7` CRC emission are all
    right, since any of the three being wrong makes the ID field unreadable.
    The host shifts in a whole track image and `mfm_encode_format_image()`
    obeys the four write-channel commands in it: `F5` writes 0xA1 with a
    missing clock bit and presets the CRC, `F6` writes 0xC2 the same way, `F7`
    emits the accumulated CRC, everything else is written literally. Those
    missing clock bits are the point — they make a sync mark unambiguous in a
    stream where any data pattern is legal, they cannot be produced by writing
    an ordinary 0xA1, and creating them is the reason formatting is a distinct
    operation rather than a run of sector writes.
    - **The CRC presets once per run of `F5`, not per byte.** A real track has
      three in a row and the CRC that follows must cover all three 0xA1s;
      resetting on each would leave it covering only the last, and every ID
      field on the disk would be unreadable.
    - **`F8..FB`/`FE` are not CRC-initialising here.** That rule is FM's, where
      there is no 0xA1 sync to do the job. In MFM they are ordinary bytes.
    - **Safer than the sector splice, despite being far more destructive.** It
      runs index-to-index, so there is no ~700 µs window to hit — the whole
      failure mode the splice is built around simply does not arise. Short
      images are padded with gap bytes so a track can never end up half one
      format and half the previous one, with the seam wherever the host
      stopped.
    - Gated exactly like WRITE SECTOR: physical `~WPT` and the `wren` switch,
      both checked before anything is destroyed. Test with
      `mzp_probe.py PORT fmtverify <track>` (format, then read the sectors
      back) on a scratch disk.
  - **READ TRACK (0xEX). IMPLEMENTED; first hardware run found a bug, fixed,
    awaiting re-test.** The first version dropped the rule governing the symbol
    immediately after a sync mark and so latched *clock* bits as data: gaps came
    back as 0x84 rather than 0x4E — not even a rotation of it, which is what
    identified the fault as a parity error rather than an arbitrary byte
    boundary — and no address mark was ever recognisable. Returns every
    byte on the track — gaps, address marks, CRC bytes — uninterpreted, via
    `mfm_read_track_raw()`. It is the one decode here that cannot anchor itself
    to a sync mark, since the point is to see what lies between them, so it
    free-runs and resynchronises whenever an 0xA1 passes. Byte boundaries in the
    gaps are therefore arbitrary — true of a real WD too, which is why READ
    TRACK is a diagnostic rather than a way to copy a disk. `mzp_probe.py PORT
    readtrack <track>` dumps it and counts the address marks.
  - **Density applied to `~DEN_RDY`: closed as "not to be done".** The ROM
    defines the port and never writes it, so there is no traffic to infer the
    bit's polarity from and the two documented readings disagree; driving
    DENSEL on a guess risks a machine that currently boots, for media this
    platform does not have. See wd-fidelity.md.
  - **Still to do:** drive-select via `~DS0`/`~DS1` and real multi-drive. Left
    alone deliberately: it needs per-drive head positions, and with one drive
    attached there is no way to test that a second select line does not simply
    deselect the working drive.
- **3.5 — Robustness.** Error/retry paths, edge-case timing, soak testing.
  - **Write precomp: not needed so far.** `wrsoak` verified 200/200 writes over
    tracks 0–76 (both sides, 5 passes, pseudo-random patterns), covering the
    innermost tracks where precomp would show up first and the 16×256 boot
    track. Left unimplemented deliberately — it is a fix for a problem this
    hardware has not exhibited.
  - **Head position is self-correcting** (see `fdc_resync_head`): a wrong
    cylinder byte in an ID field is adopted as the truth and the head driven
    back to target, so a lost step no longer persists. `firstfail` counts it as
    `pos_resyncs`; that counter climbing while everything appears to work is
    itself the signal that steps are being dropped.
  - **First-failure latch** (`mzp_probe.py firstfail`) holds the first
    RNF/CRC/wrong-cylinder/Verify failure since power-up, with the whole
    position chain sampled at that instant. Built for the ~1-in-20 boot-time
    `FD:Loading error`, which a host retry otherwise erases before anyone can
    look. **Still needs to catch one.**

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
