# MZPico-floppy-firmware

Firmware for the **MZPico-800-Floppy-Board** — a Raspberry Pi Pico (RP2040)
cartridge that lets a **Sharp MZ-800** use a **real, physical 5.25"/3.5" floppy
drive**. The Pico sits on the MZ-800 expansion bus, emulates the machine's
floppy disk controller (FDC), and translates FDC sector reads/writes into
flux-level operations on an attached Shugart/PC floppy drive.

> This file captures the design intent, the two upstream codebases this repo
> was built from, and the current state of the work (Phases 0–3 implemented
> and hardware-verified — see Status at the bottom).

## What makes this different from the reference firmware

The existing MZPico firmware (`../MZPico-firmware-reference`) emulates an FDC
backed by **`.DSK` image files** on flash/SD. This project keeps the *MZ-800
facing* half of that design but replaces the *storage* half: instead of reading
bytes from an image file, it reads/writes sectors from a **physical floppy
drive** using the flux capture/encode engine from `../Adafruit_Floppy`.

```
   MZ-800  <== expansion bus ==>  RP2040 Pico  <== 34-pin floppy ==>  Floppy drive
             (FDC emulation)                      (flux read/write)
        [from MZPico-firmware-reference]      [from Adafruit_Floppy]
```

## The two upstreams (both are sibling working dirs)

- **`../MZPico-firmware-reference`** — source of the MZ-800 side:
  - Pico-SDK (C/C++17) project, PIO-driven expansion-bus servicing on core1.
  - `src/mz_devices/fdc.cpp/.hpp` — the MB8877/WD177x-style FDC state machine
    (status/track/sector/data registers, commands, EXWAIT handshaking, INT).
    This is the behavior to reproduce; only its **byte backend** changes.
  - `src/bus.hpp`, `src/bus_io.pio`, `src/device.cpp` — bus timing, pin map,
    IORQ/RD/WR decode, port dispatch. **Note: that pin map is for the Frugal/
    Deluxe boards, NOT this Floppy Board — the GPIO assignments differ.**
  - `MZDevice` base class + `.ini` config model for port assignment.
- **`../Adafruit_Floppy`** — source of the drive side:
  - `Adafruit_Floppy` class: `select`, `spin_motor`, `goto_track`, `side`,
    `capture_track`, `write_track`, MFM encode/decode.
  - `arch_rp2.cpp` / `arch_rp2.h`: `rp2040_flux_capture` / `rp2040_flux_write`
    (PIO-based). This is an **Arduino** library; porting to bare Pico-SDK
    (or vendoring just the RP2040 flux + MFM code) is a key early decision.

## Target hardware — MZPico-800-Floppy-Board

KiCad design in `../MZPico-800-Floppy-Board` (see its `CLAUDE.md` for the full
extraction). The board uses the **purple AliExpress RP2040 clone with 16 MB
flash** (`Pico_Purple_16MB`), which exposes the extra GPIOs — the firmware
**relies on GPIO23/24/25/29 being usable** (they're internal on a genuine Pico).

### GPIO map (from the schematic — active-low = `~`)

| GPIO | Signal | GPIO | Signal |
|-----:|--------|-----:|--------|
| 0–7  | `C0..C7` shared bus (addr/data) | 16 | `~MOTOR_ON` |
| 8    | `~IORQ` | 17 | `~WGATE` |
| 9    | `~RD`   | 18 | `~WDATA` (flux out) |
| 10   | `~WR`   | 19 | `~HDSEL` |
| 11   | `~EXWAIT` | 20 | `~RDATA` (flux in) |
| 12   | `~INT`  | 21 | `~INDEX` |
| 13   | `~DEN_RDY` | 22 | `~TRK00` |
| 14   | `~STEP` | 23 | `~WPT` |
| 15   | `~DIR`  | 24 / 25 | `~DS0` / `~DS1` |
| 26   | `~EN0` (addr buffer /OE) | 28 | `~RD1` (data buffer DIR) |
| 27   | `~EN1` (data buffer /OE) | 29 | `RESET` (bus reset in) |

**Shared 8-bit bus:** the MZ-800 address and data are time-multiplexed onto
`C0..C7` (GPIO0–7) via two `SN74LVC245` buffers. Assert `~EN0` to read the I/O
**address**; switch to `~EN1` (+ set `~RD1` direction) for the **data** phase.
Never assert both `~EN0` and `~EN1` at once. This is the reference DELUXE-board
scheme (`DATA_BUS_BASE=0`, EN pins) on different GPIOs — adapt `bus.hpp`
accordingly rather than reusing FRUGAL's separate 16-pin addr+data layout.

Flux-critical PIO pins: `~RDATA` (GPIO20 in), `~WDATA` (GPIO18 out),
`~WGATE` (GPIO17), `~INDEX` (GPIO21).

## Key design decisions (✅ = resolved)

> Full FDC spec: **[docs/wd-fidelity.md](docs/wd-fidelity.md)** (register map,
> inverted bus, the ROM's polled-DRQ handshake, commands, architecture). The
> MZ-800 side is derived from the Disk BASIC ROM `MZ2Z046.ASM` (in repo root).
> Boot-ROM (9Z-504M) floppy port map and call flow:
> [docs/9Z_504M_floppy_routines.md](docs/9Z_504M_floppy_routines.md).
> ([docs/rom-first-validation.md](docs/rom-first-validation.md) +
> `tools/rom_first_validation.sh` are historical — pre-3.2; e.g. the
> index-synchronized read start described there was later removed.)

- **Emulation goal:** ✅ be **as faithful as possible to the original WD FDC**
  — target the **WD179x / FD1791-02** register+command model (WD1773/WD2797 are
  compatible supersets). Keep the reference `fdc.cpp` WD state machine and refine
  it for fidelity; back it with the **real drive in real time** (not an image).
- **Async, not stall-and-wait:** ✅ a faithful WD is inherently async
  (command → **BUSY** → **DRQ** per byte → **INTRQ**; CPU polls STATUS). So the
  physical seek+read (tens of ms) runs in the **background while the CPU polls**,
  NOT while the Z80 is held in a WAIT state. `~EXWAIT` is used only for µs-scale
  register-access sync — never for the whole physical latency (which would stall
  Z80 refresh / the MZ-800's DRAM). This replaces the reference's stall-based
  shortcut (fine for its instant flash/RAM storage, wrong for a real drive).
  With a real drive, index/seek/rotation timing can be **real**, not synthesized.
- **MZ-800 boot:** ✅ **the MZ-800 ROM already has floppy support** — it drives
  the FDC at ports 0xD8+ directly. No SRAM boot-loader injection needed (unlike
  the reference's other devices). Phase 3 is testable on real MZ-800 hardware
  directly.
- **No cache — real-time bridge:** ✅ the physical disk IS the storage, read/
  written in **real time** (no track cache, no on-device EDSK). This is what
  preserves fidelity — rotational latency, real CRC, deleted marks, and
  **weak-bit copy protection** all fall out for free, because we ride the real
  disk each revolution. A cache would defeat weak-bit protection; it may return
  later only as an optional read-ahead. (Weak bits are a media property, not a WD
  feature — see the spec.)
- **Data pacing = software DRQ polling:** ✅ the ROM driver polls STATUS and
  transfers a byte only when **DRQ** is set (it does *not* use wait-states for
  data). So we maintain accurate **BUSY/DRQ/RNF/CRC/NOT-READY** status; the CPU
  self-paces, and real-time decode makes DRQ toggle at the disk byte rate →
  authentic timing for free. `~EXWAIT` is used **only** for register bus-timing
  (µs), never the whole latency (no Z80/DRAM stall).
- **Inverted data bus:** ✅ the ROM `CPL`s every FDC byte — emulated registers
  must present/accept **bit-complemented** values on `0xD8`–`0xDB`.
- **Multi-sector + Force Interrupt:** ✅ the ROM uses Read/Write **Multiple**
  (`0x90`/`0xB4`) terminated by **Force Interrupt** (`0xD0`) — must be supported.
- **Head position is ours, the Track register is the host's:** ✅ core1 folds each
  Type I step into a **clamped absolute** `head_target` (our model of the head);
  core0 drives the head there. The two must be free to disagree — that is the
  basis of the recalibrate-via-READ-ADDRESS idiom. Three rules fall out of how
  the host actually behaves (CP/M 4.1 fires STEP/`FORCE-INT`/READ bursts µs
  apart and never waits for BUSY, while our steps take ms):
  1. never drop a queued Type I — several arrive while core0 is blocked in a
     ~235 ms capture;
  2. clamp **per command**, so surplus STEP-OUTs past TRK00 are absorbed as a
     real drive absorbs them, not summed against later STEP-INs;
  3. never start a capture while `op_done != op_seq` — the read must wait for
     the pending move or it answers from the previous cylinder.
- **WD ID track compare:** ✅ a read matches an ID only when **both** its sector
  and its cylinder byte agree with the Sector/Track registers; a sector-number
  match on the wrong cylinder is RNF, not data. This is real WD179x behaviour and
  it is the only thing that makes a positioning bug visible — without it the host
  silently receives CRC-clean data from wherever the head happens to be, and
  every status bit reports a perfect read.
- **On-device MFM (streaming):** ✅ required from Phase 2 (Phase 1 was flux-only,
  host-side MFM). Vendor Adafruit's `mfm_impl.h` and make the decode/encode
  **streaming** (rotational-position tracking), not batch-to-buffer.
- **Geometry:** ✅ format-agnostic — MFM IDAMs give each sector's id/size, which
  is exactly what the WD variable-geometry logic expects. Density via `~DEN_RDY`.
- **Core/PIO split:** ✅ **core1** = host/register side (bus PIO, WD registers,
  STATUS, DRQ/BUSY, `~INT`, `~EXWAIT`); **core0** = disk side (streaming flux
  decode/encode, ID/CRC, write-splice, STEP/DIR/WGATE). core0's IRQ-disable
  during flux doesn't stall core1. Fit bus_io SMs + flux SM across pio0/pio1.
- **Build system:** ✅ Pico-SDK + CMake, pico-sdk 2.2.0 submodule (done, Phase 1).

### Still open
- ~~**Apply density to the DENSEL pin.**~~ — ✅ superseded by **HD support**,
  and the `0xDE` mystery is solved by observation: CP/M 4.1 writes the port
  (`0x00` all through DD work, `0x81` entering an HD format), so **b0 = 1 =
  HD** and a host declaration now outranks the auto-detect latch. The Disk
  BASIC ROM still never writes it, which is why the per-drive latch remains
  the boot-time fallback. DENSEL is driven best-effort from the same model.
- **HD support: write path hardware-verified** (HD `fmtverify` track scans
  back CRC-clean at 500 kbit/s; `enctest 18 2 1` round-trips). A full CP/M HD
  format ran up to FORMAT4's spindle-timing phase, which exposed the missing
  Type I Index bit (now live — see the Status entry); the end-to-end HD
  format is the remaining verification.
- ~~**`~EXWAIT` budget**~~ — ✅ resolved: core1 comfortably services a register
  access inside the ~560 ns Z80 I/O window without wait states (see
  [docs/phase3-plan.md](docs/phase3-plan.md), risk #4).
- **Write-precomp: no longer suspected.** `mzp_probe.py PORT wrsoak` ran
  **200/200 writes verified byte-for-byte** over tracks 0–76 (step 4), both
  sides, 5 passes — including the innermost tracks, where flux density is
  highest and precomp would show up first, and the 16×256 boot track, which is
  the only 256-byte geometry we have. Zero failures, no clustering. Precomp
  stays unimplemented until a drive or media actually misbehaves. What is still
  worth doing: a long soak (hours, not minutes) and a second drive/disk, since
  the +2-byte splice bias has only met one spindle.
- **Rare `FD:Loading error` on boot** (~1 in 20; everything else reads clean).
  The **first-failure latch** it needed now exists (`mzp_probe.py firstfail`):
  it holds the first RNF/CRC/wrong-cylinder/Verify failure since power-up in
  full, with the whole position chain sampled at that instant, so a host's
  successful retry can no longer erase the evidence. Still to do: catch one.
  `at_ms` is the discriminator — under ~3 s points at a capture taken before the
  spindle was stable rather than at the decode path; the suspects otherwise are
  the 5 ms `settle_delay_ms` right after a seek.
- ~~**`goto_track()` assumes the head arrived**~~ — ✅ addressed, two ways.
  `goto_track()` now cross-checks its belief against the TRK00 sense on every
  call and forces a recalibrate when they disagree (the belief is derived from
  moves we *asked* for, so a lost STEP pulse used to become a permanent
  off-by-one that the early return then prevented us from ever fixing). And the
  FDC now treats a wrong-cylinder ID as a **position fix, not just an error**:
  `fdc_resync_head()` adopts the cylinder the disk reported and drives back to
  `head_target`, so the host's retry no longer repeats from the same wrong
  place forever. Counted as `pos_resyncs` in `firstfail`. This turns the WD ID
  track compare from a detector into a repair — which matters much more now
  that a mispositioned head can destroy data rather than just garble a read.

## Hard-won rules

- **Nothing goes in core1's listen loop.** Not a flash-resident call, not a
  poll, not two instructions. It has ~560 ns to answer a Z80 I/O cycle, and
  anything added per-pass makes core1 late — at which point the MZ-800 reports
  **"no floppy driver detected"**, which is indistinguishable from dead
  hardware and costs a session to diagnose. Bus RESET had to become an edge
  interrupt for exactly this reason. Anything that must watch a line either
  interrupts, or lives in PIO.
- **Bisect before theorising.** When a known-good build exists and a new one
  fails, build the midpoint and flash it. Reasoning from indirect evidence lost
  most of a session here, through four wrong hypotheses, while two flashes
  settled it. `build/known-good/` holds the uf2s from that bisect.
- **Run `mzp_probe.py PORT health` first, every session.** It verifies which
  firmware build is actually running (`fwver` — a whole test cycle was once
  spent against the wrong uf2) and screens the passive counters for the
  recurring **edge-connector contact faults**: random `FD:Loading error`s
  cured by reseating the cartridge or cleaning the fingers with IPA are
  contacts, not code, and corrupted command bytes mimic impossible firmware
  bugs (the D7 incident). `busdump` remains the detailed per-line view; the
  `bustest` Z80 program (`tools/mzf/BUSTEST.MZF`: `bustestreset`, load+run
  on the MZ-800, `bustest`) the definitive active measure. Note the summary only sees bytes
  the ROM actually wrote, so a line reported "stuck" may simply never have
  needed to be 1 — check the union of the observed `last_raw_write` values
  before believing it. (ADDRESS bits 6/7 always read stuck high: every port we
  own, 0xD8–0xDF, has both set.)

## Conventions (inherited from the reference firmware)

- Pico-SDK, CMake, C11 / C++17. RP2040 (consider RP2350). Overclock (~180 MHz).
- Direct SIO register access + PIO for bus and flux timing; `RAM_FUNC` /
  `ALWAYS_INLINE` for hot paths (see `bus.hpp`, `common.hpp` in the reference).
- FDC code carries a **GPL v2+** notice (ported MZ800 emulator origin) — keep
  license headers intact when adapting `fdc.cpp`.

## Status

- **Phase 0 (hardware bring-up): DONE** — the board reads *and* writes real
  floppies over USB via the Adafruit greaseweazle Arduino sketch. See
  [docs/PHASE0.md](docs/PHASE0.md) and [docs/hardware-notes.md](docs/hardware-notes.md).
  One bodge in place: **4k7** pull-up on `~RDATA` (GPIO20) to 3.3 V (too weak for HD — a 1 kΩ / 680 Ω swap is the first HD write-splice fix to try; see docs/hardware-notes.md RDATA crosstalk item).
- **Phase 1 (Pico-SDK greaseweazle firmware): DONE** — a bare Pico-SDK +
  TinyUSB firmware that speaks the greaseweazle protocol, ported from
  Adafruit_Floppy. Builds to `mzpico_floppy.uf2`. See [README.md](README.md).
  Phase 1 layout:
  - `src/main.cpp` — clock/USB init + main loop
  - `src/gw_protocol.*` — greaseweazle command loop (CDC) + the `mzp_probe.py`
    bench command handlers
  - `src/floppy.*` — drive control (select/motor/seek/side/sense)
  - `src/flux/arch_rp2.*` + `greasepack.h` — RP2040 PIO flux capture/write
  - `src/board_config.h` — verified GPIO map
  - `src/usb_descriptors.c`, `src/tusb_config.h` — TinyUSB CDC
  - `CMakeLists.txt`, `pico_sdk_import.cmake`
  - `external/pico-sdk` — git submodule pinned to **2.2.0** (its `lib/tinyusb`
    submodule is the only inner one needed).
  **Builds clean** (pico-sdk 2.2.0 + arm-none-eabi-gcc 13.2) **and is
  hardware-verified**: `gw` enumerates it and `rpm` / full `read` / `write` all
  work against real disks — the complete flux read+write path runs on this
  firmware.
- **Phase 2 (streaming MFM): DONE** — Adafruit `mfm_impl.h` vendored
  (`src/flux/mfm_impl.h`); the decode/encode/splice layer lives in:
  - `src/mfm_decode.*` — flux → sectors with ID/CRC (`mfm_read_sector`,
    `mfm_scan_track`, `mfm_read_address`, `mfm_verify_track`,
    `mfm_read_track_raw`)
  - `src/mfm_encode.*` — WRITE TRACK format-image → flux (WD write-channel
    bytes `F5`/`F6`/`F7`), plus gap fill
  - `src/mfm_splice.*` — in-place data-field write splice (`mfm_write_sector`)
  Validated over USB (dump/patch a sector) before any MZ-800. No cache.
- The MZ-800 side added in Phase 3 lives in `src/bus/`:
  - `src/bus/mz_bus.*` + `bus.pio` + `board_bus.h` — shared-bus PIO, core1
    listen loop, port decode, `~EXWAIT`
  - `src/bus/fdc.*` — the WD179x bridge (core1 register file + core0 disk task)
  - `tools/mzp_probe.py` — the USB bench/diagnostic probe
- **Phase 3 (WD FDC + MZ-800 bus): READ AND WRITE PATHS DONE — CP/M 4.1 boots
  from a physical disk and writes files back to it.** Real-time WD179x bridge per
  [docs/wd-fidelity.md](docs/wd-fidelity.md); core1 register/bus side, core0 disk
  side. Milestone detail in [docs/phase3-plan.md](docs/phase3-plan.md):
  - 3.0 shared-bus bring-up ✅, 3.1 Type I + STATUS ✅, 3.2 READ SECTOR
    single+multiple ✅, READ ADDRESS ✅, DRQ-driven EINT-gated `~INT` ✅.
  - **Read-path fidelity pass ✅** (CP/M 4.1 was reading whole directories off
    the wrong cylinder — see the head-position decision above). Three fixes:
    the seek handshake no longer drops queued Type I commands, a Type II no
    longer captures while a move is still pending, and `mfm_read_sector`
    enforces the WD ID **track compare**. `mzp_probe.py rddiag` now reports the
    whole position chain (Track reg → head_target → drive belief → disk cyl)
    plus cumulative `sec_ok`/`sec_crc_err`/`sec_rnf`/`sec_cyl_mismatch`.
  - **3.3 WRITE SECTOR: DONE (hardware-verified) -- CP/M 4.1 on the MZ-800
    writes files to a physical disk and reads them back correct.** Measured over
    two disks (13 and 3 host writes): every write completed, and `wrdiag`
    reported zero RNF, zero wrong-cylinder, zero too-late and zero LOST DATA --
    so the host's STEP/FORCE-INT positioning style, which is what makes a write
    dangerous, never landed a splice off-target. A `scan` of a written track
    shows all nine sectors still ID+data CRC clean: the splice stays inside its
    own data field on both ends. In-place
    data-field splice — a *live* MFM decoder finds the sector's ID as it passes
    the head, then WGATE goes on for the Gap 2 window and the new data field is
    written; nothing outside that one field is rewritten. core1 buffers the
    whole sector from the host over DRQ first, so a stalling host stalls before
    any flux is committed. Refused (before flux) on `~WPT`, on `wren 0`, and on
    an ID **track compare** miss — the host's positioning style makes that last
    one critical here, since a splice at the wrong cylinder destroys data
    instead of merely returning garbage. Verify with
    `mzp_probe.py PORT wrverify <track> <sector>` (writes, reads back, compares)
    plus `wrdiag`, on a scratch disk, before letting the MZ-800 write.
  - **3.4 WRITE TRACK (format) + READ TRACK: DONE.** Bench path
    hardware-verified — formats a track that scans back with all sectors and
    CRCs valid; READ TRACK decodes ID fields, gaps and data correctly off real
    media. The host command path is **verified from the MZ-800**: CP/M 4.1's
    `FORMAT4` formats a disk through us (see the multi-drive bullet below), and
    host formatting defaults **ON** (`fmten`; see the defaults-ON bullet below
    for why the old default-off gate was retired). WRITE TRACK obeys the WD write-channel bytes (`F5`
    0xA1+preset CRC, `F6` 0xC2, `F7` CRC, else literal) — the missing-clock
    sync marks it creates are the reason formatting cannot be expressed as a
    run of sector writes. It runs index-to-index, which makes it *safer* than
    the sector splice despite replacing the whole track: there is no timing
    window to hit. Verify with `mzp_probe.py PORT fmtverify <track>` on a
    scratch disk. READ TRACK returns every byte uninterpreted (`readtrack`) and
    is verified on real media — ID fields, 22-byte Gap 2, presync and data all
    decode correctly; one cosmetic quirk remains (data-field sync runs print as
    five 0xA1s rather than three) and is deliberately left.
    Density→DENSEL is closed as "not to be done" (the ROM never writes the
    port, so its polarity is unknowable and nothing needs it).
  - **Multi-drive (`~DS0`/`~DS1`): DONE — verified on two physical drives and
    end-to-end under CP/M 4.1.** Two
    `FloppyDrive` instances sharing every pin but `~DSn`, selected by port
    0xDC's drive field; per-drive head position, head target and sector-size
    cache; shared-line handover on each select (`~HDSEL`/`~MOTOR_ON` are one
    wire each, so the incoming drive's beliefs about them are stale by
    definition). Units 2/3 report NOT READY. Bench: `mzp_probe.py drive [0|1]`,
    escape hatch `drvlock 1`. An absent/empty unit reports NOT READY (from the
    index sighting, which was previously discarded). **Verified on two physical
    drives:** each unit reads its own media; `drvstat` shows the heads parked on
    different cylinders across selects (40 and 0) and drive 0 reads cylinder 40
    afterwards without re-stepping; the `wp` field in `drvstat` follows the
    write-protected disk across selects. **Verified end-to-end under CP/M 4.1:** booted from drive A,
    `FORMAT4 B: /720` formats drive B, files copy to it, and `copydisk`
    completes. Only the catch-up seek on select is unproven — it needs host
    Type I commands to create the divergence. See
    [docs/phase3-plan.md](docs/phase3-plan.md).
    - **A write must not need the sector length in advance.** It used to take it
      from the per-cylinder size cache, so writing a disk nobody had read used
      the 512-byte default and an MZ-800 master disk's 16x256 boot track
      (cyl 0 / side 1) silently wrote nothing — no RNF, no LOST DATA, no
      refusal, because an unfilled buffer is none of those. A gap in the host's
      byte stream now ends the sector, as WRITE TRACK already did, and the ID
      field's size code is the authority.
    - **Host formatting now defaults ON.** The old default-off gate claimed a
      glitched command byte could reformat a track; that stopped being true once
      the format image was validated — a spurious WRITE TRACK collects no image
      and writes nothing. Meanwhile a refusal is indistinguishable from a frozen
      machine (STATUS 0x40, BUSY clear, host polls forever) and it reset on every
      reflash, so it recurred. The image check was strengthened in exchange: it
      now requires a real IDAM preamble (≥2 × `0xF5` immediately followed by
      `0xFE`) rather than those bytes appearing anywhere, which arbitrary sector
      data satisfies ~86% of the time. `wrdiag` names which of ~WPT / `wren` /
      `fmten` refused a format.
    - `~MOTOR_ON` reaches every drive on the cable (J3 wires it to floppy pins
      10 *and* 16), so it is **not** gated by drive select: selecting an absent
      drive still spins the present one. Not a bug — one spindle cannot be run
      without the other on this board.
    - **Bench commands must go through `fdc_bench_drive()`.** `scan`/`streamscan`
      originally used the greaseweazle drive object, hardwired to `~DS0`, so
      `drive 1; scan` silently read drive 0 — and `fmtverify` would have
      formatted one disk and verified another.
  - **HD (500 kbit/s) support: implemented, not yet hardware-verified.** Target
    is 3.5" HD at 300 rpm — the revolution time is unchanged, only the bit
    clock halves. The capture window stays **byte-sized** (~1.05 rev:
    45700 bytes DD / 91400 HD) — byte windows are rotation-relative, adapting
    to the spindle's real speed, where a time window on a slightly-slow drive
    captured 0.96 rev and threw phase-dependent CRC errors (a lesson learned
    the hard way; see `fdc_read_capture_bytes`). Density (`mzp_probe.py density dd|hd|auto|detect`, default auto):
    a valid host `0xDE` declaration decides, else a per-drive latch (default
    DD) — **no media probing by default**, so the boot-ROM/DD read path is
    the pre-HD behaviour exactly; `detect` adds probing (a nothing-decodes
    capture re-decoded at the other bit clock, CRC-valid ID flips the latch)
    for `0xDE`-less hosts with HD media, which this machine does not have. **Port
    `0xDE` is the host's density declaration and, when valid, is trusted
    OUTRIGHT in auto mode** — a real WD never second-guesses its DENSEL
    input. Polarity observed from CP/M 4.1 (`0x00` throughout DD operation,
    `0x81` entering an HD format): b0 = 1 = HD; cleared on bus RESET so the
    boot ROM falls back to the latch. The instrumented track-write ring
    proved this BIOS declares before **every** operation, always correctly —
    including deliberately declaring **DD for the 16×256 boot track (cyl 0 /
    head 1) of an HD disk**, so the DD-only boot ROM can still boot it: HD
    data tracks, DD boot track, one disk, by design. Two generations of
    census-outranks-declaration logic each broke a track before that was
    understood (the collection tail is host memory junk that can carry
    phantom IDAMs and inflate the census past the HD threshold — we clock in
    more bytes than a real WD, whose DRQ stops at the index). A host WRITE
    TRACK therefore uses the image's **sector census** (declared sectors ×
    size codes, ~5.3k for 9×512 DD vs ~10.4k for 18×512 HD; never the image
    length) only when **no** declaration exists — Disk BASIC-era software.
    Writes never auto-detect (the splice uses the effective density).
    A status-only verify read (host drains nothing — FORMAT4's idiom, legal on
    a real WD) completes after 15 ms instead of the 250 ms LOST-DATA budget,
    which is what made an HD format ~11 min. The **Type I STATUS Index bit (b1) is live from `~INDEX`
    only while the host has declared HD** (`idxbit 0|1|2`): FORMAT4's HD
    path times the spindle by polling it and froze forever against the old
    hardwired 0, but an earlier always-live experiment regressed the boot
    ROM's RESTORE check, so the DD/boot world keeps the hardwired 0. DENSEL is
    driven best-effort (3.5" drives sense HD from the media hole). Buffers are
    HD-sized (track image 13000 bytes, format flux budget 96k; the 150 KB
    capture buffer already fit a full HD revolution). Verify bench-first:
    `enctest 18 2 1` (no disk), then `density hd; fmtverify` on a scratch HD
    disk. See [docs/wd-fidelity.md](docs/wd-fidelity.md). **HD read/format/bench-write are
    hardware-verified; the HD write SPLICE under CP/M is blocked by RDATA vs
    C0..C7 crosstalk at the 1 µs bit cell** (~7% of splices see corrupted
    flux edges while the host storm-polls STATUS; DD's 2 µs cell is immune,
    DD is 530/530 clean). Every firmware cause was instrumented out (PIO
    drops nothing; reads through the same pin are clean and retry). It is a
    signal-integrity limit of the bare board -- see the RDATA-crosstalk item
    in [docs/hardware-notes.md](docs/hardware-notes.md) for the fixes (33 Ω
    series on the bus, grounding, or the buffered input stage) and the
    index-relative-splice firmware mitigation.
- **Phase 4 (robustness):** write-precomp if needed, error handling, long soak
  testing (the `wrsoak` run so far is minutes, not hours, and has met only one
  spindle), catching the rare boot `FD:Loading error` with `firstfail`.
