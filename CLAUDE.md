# MZPico-floppy-firmware

Firmware for the **MZPico-800-Floppy-Board** — a Raspberry Pi Pico (RP2040)
cartridge that lets a **Sharp MZ-800** use a **real, physical 5.25"/3.5" floppy
drive**. The Pico sits on the MZ-800 expansion bus, emulates the machine's
floppy disk controller (FDC), and translates FDC sector reads/writes into
flux-level operations on an attached Shugart/PC floppy drive.

> This repo is new/empty. This file captures the design intent and the two
> upstream codebases it is built from so work can start with full context.

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
- **Apply density to the DENSEL pin.** `0xDC`–`0xDF` bits are all decoded now
  (see the register map in the spec), but `0xDE` is only latched — it does not
  yet drive floppy pin 2 or pick the bit time. Deferred: MZ-800 media is DD.
- **`~EXWAIT` budget** for putting a byte on the shared C0..C7 bus in time on a
  register access.
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
  One bodge in place: 1 kΩ pull-up on `~RDATA` (GPIO20) to 3.3 V.
- **Phase 1 (Pico-SDK greaseweazle firmware): SCAFFOLDED** — a bare Pico-SDK +
  TinyUSB firmware that speaks the greaseweazle protocol, ported from
  Adafruit_Floppy. Builds to `mzpico_floppy.uf2`. See [README.md](README.md).
  Layout:
  - `src/main.cpp` — clock/USB init + main loop
  - `src/gw_protocol.*` — greaseweazle command loop (CDC)
  - `src/floppy.*` — drive control (select/motor/seek/side/sense)
  - `src/flux/arch_rp2.*` + `greasepack.h` — RP2040 PIO flux capture/write
  - `src/board_config.h` — verified GPIO map
  - `src/usb_descriptors.c`, `src/tusb_config.h` — TinyUSB CDC
  - `CMakeLists.txt`, `pico_sdk_import.cmake`
  - `external/pico-sdk` — git submodule pinned to **2.2.0** (its `lib/tinyusb`
    submodule is the only inner one needed).
  **Builds clean** (pico-sdk 2.2.0 + arm-none-eabi-gcc 13.2 →
  `mzpico_floppy.uf2`, 35 KB flash / 151 KB RAM) **and is hardware-verified**:
  `gw` enumerates it and `rpm` / full `read` / `write` all work against real
  disks — the complete flux read+write path runs on this firmware.
- **Phase 2 (streaming MFM): NEXT** — vendor Adafruit `mfm_impl.h`; build a
  **streaming** MFM data-separator (flux → sectors with ID/CRC + rotational
  position) and encoder (sectors → flux, incl. in-place write-splice). Validate
  over USB (dump/patch a sector) before any MZ-800. No cache.
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
  - 3.4 leftovers: READ TRACK, WRITE TRACK (format), density applied to
    `~DEN_RDY`, drive-select via `~DS0/DS1`.
- **Phase 4 (robustness):** write-precomp if needed, multi-drive, error handling,
  soak testing.
