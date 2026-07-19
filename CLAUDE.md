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
- **Decode `0xDC`–`0xDE` bits** (motor / drive-select / side / density) from the
  ROM.
- **`~EXWAIT` budget** for putting a byte on the shared C0..C7 bus in time on a
  register access.
- **WRITE SECTOR** in-place splice details (chosen approach) — timing/gap
  handling; add write-precomp only if inner-track writes prove marginal.

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
- **Phase 3 (WD FDC + MZ-800 bus):** real-time WD179x bridge per
  [docs/wd-fidelity.md](docs/wd-fidelity.md) — core1 register/bus side (adapt the
  DELUXE-style `bus.hpp` to the shared C0..C7 scheme; inverted bus; polled DRQ;
  `~INT`/`~EXWAIT`), core0 disk side. Refactor the reference `fdc.cpp` WD state
  machine onto the streaming disk engine. Testable on real MZ-800 (its ROM has
  floppy support — no boot loader needed).
- **Phase 4 (robustness):** write-precomp if needed, density/`0xDC`–`0xDE`
  decode, multi-drive, error handling.
