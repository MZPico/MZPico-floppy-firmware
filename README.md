# MZPico Floppy Firmware

Firmware for the **MZPico-800-Floppy-Board** (Raspberry Pi Pico / purple 16 MB
clone) — lets a **Sharp MZ-800** use a real, physical floppy drive. The Pico
emulates the MZ-800's floppy controller and drives an attached 34-pin Shugart/PC
drive at flux level.

See [CLAUDE.md](CLAUDE.md) for the full architecture and phase plan, and
[docs/PHASE0.md](docs/PHASE0.md) for the hardware bring-up that validated the
board.

## Phase 1 — greaseweazle-compatible flux firmware (this code)

A bare **Pico-SDK + TinyUSB** firmware that speaks the **greaseweazle** protocol
over USB, so the mature [`gw`](https://github.com/keirf/greaseweazle) /
[fluxengine](https://cowlark.com/fluxengine/) host tools can read and write real
disks through this board. This is the reusable foundation; the MZ-800 bus + FDC
emulation is added in later phases.

It is a port of the Adafruit_Floppy Arduino `greaseweazle` example to Pico-SDK:

| This firmware (Pico-SDK)        | Ported from (Adafruit_Floppy, MIT)           |
|---------------------------------|-----------------------------------------------|
| `src/flux/arch_rp2.cpp/.h`      | `src/arch_rp2.cpp/.h` (PIO flux capture/write)|
| `src/flux/greasepack.h`         | `src/greasepack.h` (verbatim)                 |
| `src/floppy.cpp/.hpp`           | `Adafruit_Floppy` drive control               |
| `src/gw_protocol.cpp/.hpp`      | `examples/greaseweazle/greaseweazle.ino`      |
| `src/usb_descriptors.c`, `tusb_config.h` | Arduino TinyUSB stack (replaced)     |

The greaseweazle protocol is flux-level, so **all MFM decode/encode happens on
the host** — no on-device MFM here (that arrives in Phase 2 for FDC sector
serving). `src/board_config.h` holds the verified GPIO map.

## Build

Needs CMake + the ARM GCC toolchain (`gcc-arm-none-eabi`). The Pico SDK is
vendored as a git submodule at `external/pico-sdk` (pinned to **2.2.0**), so no
`PICO_SDK_PATH` is required.

```bash
# after cloning this repo:
git submodule update --init external/pico-sdk
git -C external/pico-sdk submodule update --init lib/tinyusb   # only TinyUSB needed

cmake -B build -S .
cmake --build build -j
# -> build/mzpico_floppy.uf2
```

CMake auto-uses `external/pico-sdk` unless `PICO_SDK_PATH` is set in your
environment (in which case that SDK is used instead; force the submodule with
`-DPICO_SDK_PATH=external/pico-sdk`).

**Verified:** builds clean with pico-sdk 2.2.0 + arm-none-eabi-gcc 13.2 →
`mzpico_floppy.uf2` (~62 KB; 35 KB flash, 151 KB RAM incl. the flux buffer).

Flash: hold **BOOTSEL**, plug in USB (mounts `RPI-RP2`), copy
`mzpico_floppy.uf2` onto it.

## Use with greaseweazle

The device uses a **custom USB VID/PID** (not greaseweazle's), so it won't
auto-detect and `gw update` won't try to reflash it — always pass `--device`:

```bash
gw info  --device COM5
gw rpm   --device COM5
gw read  --device COM5 --format ibm.1440 disk.img
gw write --device COM5 game.dsk
```

⚠️ **Never run `gw update`** against this device (see docs/PHASE0.md).

## Hardware note
`~RDATA` (GPIO20) needs an external ~1 kΩ pull-up to 3.3 V — the board has none.
See [docs/hardware-notes.md](docs/hardware-notes.md).

## License
Ported flux/drive/protocol code derives from Adafruit_Floppy (MIT); see file
headers. The MZ-800 FDC code added later derives from a GPL v2+ source — that
license will apply to those components.
