# MZPico Floppy Firmware

Firmware for the **MZPico-800-Floppy-Board**, a Raspberry Pi Pico (RP2040)
cartridge that lets a **Sharp MZ-800** use a **real, physical floppy drive**.

The Pico sits on the MZ-800 expansion bus and emulates the machine's WD179x
floppy controller at ports D8h–DFh. Behind it, an ordinary 34-pin Shugart/PC
drive is driven at the flux level. Sectors are read and written on the real
disk in real time, with no image file and no cache. Rotational latency, CRC
errors and weak bits all come from the disk under the head.

```
MZ-800  <== expansion bus ==>  RP2040  <== 34-pin cable ==>  floppy drive(s)
           WD179x registers           MFM flux capture/encode
```

## Status

| | |
|---|---|
| Boot from the MZ-800 ROM, CP/M 4.1 | works, DD |
| Read, write, format (FORMAT4), two drives | works, DD |
| HD (500 kbit/s) | reads, formats and writes work as data disks. Two-drive use still throws an occasional wrong-cylinder read, which is rejected and retried ([#31](https://github.com/MZPico/MZPico-floppy-firmware/issues/31)): don't host-write HD disks you care about. HD disks can't be boot disks on drives that sense density from the media hole ([hardware notes §2c](docs/hardware-notes.md)) |
| Greaseweazle-compatible flux access over USB | works (`gw`, fluxengine) |

The design, the hard-won rules and the detailed status are in
[CLAUDE.md](CLAUDE.md). The controller's behaviour is specified in
[docs/wd-fidelity.md](docs/wd-fidelity.md). Releases (uf2 + test programs) are
on the [releases page](https://github.com/MZPico/MZPico-floppy-firmware/releases).

## Hardware

- **Board:** MZPico-800-Floppy-Board with the purple RP2040 clone (16 MB
  flash). The firmware uses GPIO23/24/25/29, which a genuine Pico does not
  bring out.
- **Drives:** one or two 3.5"/5.25" PC drives on `~DS0`/`~DS1`, with their own
  +5 V/+12 V supply.
- **`~RDATA` pull-up:** GPIO20 needs an external pull-up to 3.3 V. The current
  board has a 4k7 bodge; trying a stronger 1 kΩ changed nothing
  ([docs/hardware-notes.md](docs/hardware-notes.md)).

**Two hazards on the current board** (details and fixes in
[docs/hardware-notes.md](docs/hardware-notes.md)):

- The drive's outputs land on bare RP2040 pins. **Switch the drive supply on
  with or after the MZ-800, and off with or before it.** Drives powered while
  the Pico is not back-feed it; it then wakes up hung, with no USB. Heavily
  terminated (150 Ω) drives can damage the board.
- USB VBUS reaches the MZ-800's +5 V through the module's VSYS. Plugging USB
  in with the cartridge seated is the normal workflow and works, but it puts
  the two supplies in contention through a diode.

## Flash

Download `mzpico_floppy-<release>.uf2` from the releases page, or build it
(below). Hold **BOOTSEL**, plug in USB (the board mounts as `RPI-RP2`), and
copy the uf2 onto it. Then check which build is running:

```bash
pip install pyserial
python tools/mzp_probe.py COM6 health      # or /dev/ttyACM0
```

`health` reports the running firmware's git hash and screens the bus
counters. **Run it first in every session**: random `FD:Loading error`s that
reseating the cartridge cures are edge-connector contacts, not firmware.

## Use

Plug the cartridge into the MZ-800 and connect the drive(s). The MZ-800's own
ROM finds the controller: boot a master disk (Disk BASIC or CP/M) as you would
with the original Sharp floppy card.

Two test programs for the machine are attached to every release:

- **`FDTEST.MZF`** tests the controller on the MZ-800 in about a minute.
  - Option `1` is read-only and works on any MZ-800 disk.
  - Option `3` also writes and formats, so use a **scratch disk** for it.
  - See [mzf/fdtest/README.md](mzf/fdtest/README.md).
- **`BUSTEST.MZF`** is a bus loopback test. Run `mzp_probe.py PORT
  bustestreset`, then run it from the monitor, then run `mzp_probe.py PORT
  bustest`.

`tools/mzp_probe.py` is the bench and diagnostic tool; run it without
arguments for the command list. The useful commands:

| Command | Shows |
|---|---|
| `health` | Which firmware is running; bus anomalies |
| `rddiag` | The last read: capture, decode, the position chain |
| `wrdiag` | The last write: what was written, or why it was refused |
| `firstfail` | The first failure since power-up, held unaltered |
| `scan <track> <side>` | Every sector on a track, with CRCs |
| `drive 0` / `drive 1` | Which drive the bench commands use |

Bench commands that move or read the drive are refused while the MZ-800 has a
floppy command in flight. Commands marked `!!` in the usage text write to the
disk.

## Greaseweazle

The firmware also speaks the [greaseweazle](https://github.com/keirf/greaseweazle)
protocol, so `gw` and [fluxengine](https://cowlark.com/fluxengine/) can image
disks through the board. It uses its own USB VID/PID, so always pass
`--device`:

```bash
gw rpm   --device COM6
gw read  --device COM6 --format ibm.1440 disk.img
```

⚠️ **Never run `gw update`** against this device ([docs/PHASE0.md](docs/PHASE0.md)).

## Build

You need CMake, Ninja or Make, and `gcc-arm-none-eabi` (CI uses Ubuntu
24.04's 13.2). The Pico SDK is the git submodule `external/pico-sdk`, pinned
to **2.2.0**.

```bash
git submodule update --init external/pico-sdk
git -C external/pico-sdk submodule update --init lib/tinyusb   # the only inner one needed
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build                 # -> build/mzpico_floppy.uf2 (~180 KB)
```

The firmware reports the git hash it was built from (`health`, `fwver`). A
build from a tree with uncommitted changes says `+dirty`.

## Tests

```bash
cmake -S tests -B build-tests -G Ninja && cmake --build build-tests
ctest --test-dir build-tests -j"$(nproc)" --output-on-failure
```

The host tests compile the controller and drive code **unmodified** against a
simulated drive and an emulated MZ-800 that runs `FDTEST.MZF`. The test
program needs z88dk, either as the snap or through the Docker image that CI
uses. See [tests/README.md](tests/README.md).

CI builds the firmware and runs these tests on every push and PR. Pushing an
annotated `fw-*` tag publishes a release.

## Layout

| Path | What it holds |
|---|---|
| `src/bus/` | MZ-800 bus (PIO, core1 listen loop) and the WD179x controller (`fdc.cpp`) |
| `src/mfm_*.cpp` | MFM decode, WRITE TRACK encoding, the in-place sector write splice |
| `src/flux/` | RP2040 PIO flux capture/write (from Adafruit_Floppy) |
| `src/floppy.*` | Drive control |
| `src/gw_protocol.*` | Greaseweazle and probe commands over USB |
| `tests/` | Host simulation and test suites |
| `mzf/` | MZ-800 test programs (z88dk) |
| `tools/` | `mzp_probe.py`, CI scripts |
| `docs/` | Design notes and hardware findings |

The Sharp ROMs, their disassembly listings and real disk images used during
development are copyrighted and are **not** in this repository.

## License

Apache License 2.0 ([LICENSE](LICENSE)). The flux, drive and greaseweazle
protocol code is ported from Adafruit_Floppy and keeps its MIT license; the
test harness vendors a MIT Z80 core. Attributions and license texts:
[NOTICE](NOTICE), [LICENSES/](LICENSES/).
