# Phase 0 — Floppy hardware bring-up (over USB, no MZ-800)

Goal: prove the MZPico-800-Floppy-Board + purple Pico can read a real floppy
drive at flux level, controlled from a PC over USB — before any MZ-800 bus work.
This uses the Adafruit_Floppy **greaseweazle** Arduino sketch as throwaway
firmware and the greaseweazle host tool (`gw`).

**Status: PASSED** — `gw rpm` reports ~298 RPM; drive select, motor, seek,
index, and flux read over RDATA all verified.

## Toolchain
- **Arduino IDE** + Earle Philhower **arduino-pico** core
  (Boards Manager URL: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`).
- **Adafruit Floppy** library **0.6.1** (+ deps: Adafruit BusIO, SdFat–Adafruit
  Fork, Adafruit ST7735/ST7789, Adafruit NeoPixel).
- **greaseweazle host tools** — `gw` (1.23 works fine; see gotcha below).

## Firmware: greaseweazle example with this board's pins
Open **File → Examples → Adafruit Floppy → greaseweazle**. Add this block before
the `#else // #error` in the pin section (activates when Board = "Raspberry Pi
Pico", which defines `ARDUINO_RASPBERRY_PI_PICO`):

```c
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
// MZPico-800-Floppy-Board (purple 16MB clone)
#define DENSITY_PIN 13 // DEN_RDY   (GPIO13)
#define INDEX_PIN   21 // ~INDEX    (GPIO21)
#define SELECT_PIN  24 // ~DS0      (GPIO24)   // see drive-select note
#define MOTOR_PIN   16 // ~MOTOR_ON (GPIO16)
#define DIR_PIN     15 // ~DIR      (GPIO15)
#define STEP_PIN    14 // ~STEP     (GPIO14)
#define WRDATA_PIN  18 // ~WDATA    (GPIO18)
#define WRGATE_PIN  17 // ~WGATE    (GPIO17)
#define TRK0_PIN    22 // ~TRK00    (GPIO22)
#define PROT_PIN    23 // ~WPT      (GPIO23)
#define READ_PIN    20 // ~RDATA    (GPIO20)   // NEEDS EXTERNAL PULL-UP, see below
#define SIDE_PIN    19 // ~HDSEL    (GPIO19)
#define READY_PIN   25 // dummy (DEN_RDY shared w/ DENSITY; ready-sense unused)
```

Arduino **Tools** settings (all required):
- Board: **Raspberry Pi Pico** (not Generic RP2040 / not Pico W — different macros)
- USB Stack: **Adafruit TinyUSB** (sketch `#error`s otherwise)
- CPU Speed: **200 MHz**, Optimize: **-O3 / Fastest**

Flash: hold **BOOTSEL** while plugging USB (mounts `RPI-RP2`), Upload, confirm
"Done uploading".

## Host commands (Windows; keep the drive on a twisted cable — see below)
```powershell
gw info --device COM5                              # identify device
gw seek --device COM5 0                            # step to track 0 (tests STEP/DIR/TRK00)
gw rpm  --device COM5                              # ~300 rpm (tests select/motor/index/flux)
gw read  --device COM5 --format ibm.1440 disk.img  # decode a PC 1.44MB disk (READ path)
gw read  --device COM5 disk.scp                    # or raw flux (any disk)
gw write --device COM5 --format ibm.1440 disk.img  # write HD image (WRITE path; auto-verifies)
gw write --device COM5 game.dsk                    # write an MZ-800 EDSK (.dsk) — auto-detected
```
`gw write` reads each track back and verifies it, so a clean write = read+write both proven.

## Gotchas we hit (in order) — READ THIS
1. **`pip install greaseweazle` fails** ("from versions: none"). There is no
   PyPI package. On Windows download the release zip and run `gw.exe`; or
   `pipx install git+https://github.com/keirf/greaseweazle@latest`.
2. **Serial-port exclusivity.** If the Arduino **Serial Monitor** (or any
   terminal) has the COM port open, `gw` reports **"Device: Not found"** even
   with `--device`. Close everything on the port before running `gw`. This cost
   us the most time — the device was fine all along.
3. **NEVER run `gw update`.** It offers to flash Keir's official greaseweazle
   firmware, which would overwrite the Adafruit firmware this board needs.
   Ignore the "New firmware available" message permanently. Device correctly
   reports `Model: Adafruit Floppy Generic, Firmware: 1.0`.
4. **Host-tools version is NOT the issue.** gw 1.23 works with the Adafruit v1.0
   firmware. (We briefly chased a downgrade to v0.34 — unnecessary.)
5. **Drive select / cable twist.** A bare PC 3.5" drive is usually jumpered as
   Drive-Select-B and relies on the ribbon **twist**. Symptom of not-selected:
   motor spins but `gw seek` says "Track 0 not found" and the head clicks
   against the stop (no status outputs). Fix: use a **twisted PC cable** with
   the drive on the **post-twist end connector**. (J3 is the standard-PC header;
   J2 has a different select/motor pinout — see hardware-notes.md.)
6. **RDATA needs an external pull-up.** The PCB has **no pull-up** on `~RDATA`
   and the library leaves the read pin as bare `INPUT` (no internal pull-up).
   Result: `gw rpm`/`gw read` hang (flux capture over a floating line) while
   `seek` works. **The bodge fitted is 4k7 from GPIO20 (Pico pin 23) to +3.3 V
   (Pico pin 36 / VCC)** (per the Adafruit suggestion). Use 3.3 V, never 5 V
   (GPIO not 5 V tolerant).
   - **Is 4k7 strong enough for HD?** RDATA is open-collector from the drive,
     so the pull-up alone sets the rise time: 4k7 into ~1.5 m of ribbon
     (~150 pF) is ~0.7 µs, which looked marginal against HD's 1 µs bit cell
     and was for a while suspected as the cause of the HD write-splice
     failures. **Measurement retired that theory:** a 4k7→1 kΩ swap changed
     nothing, a PIO pulse-width monitor showed a clean RDATA train during
     failing hunts, and the "blind hunts" were two firmware bugs (see
     [hardware-notes.md](hardware-notes.md) §2b). The 4k7 bodge stays; a
     proper 1 kΩ-to-+5 V termination behind 5 V-tolerant buffers is the plan
     for the next board revision (hardware-notes.md §1).
7. **Writing DD images needs DD media.** `gw write` verifies each track by
   reading it back; writing a **DD** image (e.g. an MZ-800 720K EDSK) onto **HD**
   media fails verify (HD media magnetizes too weakly at DD current). Options:
   use real **720K DD** disks, or (interim, unreliable) **tape over the HD-
   detection hole** of an HD disk to coerce DD write current — this worked for
   testing. To isolate a suspected write-path fault from media, first write a
   *matched* HD image (`gw write --format ibm.1440 disk.img`) to an HD disk.
8. **`.dsk` = Extended DSK (EDSK).** The reference firmware's images (and the
   ones we tested) are standard **EDSK** ("EXTENDED CPC DSK File" magic). gw
   auto-detects and writes them directly — no conversion. (Geometry was
   inspected with a local helper script, `dsk_analyze.py`, which is not part of
   this repository.)

## Result
```
gw rpm --device COM5
Rate: 298.232 rpm ; Period: 201.186 ms

gw read --device COM5 --format ibm.1440 disk.img
... all tracks: IBM MFM (18/18 sectors) ...
Found 2880 sectors of 2880 (100%)
```
Full 1.44MB PC disk read error-free (80 cyl × 2 heads × 18 sectors). Flux
capture + MFM decode + sector recovery all proven end to end.

**Write also validated:** HD image written+verified to HD media; and a real
MZ-800 **DD CP/M EDSK** written+verified onto an HD disk with the HD-detection
hole taped. Entire physical floppy interface (read + write, DD + HD) confirmed.

Next: Phase 1 (own Pico-SDK USB flux firmware). See ../CLAUDE.md for the plan.
