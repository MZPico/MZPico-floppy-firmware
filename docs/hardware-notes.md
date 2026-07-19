# Hardware notes & feedback — MZPico-800-Floppy-Board

Findings from Phase 0 bring-up (see PHASE0.md). Board revision at time of
testing: **v0.0.1**.

## Confirmed-working GPIO map (purple 16MB Pico clone, U4)
Signal names `~NAME` are active-low. The clone exposes GPIO23/24/25/29, which a
genuine Pico does not — the firmware depends on that. Full table + the shared
`C0..C7` bus / SN74LVC245 details are in `../../MZPico-800-Floppy-Board/CLAUDE.md`.

Floppy side (all direct to Pico, no buffering): DEN_RDY=13, STEP=14, DIR=15,
MOTOR_ON=16, WGATE=17, WDATA=18, HDSEL=19, **RDATA=20**, INDEX=21, TRK00=22,
WPT=23, DS0=24, DS1=25.

## Issues to address in the next PCB revision

### 1. Missing pull-ups on drive-output (input) lines — **important**
The floppy status lines have **no pull-up resistors** on the board; they rely on
the RP2040's weak internal pull-ups.
- **`~RDATA` (GPIO20) is the blocker:** the Adafruit library leaves the read pin
  as bare `INPUT` (no internal pull-up), so with no board pull-up the line
  floats and flux reads fail entirely. **Required fix:** add a **1 kΩ pull-up to
  3.3 V** on `~RDATA`. (Phase 0 was unblocked by bodging exactly this.)
- Recommended: also add **~1 kΩ pull-ups to 3.3 V** on `~INDEX`, `~TRK00`,
  `~WPT`, `~DEN_RDY`. They currently work on internal pull-ups but that is
  marginal (slow signals only, noise-prone).
- **3.3 V only** — RP2040 GPIO is not 5 V tolerant. Do not pull to VBUS/5 V.

### 2. Two floppy headers J2/J3 have different select/motor pinouts — document it
- **J3** matches the standard PC 3.5" pinout (DS0→pin14/Drive-Select-A, MOTOR on
  pins 10 & 16). Use J3 for a normal PC drive.
- **J2** differs (DS0 on pin10, MOTOR on pin16, pin14 n/c) — a standard PC drive
  on J2 spins but is never selected. Intended for a different drive/cable type.
- Action: silkscreen/label which header is for which drive type, or standardize.
- Note the usual bare-PC-drive convention: drive jumpered Select-B + ribbon
  twist. Firmware `SELECT_PIN` = DS0 (GPIO24); switching to DS1 (GPIO25) selects
  via the other line if a straight cable is used.

### 3. (Minor) DEN_RDY is a single shared net (GPIO13)
Density-select (output) and ready-sense (input) share one pin, so the
Adafruit-style API can't use both independently. Fine for our use (we drive
density, ignore ready), but note it for the real firmware.
