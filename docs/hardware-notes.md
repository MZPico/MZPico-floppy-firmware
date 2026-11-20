# Hardware notes & feedback — MZPico-800-Floppy-Board

Findings from Phase 0 bring-up (see [PHASE0.md](PHASE0.md)) plus a design review
of the schematic/PCB for the next revision. Board revision reviewed: **v0.0.1**.

## Confirmed-working GPIO map (purple 16MB Pico clone, U4)
Signal names `~NAME` are active-low. The clone exposes GPIO23/24/25/29, which a
genuine Pico does not — the firmware depends on that. Full table + the shared
`C0..C7` bus / SN74LVC245 details are in `../../MZPico-800-Floppy-Board/CLAUDE.md`.

Floppy side (all direct to Pico, no buffering): DEN_RDY=13, STEP=14, DIR=15,
MOTOR_ON=16, WGATE=17, WDATA=18, HDSEL=19, **RDATA=20**, INDEX=21, TRK00=22,
WPT=23, DS0=24, DS1=25.

## What is actually on the board today (extracted from the KiCad files)

Four facts that most of the recommendations below turn on:

- **There are no resistors on the board at all.** The entire passive BOM is
  `C1`/`C2` (100 n on 3V3). No pull-ups, no series damping, no clamps, no
  termination. Both `SN74LVC245APW` (U1 address, U2 data) run at 3V3 from the
  module's pin 36.
- **`J2` and `J3` are wired in parallel on every net** — they are alternate
  footprints for *one* floppy port, not two ports. `J2` is the Shugart 5.25"
  pinout (DS on 10/12/14, MOTOR on 16, READY on 34); `J3` is the PC 3.5" pinout
  (MOTORA 10, DSB 12, DSA 14, MOTORB 16, DENSEL 2).
- **`~MOTOR_ON` is tied to `J3` pins 10 *and* 16** — one motor net for the whole
  cable. And `~DEN_RDY` is an **input** at `J2` pin 34 but an **output** at `J3`
  pin 2, on the same net: with cables in both headers, our DENSEL output fights
  a drive's READY output.
- **All 30 RP2040 GPIOs are used.** There is no spare pin on this board.

## Issues to address in the next PCB revision

Ordered by severity. Items 1–3 are electrical-safety; 4–5 are architecture.

### 1. Bare GPIOs on the floppy bus vs. terminated drives — **can destroy the board**

This supersedes the old "add 1 kΩ pull-ups to 3.3 V" note, which is only safe
because the drives tested so far happen to be lightly terminated.

The floppy bus is open-collector with the pull-ups at the *host* (a real FDC
card uses 1 kΩ to +5 V) and a terminating resistor pack in the last drive on the
cable. Old 5.25" drives — and early 3.5" — carry a **150 Ω pack to +5 V**, and
that pack terminates *all* lines, including the drive's own outputs (`~RDATA`,
`~INDEX`, `~TRK00`, `~WPT`, READY).

On this board those land on bare RP2040 pins. A 150 Ω-to-5 V pull-up on
`~RDATA` drags the pad through its ESD diode to ≈3.9 V and sinks
(5 − 3.9)/150 ≈ **7–9 mA continuously into the 3V3 rail**. That exceeds the pad
abs-max, and it back-feeds a rail that cannot sink — the 3V3 rail rises and
takes the whole board with it. Multiply by the four or five input lines. The
same path bites whenever the drive is powered and the board is not.

**Confirmed in the field (2026-08):** with the drives on their own PSU, powering
the MZ-800 off leaves the drives powered while the Pico is not. The Pico then
sits at a parasitic ~2–3 V from the drive's pull-ups via the GPIO diodes, its
power-on reset never sees a clean rise when the MZ comes back, and it wakes up
hung — no boot, **no USB enumeration**. Unplugging the floppy data cable and
powering up again clears it. Until the buffers exist: **switch the floppy PSU
on with or after the MZ-800 and off with or before it — never leave the drives
powered while the MZ is off.**

**Fix — one buffer in each direction:**

- **Drive → Pico:** a `74LVC244A`/`74LVC541A` at 3V3 for `~RDATA`, `~INDEX`,
  `~TRK00`, `~WPT`, READY (+2 spare). LVC inputs are 5.5 V tolerant and the part
  has Ioff (partial power-down), so both 5 V pull-ups and unpowered back-drive
  become non-events. For `~RDATA`/`~INDEX` prefer a **Schmitt** part —
  `SN74LVC17A` (non-inverting hex) or `74LVC14A` (inverting; fine, these are
  active-low anyway — invert in PIO). Hysteresis is worth real money on a 1.5 m
  ribbon with a slow open-collector rise, and it directly helps flux-edge jitter.
- **Pico → drive:** `74LVC07A` **open-drain** buffer for `~STEP`, `~DIR`,
  MOTORA/B, `~DS0/1`, `~HDSEL`, `~WGATE`, `~WDATA`, DENSEL. This is what
  Greaseweazle and FluxEngine do. We then behave as a real FDC — pull down only —
  and can never push 3.3 V push-pull into a line another drive is holding low.

**Current budget (the strong-pull-up question from the other side):** into a
150 Ω terminator a low costs 5 V/150 ≈ 33 mA per line — over the '07's 24 mA
rating, and over the package's 100 mA GND limit if several lines are low at
once. Three ways out, pick per target media:
1. Require 1 kΩ termination (5 mA/line — the '07 is comfortable). Document
   "1 kΩ pack, or de-terminate" as the supported drive configuration.
2. For 5.25" targets, use a classic `7407`/`7406` open-collector hex (40 mA,
   30 V) on the lines that matter — its LS input VIH of 2.0 V is happy with a
   3.3 V drive.
3. A `2N7002`/`BSS138` per line.

Default to the '07 plus option 1; add headroom only on `~WDATA`/`~STEP` if
5.25" with a factory terminator must work.

**Then move the pull-ups where they belong.** Once the inputs sit behind
5 V-tolerant buffers, fit proper **1 kΩ to +5 V** at the connector, exactly like
a real controller, instead of the current 4k7-to-3.3 V bodge on `~RDATA`.
Better noise margin over a long ribbon, and it is the level the drives were
designed against. **Do not do this while the pins are bare** — that is precisely
the hazard above.

Also: give every odd (GND) ribbon pin its own via to the plane rather than
daisy-chaining them, keep `~RDATA`'s trace to the buffer short, and put 33 Ω in
series at the source on `~WDATA`.

### 2. VSYS back-power onto the MZ-800's +5 V rail — **happens in normal use**

`J1`'s +5 V goes straight to module pin 39 (**VSYS**), and VBUS (pin 40) is
unconnected. On a Pico-style module VBUS reaches VSYS through an internal
Schottky — so plugging USB in while the cartridge is seated pushes ≈4.7 V *out*
of VSYS onto the MZ-800's +5 V rail. With the MZ off you are trying to power the
whole computer from a USB port; with it on you have two supplies in contention
through a diode. Flashing and running `mzp_probe.py` with the board in the
machine is the normal workflow, so this is being exercised routinely.

**Fix:** series Schottky — better, a P-FET ideal-diode / load switch — from
`J1` +5 V into VSYS, 10 µF bulk after it, plus a solder link to isolate MZ +5 V
for bench work. On a chip-down design (item 5), do proper power ORing of USB
VBUS and MZ +5 V into the regulator input.

While there: add ~10 µF bulk on 3V3. Two 100 n for two '245s plus the module is
thin.

### 2b. HD write faults — what was measured, what was fixed, what is open (2026-08)

An extended investigation of unreliable HD host writes under CP/M. Recorded here
because the *hardware* hypotheses it went through were all disproved by
measurement, and the next person should not repeat them.

**Signal integrity was NOT the cause.** The RDATA signal was captured during
actual failing hunts by an independent PIO pulse-width monitor (15 ns
resolution): LOW ~465 ns, HIGH ~1500 ns, period ~1965 ns — a clean 2 µs MFM
train, identical in shape to a successful bench hunt. No doubling, no glitch, no
noise. A scope confirmed a sharp falling edge with a slow (~1 µs RC) rise, and
that the drive mutes RDATA while WGATE is asserted. A 4k7→1 kΩ RDATA pull-up
swap changed nothing. A min-low-duration deglitch in the capture program
changed nothing. Both the deglitch and the pull-up theory are retired; the
original Adafruit capture program is back. (This retires the earlier advice to
fit 1 kΩ / 680 Ω on `~RDATA` as the HD fix; the board runs with the 4k7
bodge, see [PHASE0.md](PHASE0.md) item 6.) (Items 1 and 3 below remain good
hygiene for the next revision — buffered Schmitt inputs, 33 Ω on the bus,
proper grounds — but they are not the fix for anything observed.)

**Two firmware bugs were the cause of the "blind hunts"** (zero sync marks in
two revolutions of clean flux); both fixed:
1. The write splice intermittently decoded HD flux at **DD thresholds** —
   `fdc_bit_us()` reads the host's `0xDE` declaration through a cross-core path
   that can lag the write it belongs to. At DD thresholds every HD interval is
   2T and the `0xA1` sync (3T/4T) never forms. Proven by a symbol histogram
   (100 % 2T while the raw deltas held 3T and 4T). Fix: bail after ~12k
   symbols without a 3T/4T and retry at the other density.
2. `SEEK` folded a `Data − Track` delta into `head_target`; the host's Track
   register is free to disagree with our position by design, and CP/M rewrites
   it on drive switches. Fix: fold the absolute cylinder.

**Open, and where work stopped:** HD two-drive copies (A: DD → B: HD) still
fail ~1.5 % of positioning bursts to the directory track with the head
physically short, and bulk-copied files can hold another sector's content with
the directory sometimes wrong too. Established by measurement: identical rate on
two unrelated drives (ALPS DF354H090F, Mitsumi D359M3) → not the drive; every
STEP folded (`head_target` correct); firmware resync innocent (`posfix 0`
identical); bench reproduction of the same A/B alternation with seeks to 0
clean with core1 idle; Z80 bus clean (0 missed / 0 corrupt host writes on the
Track, Sector and Data ports over ~280k writes, per-port counted); all GPIO
writes on both cores atomic; bench HD writes byte-exact. Not yet split: whether
B:'s own write/read corrupts or only the A→B interplay does (a `SAVE`→run test
on B: alone, and a single-file `PIP`, decide it); the ID **head** byte was never
compared (an attempt to enforce it broke booting — MZ media does not guarantee
head byte == physical side; measure per physical side instead of enforcing);
LOST-DATA sectors (CP/M's status mask ignores that bit). The one physical
measurement not yet taken: STEP (GPIO14) and DIR (GPIO15) on a scope during
`PIP B:=A:*.*` — pulses present while the head stays put points at the
drive/cable, pulses absent or DIR glitching at the board or timing.

**Diagnostics built and then removed in the consolidation** (speculative, or a
cost on the timed paths; recoverable from pre-consolidation history): raw-delta
recorder in the live hunt (`huntflux`), PIO pulse-width monitor (`pwmon`),
sync-search histogram/best-match, per-seek ID verification with resync, in-
operation wrong-cylinder retry, per-port bus-lateness counters, drive-tagged
command history in the failed-write ring, a bench "bus aggressor". What
survived, because it is measurement rather than speculation: the DMA-fed
capture ring with absolute-counter overrun detection, `fwver`, `health`,
`firstfail`, the failed-write and track-write rings, `bustest` +
`BUSTEST.MZF`, `fdcseek`, `wrburst`, and the density/idxbit/densel controls.

### 2c. A DD track on HD media is unreadable on these drives (#41, 2026-11)

CP/M 4.1's `FORMAT4 /1440` writes cylinder 0 / head 1 at DD (16×256, so the
DD-only boot ROM can read it) on HD media. On the ALPS DF354H090F and Mitsumi
D359M3 that track reads back with CRC errors in almost every sector, so
FORMAT4 reports `Verify error 1` and FDTEST fails the boot cylinder. Both
drives select their read channel from the media's density hole and ignore
pin 2 (DENSEL), so they play the DD track back through the HD channel.

Measured from a `gw read --revs 5` capture, decoded offline with the
firmware's decoder and an independent one:

- The flux timing is clean: tight groups at 4 / 6 / 8 µs, no drift, no
  dropouts. A healthy HD track on the same disk decodes 18/18 on every
  revolution.
- About 2 % of the **4T (8 µs) intervals** come back with one or two
  spurious pulses inside them (`[97, 30, 65]` where the disk holds `[192]`
  ticks at 24 MHz), the first one near the middle. That is the HD channel
  playing back the longer DD wavelength: the readback pulse is wide enough to
  show a second peak. A 256-byte sector holds ~256 such intervals, so nearly
  every one takes a hit: 6 of 74 data fields passed their CRC over 5
  revolutions.

**Not firmware-fixable, deliberately.** The first spurious pulse sits on the
bit grid (2T into a 4T), so timing cannot tell 4T from 2T+2T, and glitch
merging (48–80-tick thresholds) changed nothing. A WD179x data separator would
accept the pulse too, so a Sharp card with this drive and disk fails the same
way. The only decoder-side "fix", CRC-guided repair of glitch clusters, would
turn a real CRC error into clean data, which is what weak-bit fidelity (#51)
forbids.

**So HD disks are data disks here, not boot disks.** Boot from a DD disk, or
use a drive that can take its density from pin 2 (some have a jumper for it).
The firmware already drives DENSEL from the effective density. Taping the HD
hole makes the boot track readable but the HD data tracks not.

### 3. MZ-800 bus side — mostly right, three additions

The bus side is already sound in principle: `SN74LVC245A` at 3V3 has 5.5 V
tolerant inputs and Ioff, so it will not back-drive the MZ when unpowered.
Missing:

- **22–33 Ω series damping** on `C0..C7` and on the `D0..D7` side. Fast edges
  into an unterminated cartridge stub, and there is a documented recurring
  marginal joint on U2's D7.
- **ESD/TVS array on the edge connector.** It is a user-inserted cartridge with
  no protection at all today.
- Output level, for the record: U2 at 3V3 drives the MZ data bus to ≈3.2 V.
  Fine for LS/HCT/Z80 (VIH 2.0 V), would fail into plain HC (VIH 3.5 V). It
  works today, so the MZ-side input is TTL-level — do not let a later revision
  "improve" that part into an HC type.
- No reset button (RUN is unconnected) and no SWD header. Add both.

### 4. Per-drive motor, and making J2/J3 two real ports

**DECIDED for the next revision: `J3` gets separate motor signals for drive A
and drive B** — pin 10 (MOTORA) and pin 16 (MOTORB) become two nets, derived
from the existing signals through glue logic rather than new GPIOs (there are
none spare). Default implementation is the zero-GPIO `74LVC2G32` option below
(motor follows select, jumper to fall back to the shared net); step up to the
one-GPIO or shift-register variant only if the respin-on-alternation cost shows
up in practice. Route the new motor outputs through item 1's open-drain stage
like every other output.

`J3` already brings out both motor pins (10 = MOTORA, 16 = MOTORB); only the net
is shared. `J2` (Shugart) has a single motor line for the whole cable, so
per-drive motor is physically impossible there. That points at the clean answer:

**Make `J2` and `J3` two independent ports.** Drive 0 = `J2` (5.25"), drive 1 =
`J3` (3.5"), each with its own `~DSn`, `~MOTOR_ON` and `~DEN_RDY`; STEP, DIR,
HDSEL, WGATE, WDATA, RDATA, INDEX, TRK00, WPT stay shared. That buys per-drive
motor, a connector per form factor, no twist cable, no shared-terminator
ambiguity, and it removes the pin-2/pin-34 direction clash noted above. Cost:
3 GPIOs over today — of which there are zero spare, hence:

**Ways to afford it on RP2040**, cheapest first:

- **Zero GPIOs, glue only.** `MOTORA# = MOTOR_ON# OR DS0#`,
  `MOTORB# = MOTOR_ON# OR DS1#` — one `74LVC2G32`. Motor follows select.
  Downside: deselect drops the spindle, so every A/B alternation costs a ~500 ms
  respin, and select must be held for the whole operation (the firmware already
  does this). Fit a jumper to fall back to the shared net. An 80 % answer for
  free.
- **One GPIO.** Keep `~MOTOR_ON` as a motor *enable* and add `MOTOR_SEL`:
  `MOTORA = EN & SEL`, `MOTORB = EN & !SEL`. Still one spindle at a time, but
  decoupled from select — no respin when `~DSn` is flipped briefly.
- **Shift register — the option that actually creates headroom.** `DS0`, `DS1`,
  MOTORA, MOTORB, `DIR`, `HDSEL`, DENSEL are all static/slow: put those 7 behind
  a `74HC595` (SER/SRCLK/RCLK = 3 GPIOs, driven from SPI or PIO) and net
  **+4 free GPIOs**. Keep `~STEP`, `~WGATE`, `~WDATA` and every input direct —
  those are timing-critical. Latch latency is ~µs, irrelevant for these lines;
  push the latch before `~STEP` and the 1 µs DIR setup is trivially met. Feed
  the '595 through the open-drain stage from item 1 anyway.
- **Not recommended:** folding `~EN0`/`~EN1` into one pin plus an inverter. It
  saves 1 GPIO but imposes a break-before-make requirement (RC-delayed Schmitt
  inverter; otherwise both '245s briefly drive `C0..C7`) on the single most
  timing-critical path — the one with ~560 ns of budget and an open `~EXWAIT`
  question.

Whatever is chosen, the firmware's shared-line handover on select stays
necessary: `~HDSEL` and the step lines remain one wire each.

### 5. Chip-down — worth it, but only as RP2350B

Chip-down RP2040 (QFN-56) or RP2350A (QFN-60) buys layout control and supply
independence but **not one extra pin** — both are 30 GPIO. **RP2350B** (QFN-80,
48 GPIO) is the variant that changes the design:

- Separate MOTORA/MOTORB, separate DENSEL out vs READY in (today one net on
  GPIO13), per-connector selects, level-shifter direction pins — no tricks
  needed, item 4 dissolves.
- **The address/data time-multiplex can be deleted entirely.** `AD0..AD7` and
  `D0..D7` are already separate pins on `J1`. Feed the address through a
  permanently-enabled buffer so PIO sees the port address continuously, and keep
  one '245 for the data direction. That removes the `~EN0`/`~EN1`
  mutual-exclusion hazard *and* most of the `~EXWAIT` budget problem in one
  move.
- 3 PIO blocks / 12 SMs instead of 2/8 — the bus SMs and the flux SM stop
  competing for space — plus 150 MHz stock clock.
- It ends the dependency on one specific AliExpress clone exposing
  GPIO23/24/25/29. That is a real supply and repeatability risk, and item 2 is a
  module artefact that disappears with chip-down.

Costs: 0.4 mm QFN-80 (fine for assembly houses, unpleasant by hand), external
QSPI flash, 12 MHz crystal, 1.1 V core buck (3.3 µH), 3V3 rail, USB-C, BOOTSEL,
reset button, SWD header; 4 layers.

Caveats: RP2350 IO is **still not 5 V tolerant** (3.63 V abs max) — item 1
applies unchanged. RP2350 erratum E9 means do not build anything on internal
pull-downs; everything here is active-low with pull-ups, so keep it that way.

If chip-down is unattractive, third-party RP2350B modules break out all 48
GPIOs — verify the exposed pin count before committing.

### 6. (Minor) DEN_RDY is a single shared net (GPIO13)
Density-select (output) and ready-sense (input) share one pin, so the
Adafruit-style API cannot use both independently — and, per the header note
above, the two connectors disagree about which direction that net is. Item 4's
per-port split fixes it. Currently harmless: the MZ-800 ROM never writes the
density port, so DENSEL is unused and density→DENSEL is closed as
"not to be done".

### 6b. Edge-connector fingers — plating and wipe — **recurring field failure**

The cartridge fingers are a live reliability problem on v0.0.1: intermittent
`FD:Loading error`s (and command bytes corrupted into shapes that impersonate
firmware bugs — the D7 incident, twice) that clear after reseating the card or
cleaning the fingers with IPA. For the next revision: **hard-gold (ENEPIG or
30µ" hard gold) plated fingers with a proper 20–30° chamfer**, maximum finger
length the slot allows (wipe), and no soldermask pullback ambiguity over the
contact area. HASL fingers oxidize and shed into the slot — they are the
cheapest part of this board and the most frequent failure. Until then:
`mzp_probe.py PORT health` screens for the symptom, and IPA plus reseat is the
field fix. Consider cleaning the MZ-800 slot springs too — the other half of
every contact.

### 7. Silkscreen / documentation
Label the headers by form factor (`J2` = 5.25" Shugart, `J3` = 3.5" PC) and
state that with v0.0.1 they are **one port** — do not populate both. Note the
bare-PC-drive convention (drive jumpered Select-B + ribbon twist); firmware
`SELECT_PIN` = DS0 (GPIO24), switching to DS1 (GPIO25) selects via the other
line on a straight cable.

Also: never power a drive from the MZ-800 expansion +5 V — a 5.25" spindle's
transients would share our ground return.
