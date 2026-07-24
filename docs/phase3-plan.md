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
- **3.2 — READ SECTOR (single + multiple) — the payoff.** On command: BUSY set,
  core0 streaming-decodes the live flux, finds the sector by ID, streams data to
  DATA with DRQ; core1 serves the software-polled transfer; INTRQ at end; CRC/RNF
  status. Multi-sector + Force Interrupt. Test: **MZ-800 reads a directory / boots
  CP/M from the physical disk.**
- **3.3 — WRITE SECTOR (in-place splice).** Locate the sector by ID (streaming
  decode), then WGATE + write the MFM-encoded data field in the ~22-byte Gap 2
  window (~700 µs slack). Test: MZ-800 writes a file; read back (host `gw`/scan).
- **3.4 — Type III / format / density / multi-drive.** READ ADDRESS, READ TRACK,
  WRITE TRACK (format), density via `~DEN_RDY`, drive-select via `~DS0/DS1`,
  decode `0xDC`–`0xDE` control bits.
- **3.5 — Robustness.** Error/retry paths, write precomp if inner tracks are
  marginal, edge-case timing, soak testing.

## Primary risks / unknowns

1. **Bus real-time response** (M3.0): detecting `~IORQ` + address and asserting
   `~EXWAIT` fast enough, with the shared-bus buffer switching. Mitigated by
   adapting the proven reference DELUXE PIO/loop. **Biggest risk.**
2. **No `~M1` pin on this board.** The reference uses M1 to distinguish I/O from
   opcode-fetch/interrupt-acknowledge. We decode on `~IORQ`+address only →
   theoretical risk of responding during an IORQ-based INTA cycle. Assess against
   the MZ-800's interrupt scheme; likely benign but confirm.
3. **Streaming decode keeping bit-sync in real time** (M3.2): the disk engine must
   maintain the data separator continuously off the live FIFO. New code, but built
   on Adafruit's `mfm_io_sample_flux` primitive + `mfm_impl` logic.
4. **`~EXWAIT` budget** to put a byte on C0..C7 within the Z80 cycle.
5. **`0xDC`–`0xDE` control-bit meanings** (motor/drive-select/side/density) —
   decode from `MZ2Z046.ASM` before M3.1/M3.4.

## Immediate next step

**Milestone 3.2 — READ SECTOR (single + multiple).** M3.1 is done; the ROM now
issues `READ-SEC(M)` (0x94) and errors ("FD Loading error") because Type II isn't
implemented. Next: on a Read Sector command, core0 streaming-decodes the live
flux (seek is already in place), finds the sector by ID (Track/Sector registers,
side), and streams the data field to the DATA register with **DRQ** as the ROM
polls STATUS; set BUSY during, RNF/CRC on error, and support multi-sector
terminated by Force Interrupt (0xD0). Reuse the Phase-2 streaming MFM decoder.
Test: **MZ-800 reads a directory / boots CP/M from the physical disk.** Keep the
same discipline: one testable milestone at a time.
