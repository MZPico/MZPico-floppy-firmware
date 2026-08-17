# WD FDC fidelity — design & spec

Goal: emulate the MZ-800's floppy disk controller **as faithfully as possible to
the real chip**, backed by a **real physical drive**. Target the **WD179x /
FD1791-02** register+command model (the MZ-800 Disk BASIC ROM's baseline;
**WD1773** and **WD2797** are register-compatible supersets, so emulating the
179x model satisfies all three). MZ-800 media is **MFM / double density**.

Source of truth for the MZ-800 side: the Disk BASIC ROM disassembly
`MZ2Z046.ASM` (in repo root) — the low-level driver is around lines 7600–8060.

## Key principle: no cache — a real-time bridge

We do **not** decode a track into a RAM cache and serve from it. We build a
**transparent real-time bridge**: the emulated WD register interface is driven
directly by continuous MFM decode/encode of the spinning disk. This is what the
real chip does in silicon, and it's what preserves fidelity:

- **Rotational latency, byte timing, real CRC, deleted marks, odd formats** all
  fall out because we ride the real disk.
- **Weak/fuzzy-bit copy protection works for free.** Weak bits are a property of
  the *media* (ambiguous/absent flux the analog data separator resolves
  differently each revolution), **not** a WD feature. Reading the real disk each
  revolution reproduces the read-to-read variation the protection checks for
  (differs = original, identical = copy). A cache would defeat it; the real-time
  bridge preserves it. (Reproducing weak bits from an *image file* would be hard
  — but we never do that; we have the real disk.)

A cache may return later only as an *optional* read-ahead, disabled whenever
fidelity matters.

## MZ-800 FD interface — register map (from the ROM)

Ports (see `MZ2Z046.ASM` ~line 258):

| Port | Read | Write | Notes |
|------|------|-------|-------|
| `0xD8` | Status | Command | WD status/command register |
| `0xD9` | Track | Track | |
| `0xDA` | Sector | Sector | |
| `0xDB` | Data | Data | sector data streams here |
| `0xDC` | — | Motor/Drive | b7 = motor on, b1..0 = drive select |
| `0xDD` | — | Page/Side | b0 = side (see the inversion note below) |
| `0xDE` | — | Density | b0: 1 = HD, 0 = DD; drives floppy pin 2 (DENSEL) |
| `0xDF` | — | EINT | b0 = 1 arms the card's interrupt, 0 disarms |

`0xD8`–`0xDB` are the WD registers proper and are bus-inverted (see below);
`0xDC`–`0xDF` are the card's own latches and are **not** inverted.

**Side-select is inverted.** The ROM's side 0 is physical head 1. The ROM writes
`0xFF`/`0xFE` after a `CPL`, so bit0 *low* is its side 1; on top of that the
card drives HDSEL inverted relative to the PC/Shugart convention. Confirmed by
booting CP/M: every MZ-800 CP/M image carries its 16×256 IPL track on
cyl0/head1 as a PC reads it. Runtime-togglable for A/B testing via
`mzp_probe.py sideswap`.

**The `0xDE` port is decoded but deliberately not trusted** — it is latched and
reported, but never used to pick the density. The Disk BASIC ROM defines
`DE_REG = 0DEH` and **never writes it** (checked against `MZ2Z046.ASM`), so
there is no observed traffic to infer the bit's polarity from — the two
candidate readings ("b0=1 means HD" vs "b0=1 means single density") disagree.
Revisit only if a host is found that writes the port; `iotrace` will show it.

**Density (DD vs HD MFM) is instead the firmware's own determination** —
`mzp_probe.py density dd|hd|auto`. In auto (the default) each drive carries a
latched density (default DD, the machine's native format): a capture that
decodes to *nothing* at the latched bit clock is cheaply probed at the other
one (first CRC-valid ID field), and a proven ID flips the latch. The capture
window is byte-sized per density (~1.05 rev; byte windows are
rotation-relative, so they hold on a spindle that runs slow) — a DD-sized
capture of an HD disk still holds ~half an HD revolution, enough for the
probe; the first retry then captures at the flipped density's full size. A host WRITE TRACK infers density
from the format image's **sector census** — the sectors it declares and their
size codes (~5.3k on-disk for 9×512 DD vs ~10.4k for 18×512 HD) — which is
what lets a blank disk be formatted HD. Not from the image length: the host
streams bytes for as long as DRQ is up (the index ends WRITE TRACK on a real
WD), so length measures the host's patience, not its intent. Writes never auto-detect: the splice uses
the latched density, which reading (or READ ADDRESS) will have established.
DENSEL (the shared `~DEN_RDY` pin) is driven best-effort from the same model;
nothing relies on it — the target 3.5" drives sense HD from the media hole.
FM/single density remains out of scope.

## Interrupts — DRQ-driven, not INTRQ-driven

This card does **not** follow the WD "INTRQ at command completion" model. `~INT`
(GPIO12) is driven from **DRQ**, gated by the EINT latch on `0xDF`:

- Asserted while a data byte is pending during a transfer command.
- Cleared by the host **reading DATA** (`0xDB`) — *not* by polling STATUS.
  Releasing on a STATUS read would drop the interrupt before the ISR ever
  reached the transfer.
- Also released by a new command write, or by EINT going to 0.
- Type IV Force Interrupt with **I3** (`0xD8`) latches a separate one-shot
  INTRQ, cleared by a STATUS read or the next command — the one place the WD
  rule does apply.

The EINT gate is what makes this safe. `~INT` goes straight to expansion pin 38
with no buffer, and the Z80's INT input is **level** sensitive: asserting it for
a host with no floppy ISR re-enters `0x0038` forever and hangs the machine. The
boot ROM and CP/M 2.3 never write `0xDF`, so they are never interrupted; CP/M
4.1 writes `0x01` before each data command and `0x00` after, and boots.

`~INT` is driven **open-drain** (pull low to assert, Hi-Z to release) so we never
fight another device on the shared INT line. Bus RESET (GPIO29, **active high**)
disarms EINT and releases the line — EINT is a latch in our state that a
resetting host cannot clear, so without that a reset during an interrupt brought
the machine back up into a permanent one.

## Inverted data bus — mandatory

The driver `CPL`s **every** byte read from or written to the FDC ("Data are
complemented", e.g. lines 7774, 7889, 7998). The FDC data bus is wired inverted
to the Z80. So our emulated FDC must:

- **Read (MZ reads a register):** place `~value` (bit-complement of the true WD
  register value) on the data bus.
- **Write (MZ writes a register):** the byte we receive is `~value`; complement
  it to recover the true command/data.

(Separately, the ROM does an *additional* content complement only in MZ-80K
mode — that's application-level, not our concern.)

## Data transfer handshake — software DRQ polling

The MZ driver does **not** rely on wait states to pace data. It **polls STATUS**
and transfers one byte (`INI`/`OUTI`, self-modified opcode at `D39DD`) only when
**DRQ** is set, ending the sector when **BUSY** clears. The loop (`A39D3`,
status read is inverted; `E=0x7C`, `D=0xFE`):

```
A39D3: IN A,(ST_REG)      ; A = ~status
       OR  E              ; force bits 2..6 = 1
       CP  D              ; == 0xFE  <=>  BUSY=1 & DRQ=0 (ready, no byte)
       JR  Z,A39D3        ; keep polling while busy & not ready
       CP  0xFC           ; == 0xFC  <=>  BUSY=1 & DRQ=1 (byte ready)
       JR  NZ,A39E2       ; else BUSY=0 -> sector done
D39DD: INI / OUTI         ; transfer exactly one byte
       JP  NZ,A39D3       ; until byte count exhausted
```

Implications for our emulation:
- Maintain **accurate STATUS** (esp. BUSY bit0, DRQ bit1) in real time; the CPU
  self-paces off it. No per-byte `~EXWAIT` data pacing needed.
- With real-time decode, DRQ goes 0→1 at the disk byte rate (~32 µs at DD); the
  CPU poll loop (~11 µs) spins a few times per byte → **authentic pacing for
  free**, nothing to synthesize.
- `~EXWAIT` is used **only** to hold the Z80 long enough for us to put a valid
  byte on the bus during a register *access* (µs-scale) — never for the whole
  seek/read latency. This is why long physical latency does **not** stall the
  Z80 (avoids the DRAM-refresh hazard): during a seek or rotational wait, BUSY=1
  and the CPU polls STATUS (fast accesses), it is not held.

## Status register bits (true values, WD179x)

Type I (RESTORE/SEEK/STEP): b7 NotReady · b6 WriteProtect · b5 HeadLoaded ·
b4 SeekError · b3 CRCError · b2 Track00 · b1 Index · b0 Busy.

Type II/III (READ/WRITE): b7 NotReady · b6 WriteProtect(write) · b5 RecordType/
WriteFault · b4 RecordNotFound · b3 CRCError · b2 LostData · b1 DRQ · b0 Busy.

(The ROM checks mask `10011001b` = NotReady/RNF/CRC/Busy after transfers.)

## Commands to implement

- **Type I:** RESTORE, SEEK, STEP, STEP-IN, STEP-OUT — drive the real STEP/DIR,
  read real TRK00/INDEX. (Phase-1 drive code already does the mechanics.)
- **Type II:** READ SECTOR, WRITE SECTOR — single **and multiple** (the ROM uses
  Read Multiple `0x90` / Write Multiple `0xB4`).
- **Type III:** READ ADDRESS, READ TRACK, WRITE TRACK (format).
- **Type IV:** FORCE INTERRUPT `0xD0` (the ROM uses it to terminate multi-sector
  ops).
- Correct **INTRQ** (→ MZ-800 `~INT`) and **DRQ**/**BUSY** sequencing.

## Architecture — mirrors the chip's host/disk split

- **Core 1 — host/register side:** bus PIO decodes IORQ/RD/WR to `0xD8`–`0xDE`;
  WD register state machine (COMMAND/STATUS/TRACK/SECTOR/DATA); computes STATUS;
  drives DRQ/BUSY, `~INT` (INTRQ), `~EXWAIT` (bus timing only); handles the
  inverted bus. Always responsive.
- **Core 0 — disk side:** continuous flux capture + **streaming MFM data
  separator** that always knows the current rotational position; on command,
  locate the sector by ID, stream data bytes (+ CRC) into an inter-core FIFO for
  reads; for writes, read one rev to locate then **write-splice** the data field
  on the next rev (WGATE over just that sector — the ~22-byte Gap 2 gives
  ~700 µs of slack, so timing is forgiving); drives STEP/DIR/WGATE.
- **Shared:** command/status flags + a small byte FIFO between cores.

PIO/SM budget: bus_io SMs + one flux SM across pio0/pio1. Flux capture disables
IRQ only on core0, so core1 keeps servicing the MZ-800 during a read.

## Scope / limitations

- **MFM double-density** only initially; add FM/single-density only if a disk
  needs it.
- **Write precompensation** skipped initially (real drive tolerates read-back);
  add if inner-track writes prove marginal.
- **Weak bits**: reproduced naturally via the real disk (no special code); only
  perfectly-exact analog reproduction is out of scope.

## Open items

- ~~Decode `0xDC`–`0xDE` bit assignments~~ — ✅ done, see the register map above.
  `0xDE` is decoded but not yet applied to the DENSEL pin (deferred: MZ-800
  media is DD).
- Confirm whether any bus wait is inserted on register access by the FD-I/F, and
  the exact `~EXWAIT` timing budget for putting a byte on the shared C0..C7 bus.
- ~~Write-back for WRITE SECTOR: in-place splice (chosen) vs. any fallback.~~ —
  ✅ in-place splice, implemented in [src/mfm_splice.cpp](../src/mfm_splice.cpp);
  see Milestone 3.3 in [phase3-plan.md](phase3-plan.md) for the mechanism and the
  two places this deviates from a real WD (the sector is buffered from the host
  before any flux is committed, and the sector length comes from a per-cylinder
  cache rather than from the ID field the write has not found yet). Verified on
  hardware: CP/M 4.1 writes files to a physical disk and reads them back
  correct.
