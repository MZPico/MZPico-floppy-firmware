# WD FDC fidelity — design & spec

Goal: emulate the MZ-800's floppy disk controller **as faithfully as possible to
the real chip**, backed by a **real physical drive**. Target the **WD179x /
FD1791-02** register+command model (the MZ-800 Disk BASIC ROM's baseline;
**WD1773** and **WD2797** are register-compatible supersets, so emulating the
179x model satisfies all three). MZ-800 media is **MFM**, double density;
high density (500 kbit/s) is supported as well (see Density).

This is the living spec: it describes what the firmware does now. The code is
the final authority — `src/bus/fdc.cpp` (register side and disk task),
`src/mfm_decode.cpp`, `src/mfm_splice.cpp`, `src/mfm_encode.cpp`,
`src/flux/arch_rp2.cpp`, `src/bus/bus.pio` / `src/bus/mz_bus.cpp`. The
milestone history is in [phase3-plan.md](phase3-plan.md).

Source of truth for the MZ-800 side: the Disk BASIC ROM disassembly
`MZ2Z046.ASM` — the low-level driver is around lines 7600–8060. That listing
(and the boot-ROM files `9Z_504M.ROM` / `9Z_504M.dis.asm`) is copyrighted
third-party material: it is kept locally, git-ignored, and **not distributed
with this repository**. Line references below are to that local copy.

## Key principle: no cache — a real-time bridge

We do **not** decode a track into a RAM cache and serve later commands from it.
Every READ command captures the spinning disk afresh, and every write rides the
disk live. This is what preserves fidelity:

- **Rotational latency, real CRC, deleted marks, odd formats** all fall out
  because each command reads the real disk.
- **Weak/fuzzy-bit copy protection is preserved.** Weak bits are a property of
  the *media* (ambiguous/absent flux the data separator resolves differently
  each revolution), **not** a WD feature. Because each READ command takes a new
  capture, two reads of a weak sector see two different revolutions and can
  return different bytes and CRC results — the read-to-read variation the
  protection checks for. A data CRC error is reported to the host, never
  retried internally (see the read path), so the firmware does not mask it
  either. A cache would defeat this.

What *is* reused: within one READ MULTIPLE, the remaining sectors of the track
come out of the capture that command is already running (that is still one
pass of the real disk, exactly what a WD sees).

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
| `0xDE` | — | Density | b0: 1 = HD, 0 = DD (see Density) |
| `0xDF` | — | EINT | b0 = 1 arms the card's interrupt, 0 disarms |

`0xD8`–`0xDB` are the WD registers proper and are bus-inverted (see below);
`0xDC`–`0xDF` are the card's own latches and are **not** inverted.

**Side-select is inverted.** The ROM's side 0 is physical head 1. The ROM writes
`0xFF`/`0xFE` after a `CPL`, so bit0 *low* is its side 1; on top of that the
card drives HDSEL inverted relative to the PC/Shugart convention. Confirmed by
booting CP/M: every MZ-800 CP/M image carries its 16×256 IPL track on
cyl0/head1 as a PC reads it. Runtime-togglable for A/B testing via
`mzp_probe.py sideswap`.

**Drives.** Port `0xDC`'s drive field selects unit 0 (`~DS0`) or unit 1
(`~DS1`); units 2/3 have no select line on this board and report NOT READY.
Each drive has its own head position, head target, density latch, revolution
period and sector-size cache. `~HDSEL` and `~MOTOR_ON` are single wires shared
by both drives (the motor line reaches every drive on the cable, so it is not
gated by select).

## Density (DD / HD)

**The `0xDE` port is the host's density declaration, and its polarity is
OBSERVED: b0 = 1 means HD.** The Disk BASIC ROM never writes the port (checked
against `MZ2Z046.ASM`), which long made the polarity unknowable — but CP/M 4.1
writes it routinely: `0x00` throughout ordinary DD work, then `0x80`/`0x81` at
the moment FORMAT4 switches to an HD format (`iotrace`, 2026-08). A valid
declaration is cleared by bus RESET, so the boot ROM starts from the per-drive
latch again.

Modes (`mzp_probe.py density dd|hd|auto|detect`):

- **`auto` (default):** a valid host `0xDE` declaration decides. With no
  declaration (the boot ROM), the per-drive latch decides — default DD, the
  machine's native format. **No media probing** in this mode; the boot/DD path
  behaves exactly as it did before HD support.
- **`detect`:** `auto`, plus media probing for hosts that never write `0xDE`.
  A capture that decodes to *nothing* at the latched bit clock — no CRC-valid
  ID at all, not even a wrong-cylinder one — is re-decoded at the other clock
  (first CRC-valid ID field, `mfm_read_address`); a proven ID flips the latch.
  No recapture is needed: the read window is sized in time, so it holds a full
  revolution at either density. Never used while a host declaration exists.
- **`dd` / `hd`:** forced, bench only.

Writes never auto-detect from media. The write splice uses the effective
density; if its sync hunt sees flux but no 3T/4T symbols at all (the signature
of decoding HD flux at DD thresholds), it bails and retries once at the other
density, adopting whichever works.

A host WRITE TRACK under a valid `0xDE` declaration follows it unconditionally.
The collection tail (we clock in more bytes than a real WD, whose DRQ stops at
the index) is host memory junk that can carry phantom IDAMs; it once inflated a
16×256 DD boot image past the HD threshold and got the boot track encoded at
the wrong clock. Only with **no** declaration does the image's **sector
census** — declared sectors × size codes (~5.3k on-disk bytes for 9×512 DD vs
~10.4k for 18×512 HD), never the image length — pick the density. Layout fact
this exposed: **an MZ-800 HD disk carries a DD 16×256 boot track on cyl 0 /
head 1**; FORMAT4 declares DD for exactly that one track so the DD-only boot
ROM can boot the disk.

**DENSEL** (floppy pin 2, on the shared `~DEN_RDY` net, GPIO13) is driven
best-effort from the effective density (`densel` overrides it on the bench).
Nothing relies on it: the target 3.5" drives (ALPS DF354H090F, Mitsumi D359M3)
sense HD from the media hole and ignore pin 2.

**Type I Index bit (b1)** is live from `~INDEX` only while the host has
declared HD (`idxbit`, default mode 1): FORMAT4's HD path times the spindle
with it, and the DD boot ROM must not see it (an earlier always-live
experiment broke the ROM's RESTORE completion check).

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
fight another device on the shared INT line. Bus RESET (GPIO29, **active high**,
taken as an edge interrupt on core1 — never polled from the listen loop)
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
- Maintain **accurate STATUS** (esp. BUSY bit0, DRQ bit1); the CPU self-paces
  off it. No per-byte `~EXWAIT` data pacing.
- **DRQ is paced by the host, not by the disk.** The firmware buffers a whole
  sector before raising DRQ (reads: decoded and CRC-checked; writes: collected
  before any flux is committed). Once DRQ is up it stays up for every byte
  until the buffer is drained or filled, so the transfer runs at the host's
  own loop speed, not at the disk byte rate (~32 µs/byte DD, ~16 µs HD) a real
  WD imposes. What *is* real is the latency to the first DRQ of a read:
  rotational latency to the sector, plus its pass under the head, plus a few
  ms of decode. A host whose correctness depended on per-byte disk timing
  (none known on the MZ-800) would see a difference here.
- LOST DATA is ours to declare, since we can wait: a READ whose first byte is
  not taken within 15 ms is treated as a status-only verifier (FORMAT4 checks
  CRCs that way) and completes with LOST DATA; a mid-transfer stall longer than
  250 ms raises LOST DATA in either direction. A WRITE that stalls is aborted
  with nothing written.
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

## Commands

- **Type I:** RESTORE, SEEK, STEP, STEP-IN, STEP-OUT drive the real STEP/DIR
  and read the real TRK00/INDEX. core1 folds each command into a **clamped
  absolute** `head_target` per drive (SEEK folds the absolute `Data` cylinder,
  not a `Data − Track` delta; surplus STEP-OUTs past TRK00 are absorbed per
  command); core0 drives the head there. No queued Type I is dropped, and no
  Type II capture or splice starts while a move is still pending. The head
  position and the host's Track register are free to disagree. Verify (V bit)
  passes only on a CRC-valid ID whose cylinder equals the Track register.
- **Type II:** READ SECTOR, WRITE SECTOR — single **and multiple** (the ROM uses
  Read Multiple `0x90` / Write Multiple `0xB4`).
- **Type III:** READ ADDRESS, READ TRACK, WRITE TRACK (format).
- **Type IV:** FORCE INTERRUPT `0xD0` (the ROM uses it to terminate
  multi-sector ops).

### ID track compare (all Type II, Verify, and the write splice)

A sector matches only when its ID's **sector** byte equals the Sector register
**and** its **cylinder** byte equals the Track register. A sector-number match
on the wrong cylinder is RNF, not data. This is real WD179x behaviour and the
only thing that makes a positioning bug visible. The **head (side) byte is not
compared**: MZ-800 media does not guarantee head byte == physical side, and an
attempt to enforce it broke booting.

A wrong-cylinder ID is also treated as a **position fix**: `fdc_resync_head()`
adopts the cylinder the disk reported and drives the head back to
`head_target`, so the host's retry does not repeat from the same wrong place
(`posfix 0` disables it; counted as `pos_resyncs`).

### READ SECTOR / READ MULTIPLE

- **Window: one revolution + one sector span, in TIME.** The sector under the
  head when the capture starts is lost at the front and must be caught whole on
  its next pass, so the window is `rev + span`, capped at 300 ms. The span is
  one sector (floored at 512 bytes, sized at DD unless the host has declared a
  density) plus slack. A flux-transition count must never be used instead: it
  is data-dependent (~38k per DD revolution of ordinary data, ~47k on an empty
  directory track), and the old 45 700-count window was shorter than a
  revolution exactly on directory tracks.
- **The revolution is measured per drive from `~INDEX`**, by one continuous
  watcher sampled from every capture loop and disk-task pass. An edge counts
  only after the line has been low for 200 µs (a glitch is µs, a real pulse
  ms). A period is trusted only once two samples agree within 2 %; after that a
  sample within 5 % is averaged in, and a larger change is adopted only when it
  repeats. Default before the first measurement: 220 ms (covers the slowest
  drive seen, 218.7 ms). `rddiag` shows the measured period.
- **Decoded while capturing; answered early.** The capture fills a DMA ring
  (`arch_rp2.cpp`, 16 KB, with absolute-counter overrun detection), so the
  disk task can decode between drains. Each decode covers the newest flux plus
  one sector span of overlap, and the READ is answered the moment the
  requested sector is in the buffer whole with a **good data CRC** — on
  average about half a revolution after the command, not a fixed window. A
  ring overrun throws the capture away and restarts it (not counted as a
  search attempt).
- **The capture keeps running for READ MULTIPLE.** It is not stopped when the
  first sector is found; it runs on for the rest of its window, so the next
  sector is normally already in the buffer when the host asks for it. A
  sector not yet captured starts a fresh window. The capture ends at once when
  the command is aborted or superseded (Force Interrupt, any new command).
- **A missing sector is searched for over 4 windows** (~4.4 revolutions, the
  WD's "until the fifth index pulse") before RNF, each starting at a new
  rotational phase. A wrong-cylinder result skips the extra windows (another
  revolution cannot move the head; the resync is the fix).
- **A data CRC error is reported, never retried internally.** If no good copy
  turns up inside the window, the first copy of the sector in the capture is
  delivered with CRC ERROR set, exactly as read; no further window is spent on
  it. (The early-answer path can only pick a later, good copy when the one
  window happens to hold two whole passes of the sector — i.e. its first pass
  began right at the window start.) CRC ERROR is sticky across a READ MULTIPLE
  and ends it, as on a WD.
- Deleted data marks set the record-type bit.

### READ ADDRESS

Returns the first CRC-valid ID field as the six bytes a WD hands over DRQ
(track, side, sector, size code, CRC1, CRC2 — the recorded CRC bytes, not a
recomputation), and loads the ID's track byte into the Sector register (WD
quirk; the CP/M BIOS reads head position from there). It captures in chunks
and stops at the first ID, bounded at 40 ms, because CP/M polls for only
~52 ms. An undrained READ ADDRESS completes silently after 2 ms.

### WRITE SECTOR / WRITE MULTIPLE — in-place splice

- DRQ goes up as soon as the command is written, as on a WD; core1 **buffers
  the whole sector** from the host before core0 touches the disk. A host that
  stalls therefore stalls before any flux is committed (LOST DATA, nothing
  written) instead of half way through a data field.
- **The sector length is not needed in advance.** Collection is bounded by the
  per-drive, per-side, per-cylinder size cache (learned from every decoded ID;
  default 512 bytes), but a gap of 4 ms in the host's byte stream ends the
  sector at whatever the host actually sent. The **ID field's size code is the
  authority**: at splice time the sent length must equal `128 << N` from the
  ID, or the write is refused (RNF, nothing written). This is what lets a
  16×256 boot track on a disk nobody has read be written.
- A *live* MFM decoder finds the sector's ID as it passes the head (≤ 500 ms
  hunt, ~2.3 revolutions), then WGATE goes on in the ~22-byte Gap 2 and only
  the data field is rewritten (12 zeros, 3×`0xA1` sync, DAM, data, CRC, 3 gap
  bytes). ID fields, Gap 2 and the rest of Gap 3 stay as recorded.
- **Refused before any flux** on physical `~WPT`, on `wren 0`, and on an ID
  track compare miss — critical here, since a splice at the wrong cylinder
  destroys data rather than returning garbage. WRITE MULTIPLE splices each
  sector on its own pass.
- Write precompensation is not implemented (200/200 `wrsoak` writes verified
  over tracks 0–76; see CLAUDE.md).

### WRITE TRACK (format) / READ TRACK

- WRITE TRACK collects the host's whole track image over DRQ (ended by a 4 ms
  gap in the byte stream, or the 13 000-byte buffer filling), then encodes it
  index-to-index. The WD write-channel bytes are obeyed: `F5` writes `0xA1`
  with a missing clock and presets the CRC (once per run), `F6` writes `0xC2`
  with a missing clock, `F7` emits the CRC, everything else is literal. Short
  images are padded with gap bytes.
- **Image check:** at least 256 bytes **and** at least one real IDAM preamble —
  ≥ 2 × `0xF5` immediately followed by `0xFE`. (Merely containing an `0xF5` and
  an `0xFE` somewhere is satisfied by ~86 % of arbitrary 512-byte sectors, so
  that weaker check protected nothing.) A failing image writes nothing.
- **Host formatting defaults ON** (`fmten 0` turns it off). A spurious WRITE
  TRACK collects no valid image and writes nothing; a refusal, by contrast,
  looks to a format routine exactly like a frozen machine. `~WPT` and `wren`
  still gate it; `wrdiag` names which of `~WPT` / `wren` / `fmten` refused.
- READ TRACK returns every byte of one revolution uninterpreted (gaps, marks,
  CRCs), free-running and resynchronising on each `0xA1`. Known cosmetic quirk:
  data-field sync runs print as five `0xA1`s rather than three.

## Architecture — mirrors the chip's host/disk split

- **Core 1 — host/register side:** the bus PIO (pio1) captures each I/O
  cycle's address (and, for writes, the data byte as a second FIFO word);
  core1 decodes ports `0xD8`–`0xDF`, runs the WD register state machine,
  computes STATUS, raises/clears DRQ/BUSY, drives `~INT`, answers reads by
  driving C0..C7 and then queuing a verdict word that lets the read SM turn
  the data buffer toward the bus (all three '245 gates are PIO side-set; a
  port nobody serves is never driven), and handles the inverted bus. Always
  responsive; nothing may be added to its listen loop. Bus RESET is an edge
  interrupt on core1.
- **Core 0 — disk side:** the data separator and write channel. Drives
  STEP/DIR/HDSEL/MOTOR/DS0/DS1/DENSEL/WGATE; captures flux into the DMA ring
  (pio0) and decodes it into a one-sector buffer; runs the live-decoding write
  splice; encodes WRITE TRACK images; measures the spindle from `~INDEX`.
  core0 disables interrupts only for its own timed sections (the splice, flux
  write), which does not stall core1.
- **Shared:** the register file and flags (BUSY/DRQ/errors, sequence numbers
  and a command generation that discards results of superseded commands), a
  one-sector transfer buffer whose ownership passes through DRQ, and a
  track-sized buffer for READ/WRITE TRACK.

## Testing

- **Host test suite** (`tests/`, see [tests/README.md](../tests/README.md)):
  `fdc.cpp`, `floppy.cpp` and the MFM code compile unmodified against a
  simulated two-core RP2040, a rotating flux-level drive and an emulated
  MZ-800. Suites cover the codec, the read window at every rotational phase,
  and the WD179x through its registers (Type I bursts, READ/READ MULTIPLE, CRC,
  deleted marks, RNF, track compare, READ ADDRESS, write splice integrity).
- **On-machine test** `FDTEST.MZF` ([mzf/fdtest/README.md](../mzf/fdtest/README.md)):
  runs on the MZ-800 against this bridge or an original Sharp card — density,
  Type I, geometry, reads, READ MULTIPLE + Force Interrupt, CP/M-style STEP
  bursts, and (scratch disk only) writes, deleted marks and format. The same
  binary runs in CI on the emulated MZ-800.
- **Bench probe** `tools/mzp_probe.py` over USB: `health` first every session,
  then `rddiag`/`wrdiag`/`firstfail`, `rdsoak`, `wrverify`, `wrsoak`,
  `fmtverify` (the last three on a scratch disk).

## Scope / limitations

- **MFM** only, DD and HD; FM/single density only if a disk needs it.
- **Write precompensation** not implemented; add it if inner-track writes
  prove marginal on some drive or media.
- **Data pacing** is the host's, not the disk's (see the handshake section).
- **Weak bits** are reproduced by the real disk per command; only
  perfectly-exact analog reproduction is out of scope.
- **HD two-drive host writes under CP/M are not reliable yet** — see the HD
  status in CLAUDE.md and [hardware-notes.md](hardware-notes.md) §2b.

## Open items

- ~~Decode `0xDC`–`0xDE` bit assignments~~ — ✅ done, see the register map;
  `0xDE` is applied (density model + DENSEL, best-effort).
- ~~`~EXWAIT` timing budget~~ — ✅ core1 services a register access inside the
  ~560 ns Z80 I/O window.
- ~~Write-back for WRITE SECTOR~~ — ✅ in-place splice, verified on hardware
  (CP/M 4.1 writes files and reads them back correct).
- Measure, never enforce, the ID head byte per physical side.
- No `~M1` on this board: decode is `~IORQ` + address only, so an IORQ-based
  interrupt-acknowledge cycle is not distinguished. Believed benign; not
  confirmed.
