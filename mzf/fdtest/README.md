# FDTEST.MZF: floppy controller test for the MZ-800

FDTEST runs on the MZ-800 and checks the floppy controller at D8h–DFh, in
about a minute, without CP/M. It only speaks WD179x, so it runs on the original
Sharp card as well as on the MZPico bridge. The same binary runs in CI on the
emulated MZ-800 (`tests/`).

Get it from a GitHub release, the `mzf` artifact of a CI run, or build it with
`cmake -S mzf -B build-mzf && cmake --build build-mzf`. Load it the way you
load `BUSTEST.MZF`. It starts at 1200h and returns to the monitor when done.

## Using it

Each answer is typed and confirmed with **CR**, through the monitor's line input (GETL).


1. Pick the drive (`0`–`3`).
2. Pick the test:
   - `1` READ: read-only. Use any MZ-800 disk (CP/M, Disk BASIC, a game).
   - `2` WRITE: **scratch disk only**. It writes, and formats, cylinders near
     the hub (the last cylinder − 6 and − 4) on both sides.
   - `3` BOTH: the read checks, then the write checks if the reads passed.
3. For `2` or `3`, confirm with `Y`.

Each check prints `OK` or `FAIL:` with the true WD status in hex (`ST=`). The
last line is `FDTEST RESULT: PASS` or `FAIL`.

| Check | What it does |
|---|---|
| DENSITY | READ ADDRESS on cylinder 2 at DD, then HD, declaring it on port DEh as CP/M 4.1 does (00h / 81h) |
| DETECT CONTROLLER | Track register write/read-back on the inverted bus |
| RESTORE | TRK00 reached, drive ready |
| SEEK + VERIFY | Seeks with the V bit to 1, 2, 5, 10, 20, 39, 0; probes 40 or 80 cylinders |
| GEOMETRY | READ ADDRESS ×40 per side on cylinder 2: sector count, size, first sector |
| READ CYL 0 (BOOT) | ROM side 0 of cylinder 0: the 16×256 IPL track on a master disk, which is DD even on a CP/M HD disk |
| READ SECTORS | Every sector of cylinders 2, 3, 10, 20, 39, both sides; prints the latency from command to first DRQ |
| READ MULTIPLE + FI | 90h across a whole track, then Force Interrupt (STATUS must be clean, as the ROM requires); compared with single reads |
| STEP BURSTS (CP/M) | CP/M 4.1's STEP + FORCE INT bursts without waiting for BUSY, including surplus STEP OUT at track 0; READ ADDRESS must then find cylinder 7 |
| WRITE + READ BACK | Writes every sector of one cylinder on both sides, verifies each, then re-reads them all to check the splices damaged nothing |
| WRITE DELETED MARK | A1h writes a deleted data mark; reading it back must set the DELETED bit |
| FORMAT (WRITE TRACK) | Formats one cylinder on both sides with a standard WD image (9 or 18 × 512, E5h), then reads it back |

The latency line tells you how fast reads are answered. Today every READ
waits out the full capture window, about 230 ms, before decoding (roadmap
#35). A real WD answers in rotational latency, about 100 ms on average.
