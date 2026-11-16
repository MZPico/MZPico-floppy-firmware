# Host tests

The firmware's FDC bridge, tested on a PC: no Pico, no drive, no MZ-800.

```sh
cmake -S tests -B build-tests -G Ninja      # z88dk.zcc on PATH builds FDTEST.MZF too
cmake --build build-tests
ctest --test-dir build-tests -j"$(nproc)" --output-on-failure
```

Without the z88dk snap, use the pinned Docker image, as CI does:
`PATH=$PWD/tools/ci/z88dk:$PATH cmake -S tests -B build-tests -G Ninja`.
Without z88dk at all, the C++ suites still build and run; only the FDTEST runs
are skipped.

## What runs

`src/bus/fdc.cpp`, `src/floppy.cpp` and the MFM code compile **unmodified**
against `tests/sim`:

| Piece | Stands in for |
|---|---|
| `sim/sim_core.*` | the two RP2040 cores and the GPIO bank, on a virtual 24 MHz clock |
| `hal/include/` | the Pico SDK headers the firmware includes |
| `sim/sim_drive.*` | a Shugart drive on the 34-pin cable, and `arch_rp2` (flux capture/write) |
| `sim/disk_image.*` | test media: an independent MFM encoder, MZ-800 geometries, an EDSK loader |
| `sim/board.*` | the board glue (`gw_init`) and an MZ-800 driving the registers from C++ |
| `mz800/` | an emulated MZ-800 (Z80 + RAM + trapped monitor calls) that runs MZF programs |

- **core0** (the disk task) runs as a coroutine. Every HAL call costs it a few
  virtual ticks, and it yields at least every microsecond, so the register
  side (**core1**) interleaves with it at about 1 µs resolution.
  `sleep_ms` jumps straight ahead. Everything is deterministic: a failure seen
  once is seen on every run.
- **The disk** stores flux transition positions around each track, not bytes,
  and it turns. A read sees what passes under the head during the capture
  window, so rotational latency, the index, a sector that straddles the window
  start, and spindle speed (`rev_ticks`, e.g. 218.7 ms) are all real. A write
  replaces the transitions under the head for as long as it runs, so a
  mistimed splice destroys its neighbour here as it would on media.
- **The drive** obeys the pins: STEP on its trailing edge, DIR, the DS0/DS1
  select lines, one shared MOTOR line, TRK00, INDEX and WPT.
- **Test media** are encoded by `disk_image.cpp`, written from the format
  definition and independent of the firmware's `mfm_impl.h`. Tests therefore
  check the firmware's decoder against something other than its own encoder.

## Suites

- `test_mfm`: the codec, covering DD, HD, 16×256, dense data, flux jitter,
  WRITE TRACK images and the ID track compare.
- `test_read_window`: the read window at every rotational phase (the
  simulation behind commit 16e5310). It also checks why a single revolution,
  or a fixed transition count, is not enough.
- `test_fdc`: the WD179x through its registers, the way the MZ-800 uses them.
  Covers:
  - Type I commands, including CP/M's STEP/FORCE-INT bursts
  - READ / READ MULTIPLE
  - CRC errors, deleted marks, RNF and the track compare
  - READ ADDRESS
  - WRITE SECTOR splice integrity, plus a soak run
  - write-protect and wrong-cylinder refusals
  - WRITE TRACK and READ TRACK
  - HD, two drives, `~INT`/EINT
- `fdtest_*` (ctest only): `FDTEST.MZF` running on the emulated MZ-800
  against the firmware, on several media, plus two negative runs (a
  write-protected disk, an empty drive) that FDTEST must report as failures.
- `fdtest_local_*`: the same against every `*.dsk` (EDSK) image in
  `MZP_MEDIA_DIR`, which defaults to the repository root. Real disk images are
  copyrighted and never committed, so these runs exist only where the images do.

Every `TEST` runs in its own forked process: the firmware keeps its state in
file-scope statics, and a fresh process is the only honest power-up. Run one
binary with a substring to filter, e.g. `./build-tests/test_fdc write_`.

## Known bugs are tests too

A test for an open bug is marked `XFAIL("#N: ...")`. It passes the suite
while it fails, and it **fails the suite once it passes** (XPASS), so the fix
that closes the issue also has to remove the marker. Open ones:

| Issue | Test |
|---|---|
| #2 side not re-applied after a seek | `side_change_after_seek_reaches_the_read` |
| #4 ID paired with a far data mark | `id_without_data_field_is_not_paired_...`, `scan_does_not_swallow_...` |
| #8 unit 2/3 wedges the controller | `unit2_selected_reports_not_ready_then_unit0_still_works` |
| #20 Verify accepts any cylinder | `seek_verify_detects_wrong_cylinder` |

## Limits

- The interleave is about 1 µs, not one instruction. A race narrower than that
  is only reachable by a test that forces the ordering.
- Bus timing is not simulated: there is no PIO, EXWAIT or ~560 ns window.
  Rules about core1's listen loop still have to be checked on hardware
  (`mzp_probe.py health`, `BUSTEST.MZF`).
- The drive is ideal unless a test makes it otherwise. It has no weak bits, no
  head-settle noise, and a step always lands. Media effects come from the disk
  image: jitter, bad CRCs, missing fields, wrong IDs.
- There is no Sharp ROM, so the boot ROM's own floppy code is not run. FDTEST
  follows its conventions instead (CPL bus, polled DRQ, Force Interrupt after
  READ MULTIPLE, inverted side).
