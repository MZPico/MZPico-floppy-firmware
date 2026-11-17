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
- `fdtest_*` (ctest only): `FDTEST.MZF` running on the emulated MZ-800 (answers are typed lines through the monitor's GETL, as on the machine)
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

One: `flux_dropout_between_id_and_orphan_data_does_not_pair_them` (#45: a
flux dropout under-counted by the 8-bit capture clamp). #2, #4, #8 and #20 started as XFAIL tests and are fixed.
Every fixed review bug keeps a test that fails on the old firmware:

| Issue | Test |
|---|---|
| #1 stale read completion breaks a write | `stale_read_completion_does_not_break_a_write_being_fed` |
| #2 side not re-applied after a seek | `side_change_after_seek_reaches_the_read` |
| #3 new read answered from the old capture | `read_issued_during_a_capture_*` |
| #4 ID paired with a far data mark | `id_without_data_field_is_not_paired_...`, `scan_does_not_swallow_...` |
| #7 WRITE TRACK hangs on an empty drive | `write_track_on_empty_drive_fails_instead_of_hanging` |
| #8 unit 2/3 wedges the controller | `unit2_selected_...`, `type1_on_missing_unit_does_not_move_drive0` |
| #20 Verify accepts any cylinder | `seek_verify_detects_wrong_cylinder`, `verify_after_lost_steps_...` |
| #21 size code 4..7 overruns the buffer | `oversized_size_code_cannot_overrun_the_transfer_buffer` |
| #22 write refused from the other drive's WP | `write_to_writable_drive_right_after_switching_from_protected_one` |
| #32 reading before the head can | `read_right_after_a_step_is_clean`, `first_read_after_a_cold_spin_up_is_clean` |
| #51 a later good copy hides a CRC error | `weak_sector_*`, `read_sector_reports_a_field_cut_off_by_the_end_of_the_flux` |

## Limits

- The interleave is about 1 µs, not one instruction. A race narrower than that
  is only reachable by a test that forces the ordering.
- Bus timing is not simulated: there is no PIO, EXWAIT or ~560 ns window.
  Rules about core1's listen loop still have to be checked on hardware
  (`mzp_probe.py health`, `BUSTEST.MZF`).
- Splices leave a 20 µs flux-free junction at each end (write-current ramp,
  tunnel erase), as on hardware: a decoder rule that rejected such intervals
  passed CI and failed FDTEST on the machine until this was modelled.
- After a step the head rings (noise, then marginal jitter), and data read
  before the spindle is at speed is garbage (#32).
- A sector can be marked `weak`: part of its data field reads noisily on even
  revolutions and cleanly on odd ones, so its CRC alternates by pass (#51).
- Otherwise the drive is ideal: a step always lands. Media effects come from
  the disk image: jitter, bad CRCs, missing fields, wrong IDs, weak bits.
- There is no Sharp ROM, so the boot ROM's own floppy code is not run. FDTEST
  follows its conventions instead (CPL bus, polled DRQ, Force Interrupt after
  READ MULTIPLE, inverted side).
