#!/usr/bin/env python3
"""MZPico Phase-2 probe — drive the firmware's on-device MFM sector layer.

Usage:
    python mzp_probe.py PORT scan    <track> <side> [dens]
    python mzp_probe.py PORT enctest <n_sectors> <size_code> [dens]
    python mzp_probe.py PORT busdump                      # MZ-800 bus hello state
    python mzp_probe.py PORT gpioread [once]              # live C0..C7 input byte
    python mzp_probe.py PORT pintest                     # per-pad pull-up/down self-test
    python mzp_probe.py PORT fdcdump [loop]              # WD FDC state (1 line; loop=on-change)
    python mzp_probe.py PORT fdclog                      # recent WD command opcodes
    python mzp_probe.py PORT fdcseek [track] [param_ms] [mode]  # bench seek; mode 1=spin_motor 0=raw
    python mzp_probe.py PORT fdcread <track> <sector> [side] [dens]  # bench READ SECTOR via the FDC
    python mzp_probe.py PORT streamscan <track> <side> [dens] [capture_ms]  # A/B: streaming vs batch capture
    python mzp_probe.py PORT bustestreset                 # reset controller-side 0xDF bus-test counters
    python mzp_probe.py PORT bustest                      # 0xDF loopback + I/O-cycle timing diagnostics
    python mzp_probe.py PORT rddiag                       # why the last Type II read succeeded / RNF'd
    python mzp_probe.py PORT sideswap [0|1]               # invert ROM side-select -> physical head (no arg = query)

  MZ-800 bus-test programs to run against `bustest` (Z80 machine code):
    write only : 3E 00 D3 DF 3C 18 FB
    write+read : 0E DF 3E 00 D3 DF ED 40 3C 18 F9
    read-check : 3E 00 D3 DF F5 DB DF D3 DE F1 3C 18 F5
                 (LD A,0 / L: OUT (0DFh),A / PUSH AF / IN A,(0DFh) /
                  OUT (0DEh),A / POP AF / INC A / JR L)
                 The echo to 0xDE lets the controller verify the byte the Z80
                 actually latched -- the only way to test the READ direction.
    python mzp_probe.py PORT <track> <side> [dens]        # shorthand for scan

    dens: 0 = DD / 250 kbit/s (default), 1 = HD / 500 kbit/s
    size_code: 0=128, 1=256, 2=512 bytes/sector

  scan    - seek a real track, capture flux, decode its sectors on-device.
  enctest - encode a synthetic track then decode it (no disk); validates the
            MFM encoder + CRC generation via a round-trip.

Needs pyserial:  pip install pyserial
"""
import sys
import serial  # pip install pyserial

MZP_CMD_SCAN = 0x80
MZP_CMD_ENCTEST = 0x81
MZP_CMD_BUSDUMP = 0x82
MZP_CMD_DRIVE = 0x83
MZP_CMD_GPIOREAD = 0x84
MZP_CMD_PINTEST = 0x85
MZP_CMD_FDCDUMP = 0x86
MZP_CMD_FDCLOG = 0x87
MZP_CMD_FDCSEEK = 0x88
MZP_CMD_FDCREAD = 0x89
MZP_CMD_STREAMSCAN = 0x8A
MZP_CMD_BUSTEST_RESET = 0x8B
MZP_CMD_BUSTEST = 0x8C
MZP_CMD_RDDIAG = 0x8D
MZP_CMD_SIDESWAP = 0x8E


def read_exact(s, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = s.read(n - len(buf))
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def print_sectors(s, ns):
    print(f"{'#':>2}  {'C':>3} {'H':>3} {'R':>3} {'N':>2} {'size':>5}  "
          f"idCRC dataCRC mark")
    good = 0
    for k in range(ns):
        rec = read_exact(s, 5)
        if len(rec) < 5:
            print("  (short sector record)")
            break
        c, h, r, n, flags = rec
        id_ok, data_ok = bool(flags & 1), bool(flags & 2)
        has_data, deleted = bool(flags & 4), bool(flags & 8)
        size = 128 << (n & 7)
        mark = "deleted" if deleted else ("DAM" if has_data else "-")
        print(f"{k:>2}  {c:>3} {h:>3} {r:>3} {n:>2} {size:>5}  "
              f"{'ok' if id_ok else 'BAD':>5} "
              f"{('ok' if data_ok else 'BAD') if has_data else '-':>7}  {mark}")
        if id_ok and data_ok:
            good += 1
    return good


def do_scan(s, track, side, dens):
    s.write(bytes([MZP_CMD_SCAN, 5, track, side, dens]))
    s.flush()
    hdr = read_exact(s, 7)  # cmd, ack, ns, nflux(4)
    if len(hdr) < 7 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    ns, nflux = hdr[2], int.from_bytes(hdr[3:7], "little")
    print(f"track {track} side {side} dens {dens}: {ns} sectors, "
          f"{nflux} flux transitions\n")
    good = print_sectors(s, ns)
    print(f"\n{good}/{ns} sectors fully valid (ID+data CRC).")


def do_streamscan(s, track, side, dens, capture_ms):
    # A/B test: same decode as `scan`, but flux comes from the new incremental
    # (streaming) capture API instead of the batch capture_track call -- lets
    # us check whether that new capture path itself produces valid flux,
    # independent of live-FDC timing/settle questions.
    s.write(bytes([MZP_CMD_STREAMSCAN, 7, track, side, dens,
                   capture_ms & 0xFF, (capture_ms >> 8) & 0xFF]))
    s.flush()
    hdr = read_exact(s, 7)  # cmd, ack, ns, nflux(4)
    if len(hdr) < 7 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    ns, nflux = hdr[2], int.from_bytes(hdr[3:7], "little")
    print(f"[streaming] track {track} side {side} dens {dens} "
          f"capture_ms {capture_ms}: {ns} sectors, {nflux} flux transitions\n")
    good = print_sectors(s, ns)
    print(f"\n{good}/{ns} sectors fully valid (ID+data CRC).")


def do_enctest(s, nsec, sizecode, dens):
    s.write(bytes([MZP_CMD_ENCTEST, 5, nsec, sizecode, dens]))
    s.flush()
    hdr = read_exact(s, 9)  # cmd, ack, nsec, ns, valid, nflux(4 LE)
    if len(hdr) < 9 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    req, ns, valid = hdr[2], hdr[3], hdr[4]
    nflux = int.from_bytes(hdr[5:9], "little")
    print(f"encode round-trip: {req} sectors x {128 << (sizecode & 7)} bytes, "
          f"dens {dens}: encoded {nflux} flux, decoded {ns}, {valid} valid\n")
    print_sectors(s, ns)
    ok = (valid == req and ns >= req)
    print(f"\n{'PASS' if ok else 'FAIL'}: {valid}/{req} sectors round-tripped "
          f"with valid CRCs.")


def do_busdump(s):
    s.write(bytes([MZP_CMD_BUSDUMP, 2]))
    s.flush()
    # cmd, ack, last_raw[8], wr[8]*4, rd[8]*4, addr_low, addr_high, dat_low, dat_high
    hdr = read_exact(s, 78)
    if len(hdr) < 78 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    last_raw = hdr[2:10]
    wr = [int.from_bytes(hdr[10 + 4 * k:14 + 4 * k], "little") for k in range(8)]
    rd = [int.from_bytes(hdr[42 + 4 * k:46 + 4 * k], "little") for k in range(8)]
    addr_low, addr_high, dat_low, dat_high = hdr[74], hdr[75], hdr[76], hdr[77]
    print("port     reads    writes  last_raw_write")
    for k in range(8):
        print(f"0x{0xD8 + k:02X}  {rd[k]:8d}  {wr[k]:8d}  0x{last_raw[k]:02X}")

    def bit_activity(name, low, high):
        # A healthy line toggles -> bit set in BOTH low and high.
        toggling = low & high
        stuck_hi = high & ~low & 0xFF
        stuck_lo = low & ~high & 0xFF
        print(f"\n{name}: sampled-low=0b{low:08b}  sampled-high=0b{high:08b}")
        for k in range(8):
            m = 1 << k
            if toggling & m:
                v = "toggles OK"
            elif stuck_hi & m:
                v = "STUCK HIGH (never seen 0)"
            elif stuck_lo & m:
                v = "stuck low (never seen 1)"
            else:
                v = "-- no activity --"
            print(f"  bit{k} (C{k}): {v}")

    bit_activity("ADDRESS bus", addr_low, addr_high)
    bit_activity("WRITE-DATA (our ports)", dat_low, dat_high)


def do_gpioread(s, once):
    # Isolate on the first read (halts SMs, C-pins = SIO in, no pull), then poll.
    def sample(isolate):
        s.write(bytes([MZP_CMD_GPIOREAD, 3, 1 if isolate else 0]))
        s.flush()
        r = read_exact(s, 3)
        if len(r) < 3 or r[1] != 0:
            return None
        return r[2]

    def show(v):
        bits = " ".join(f"C{k}={(v >> k) & 1}" for k in range(8))
        print(f"C0..C7 = 0b{v:08b} (0x{v:02X})   {bits}")

    v = sample(True)
    if v is None:
        print("bad reply")
        return
    show(v)
    if once:
        return
    print("polling (Ctrl-C to stop) — jumper C0 to GND then 3.3V; C0 must reach 0 and 1:")
    try:
        while True:
            v = sample(False)
            if v is not None:
                show(v)
    except KeyboardInterrupt:
        pass


def decode_cmd(c):
    hi = c >> 4
    names = {0x0: "RESTORE", 0x1: "SEEK", 0x2: "STEP", 0x3: "STEP",
             0x4: "STEP-IN", 0x5: "STEP-IN", 0x6: "STEP-OUT", 0x7: "STEP-OUT",
             0x8: "READ-SEC", 0x9: "READ-SEC(M)", 0xA: "WRITE-SEC",
             0xB: "WRITE-SEC(M)", 0xC: "READ-ADDR", 0xD: "FORCE-INT",
             0xE: "READ-TRACK", 0xF: "WRITE-TRACK"}
    return names.get(hi, "?")


STATUS_BITS_T1 = ["B", "-", "T00", "CRC", "Seek", "HL", "WP", "NRdy"]
STATUS_BITS_T2 = ["B", "DRQ", "Lost", "CRC", "RNF", "Del", "-", "NRdy"]


def fdc_line(r):
    (status, track, sector, data, command, flags, drive,
     at0, phys, opseq, opdone) = r[2:13]
    rd_flags = rd_sector = xpos = xlen = rdseq = rddone = None
    rd_elapsed_us = cmd_gap_us = None
    if len(r) >= 21:
        rd_flags, rd_sector = r[13], r[14]
        xpos = r[15] | (r[16] << 8)
        xlen = r[17] | (r[18] << 8)
        rdseq, rddone = r[19], r[20]
    if len(r) >= 25:
        rd_elapsed_us = (r[21] | (r[22] << 8) | (r[23] << 16) |
                         (r[24] << 24))
    if len(r) >= 29:
        cmd_gap_us = (r[25] | (r[26] << 8) | (r[27] << 16) | (r[28] << 24))
    verify_start_ok = verify_n = None
    if len(r) >= 34:
        verify_start_ok = r[29]
        verify_n = (r[30] | (r[31] << 8) | (r[32] << 16) | (r[33] << 24))
    cmd_writes_total = cmd_last_raw = cmd_last_decoded = None
    if len(r) >= 38:
        cmd_writes_total = r[34] | (r[35] << 8)
        cmd_last_raw = r[36]
        cmd_last_decoded = r[37]
    bt_active = bt_last_rx = bt_last_echo = None
    bt_rx_count = bt_rd_count = bt_err_count = None
    if len(r) >= 47:
        bt_active = r[38]
        bt_last_rx = r[39]
        bt_last_echo = r[40]
        bt_rx_count = r[41] | (r[42] << 8)
        bt_rd_count = r[43] | (r[44] << 8)
        bt_err_count = r[45] | (r[46] << 8)
    type2 = bool(rd_flags is not None and (rd_flags & 0x01))
    bits = STATUS_BITS_T2 if type2 else STATUS_BITS_T1
    sb = "".join(n + " " for k, n in enumerate(bits) if status & (1 << k) and n != "-")
    line = (f"ST={status:02X}[{sb.strip()}] TRK={track:<2} SEC={sector:<2} "
            f"DAT={data:02X} CMD={command:02X}({decode_cmd(command)}) "
            f"mot={'1' if flags & 0x80 else '0'} sd={1 if flags & 4 else 0} "
            f"wp={1 if flags & 2 else 0} busy={1 if flags & 1 else 0} "
            f"t0={at0} phys={'?' if phys == 0xFF else phys} seek={opdone}/{opseq}")
    if cmd_gap_us is not None:
        line += f"  gap_since_seek={cmd_gap_us / 1000:.1f}ms"
    if verify_n is not None:
        line += f"  verify[start_ok={verify_start_ok} n={verify_n}]"
    if cmd_writes_total is not None:
        line += (f"  cmds[n={cmd_writes_total} "
                 f"last_raw=0x{cmd_last_raw:02X} "
                 f"last_dec=0x{cmd_last_decoded:02X}]")
    if bt_active is not None:
        line += (f"  bt16[active={bt_active} last_rx=0x{bt_last_rx:02X} "
                 f"last_echo=0x{bt_last_echo:02X} "
                 f"rx={bt_rx_count} rd={bt_rd_count} err={bt_err_count} (16-bit, wraps; use `bustest`)]")
    if rd_flags is not None:
        reading = bool(rd_flags & 0x02)
        drq = bool(rd_flags & 0x04)
        multi = bool(rd_flags & 0x08)
        rnf = bool(rd_flags & 0x10)
        crc = bool(rd_flags & 0x20)
        deleted = bool(rd_flags & 0x40)
        elapsed = f" abort_after={rd_elapsed_us / 1000:.1f}ms" if rd_elapsed_us else ""
        line += (f"  [rd: type2={int(type2)} reading={int(reading)} "
                 f"drq={int(drq)} multi={int(multi)} rnf={int(rnf)} "
                 f"crc={int(crc)} del={int(deleted)} "
                 f"rdsec={rd_sector} xfer={xpos}/{xlen} "
                 f"rdseq={rddone}/{rdseq}{elapsed}]")
    return line


def do_rddiag(s):
    s.write(bytes([MZP_CMD_RDDIAG, 2]))
    s.flush()
    r = read_exact(s, 46)
    if len(r) < 46 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}")
        return

    def u32(off):
        return int.from_bytes(r[off:off + 4], "little")

    capture_ok = r[2]
    flux_n, capture_us, search_us, abort_us = u32(3), u32(7), u32(11), u32(15)
    want, result, ids_seen = r[19], r[20], r[21]
    ids = list(r[22:38])
    status_polls, status_span_us = u32(38), u32(42)

    found = bool(result & 1)
    flags = " ".join(n for b, n in
                     ((1, "found"), (2, "has_data"), (4, "crc_ok"),
                      (8, "deleted")) if result & b) or "-"
    print(f"last Type II read: sector {want}  -> {'OK' if found else 'RNF'}  "
          f"[{flags}]")
    print(f"  capture : started={capture_ok}  {flux_n} flux bytes in "
          f"{capture_us / 1000:.1f} ms  (~{flux_n / 43500:.2f} revolutions)")
    print(f"  decode  : {search_us / 1000:.1f} ms")
    if not found:
        if ids_seen:
            shown = ids[:min(ids_seen, 16)]
            print(f"  track HAS {ids_seen} sector IDs: "
                  f"{', '.join(str(v) for v in shown)}"
                  f"{' ...' if ids_seen > 16 else ''}")
            if want not in shown and ids_seen <= 16:
                print(f"  -> sector {want} is NOT on this track/side")
        else:
            print("  track HAS 0 decodable sector IDs -> the captured flux is "
                  "unusable (wrong side/density, or flux was lost)")
    print(f"  host    : {status_polls} STATUS polls over "
          f"{status_span_us / 1000:.1f} ms after the READ command")
    if status_span_us:
        print(f"            ^ this is the real DRQ patience budget; one disk "
              f"revolution is ~200 ms")
    if abort_us:
        print(f"  aborted by Force Interrupt after {abort_us / 1000:.1f} ms")


def do_bustest(s):
    s.write(bytes([MZP_CMD_BUSTEST, 2]))
    s.flush()
    r = read_exact(s, 144)
    if len(r) < 144 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}")
        return

    def u32(off):
        return int.from_bytes(r[off:off + 4], "little")

    active, last_rx, last_echo = r[2], r[3], r[4]
    rx, rd, err = u32(5), u32(9), u32(13)
    err_gap, err_garbage = u32(17), u32(21)
    bits_want1, bits_want0, mm_n = r[25], r[26], r[27]
    mm_exp, mm_got = r[28:36], r[36:44]
    wr_late_pop, wr_late_sample = u32(44), u32(48)
    rd_late_pop = u32(52)
    wr_backlog, rd_backlog = u32(56), u32(60)
    bit_high = [u32(64 + 4 * k) for k in range(8)]
    err_adjacent = u32(96)
    rd_reports, rd_err = u32(100), u32(104)
    rd_bits_want1, rd_bits_want0 = r[108], r[109]
    rd_last_exp, rd_last_got = r[110], r[111]
    rd_bit_high = [u32(112 + 4 * k) for k in range(8)]

    pct = (100.0 * err / rx) if rx else 0.0
    print(f"bus test (port 0xDF): active={active} last_rx=0x{last_rx:02X} "
          f"last_echo=0x{last_echo:02X}")
    print(f"  writes seen : {rx}")
    print(f"  reads  seen : {rd}")
    print(f"  mismatches  : {err}  ({pct:.2f} %)")
    print(f"    sequence gap (cycle(s) LOST, byte itself OK) : {err_gap}")
    print(f"    garbage      (captured byte CORRUPT)         : {err_garbage}")
    if rx and abs(pct - 100.0 * 2 / 256) < 0.15 and not err_gap:
        print("    NB: ~0.78 % with every mismatch on one bit is the signature")
        print("        of a PERMANENTLY stuck data line, not an occasional glitch")
        print("        (an incrementing sequence only trips the check twice per 256).")
    if bits_want1 or bits_want0:
        print(f"  bit errors  : want1-got0=0b{bits_want1:08b}  "
              f"want0-got1=0b{bits_want0:08b}")
        for k in range(8):
            m = 1 << k
            tags = []
            if bits_want1 & m:
                tags.append("1->0")
            if bits_want0 & m:
                tags.append("0->1")
            if tags:
                print(f"    C{k}: {' '.join(tags)}")
    if mm_n:
        print("  last mismatches (expected -> got):")
        for k in range(mm_n):
            e, g = mm_exp[k], mm_got[k]
            d = (g - e) & 0xFF
            note = f"gap +{d}" if 1 <= d <= 8 else f"xor=0x{e ^ g:02X}"
            print(f"    0x{e:02X} -> 0x{g:02X}   ({note})")
    if rx:
        # An incrementing test sequence drives every bit high exactly 50 % of
        # the time, so this measures each line directly -- unlike the last+1
        # check, which couples adjacent writes.
        print("  line duty (incrementing sequence -> every line must be 50.00 %):")
        for k in range(8):
            duty = 100.0 * bit_high[k] / rx
            dev = duty - 50.0
            flag = "" if abs(dev) < 0.02 else f"   <-- {dev:+.2f} pp  FAULTY"
            print(f"    C{k}: {duty:6.2f} %{flag}")
    if err:
        adj = 100.0 * err_adjacent / err
        shape = ("single-cycle glitches (error+recovery pairs)" if adj > 30
                 else "long stuck runs (isolated errors ~128 apart)")
        print(f"  mismatch shape: {err_adjacent}/{err} adjacent ({adj:.1f} %) "
              f"-> {shape}")
    if rd_reports:
        rpct = 100.0 * rd_err / rd_reports
        print(f"  READ-direction check (bytes echoed back via 0xDE):")
        print(f"    reports {rd_reports}   mismatches {rd_err}  ({rpct:.2f} %)")
        if rd_err:
            print(f"    last: expected 0x{rd_last_exp:02X} -> got "
                  f"0x{rd_last_got:02X}   "
                  f"want1-got0=0b{rd_bits_want1:08b} "
                  f"want0-got1=0b{rd_bits_want0:08b}")
        bad = [k for k in range(8)
               if abs(100.0 * rd_bit_high[k] / rd_reports - 50.0) >= 0.02]
        print("    line duty: " + ("all 50.00 %" if not bad else ", ".join(
            f"C{k}={100.0 * rd_bit_high[k] / rd_reports:.2f} %" for k in bad)))
    elif rd:
        print("  READ-direction check: not run (bytes read by the Z80 are")
        print("    UNVERIFIED -- use the 0xDE echo program, see --help)")
    print("  I/O-cycle timing:")
    print(f"    write cycle already over when core1 popped : {wr_late_pop}")
    print(f"    write cycle ended mid-sample               : {wr_late_sample}")
    print(f"    read  cycle already over when core1 popped : {rd_late_pop}")
    print(f"    FIFO backlog (write / read)                : "
          f"{wr_backlog} / {rd_backlog}")


def do_fdcdump(s, loop):
    def sample():
        s.write(bytes([MZP_CMD_FDCDUMP, 2]))
        s.flush()
        r = read_exact(s, 47)
        if len(r) < 47 or r[1] != 0:
            return None
        return r

    r = sample()
    if r is None:
        print("bad reply")
        return
    print(fdc_line(r))
    if not loop:
        return
    print("polling on-change (Ctrl-C to stop):")
    last = None
    import time
    try:
        while True:
            r = sample()
            if r is not None and r != last:
                print(fdc_line(r))
                last = r
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


def do_fdclog(s):
    s.write(bytes([MZP_CMD_FDCLOG, 2]))
    s.flush()
    hdr = read_exact(s, 5)  # cmd, ACK, count_lo, count_hi, n
    if len(hdr) < 5 or hdr[1] != 0:
        print(f"bad reply: {hdr.hex()}")
        return
    count = hdr[2] | (hdr[3] << 8)
    n = hdr[4]
    payload = read_exact(s, n * 2)
    if len(payload) < n * 2:
        print(f"short payload: got {len(payload)} want {n*2}")
        return
    ops = payload[:n]
    raws = payload[n:n*2]
    print(f"total commands: {count}; last {n} (oldest->newest):")
    for k, c in enumerate(ops):
        raw = raws[k]
        print(f"  dec=0x{c:02X} ({decode_cmd(c):<10}) raw=0x{raw:02X}")


def do_fdcread(s, track, sector, side, dens):
    # Bench (no MZ-800): device seeks, captures the track's flux, and decodes
    # the requested sector via the same mfm_read_sector() path the real Type II
    # READ SECTOR command uses. Validates the whole READ pipeline before wiring
    # it to the MZ-800 bus.
    s.write(bytes([MZP_CMD_FDCREAD, 6, track, sector, side, dens]))
    s.flush()
    hdr = read_exact(s, 5)  # cmd, ACK, flags, size_lo, size_hi
    if len(hdr) < 5 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    flags = hdr[2]
    size = hdr[3] | (hdr[4] << 8)
    found, has_data = bool(flags & 1), bool(flags & 2)
    data_crc_ok, deleted = bool(flags & 4), bool(flags & 8)
    data = read_exact(s, size) if size else b""
    print(f"track {track} sector {sector} side {side} dens {dens}:")
    print(f"  found={found} has_data={has_data} "
          f"data_crc={'ok' if data_crc_ok else 'BAD'} deleted={deleted} "
          f"size={size}")
    if not found:
        print("  RNF (Record Not Found) -- no IDAM matched with a valid ID CRC")
        return
    if data:
        preview = " ".join(f"{b:02X}" for b in data[:32])
        print(f"  first bytes: {preview}{' ...' if size > 32 else ''}")


def do_pintest(s):
    s.write(bytes([MZP_CMD_PINTEST, 2]))
    s.flush()
    r = read_exact(s, 4)
    if len(r) < 4 or r[1] != 0:
        print(f"bad reply: {r.hex()}")
        return
    pu, pd = r[2], r[3]
    print(f"pull-up  read = 0b{pu:08b} (0x{pu:02X})   want 0xFF")
    print(f"pull-down read = 0b{pd:08b} (0x{pd:02X})   want 0x00")
    print("\npin  pull-up  pull-down  verdict")
    for k in range(8):
        u, d = (pu >> k) & 1, (pd >> k) & 1
        if u == 1 and d == 0:
            v = "OK (pad follows pull)"
        elif u == 1 and d == 1:
            v = "STUCK HIGH (dead pad or net shorted to 3.3V)"
        elif u == 0 and d == 0:
            v = "STUCK LOW (dead pad or net shorted to GND/driven)"
        else:
            v = "INVERTED?! (u=0,d=1 — wiring odd)"
        print(f"C{k}     {u}        {d}       {v}")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    port = sys.argv[1]
    with serial.Serial(port, 115200, timeout=8) as s:
        s.reset_input_buffer()
        cmd = sys.argv[2]
        if cmd == "scan":
            t, sd = int(sys.argv[3]), int(sys.argv[4])
            dens = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            do_scan(s, t, sd, dens)
        elif cmd == "enctest":
            nsec, sc = int(sys.argv[3]), int(sys.argv[4])
            dens = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            do_enctest(s, nsec, sc, dens)
        elif cmd == "busdump":
            do_busdump(s)
        elif cmd == "gpioread":
            once = len(sys.argv) > 3 and sys.argv[3] == "once"
            do_gpioread(s, once)
        elif cmd == "pintest":
            do_pintest(s)
        elif cmd == "fdcdump":
            loop = len(sys.argv) > 3 and sys.argv[3] == "loop"
            do_fdcdump(s, loop)
        elif cmd == "fdclog":
            do_fdclog(s)
        elif cmd == "fdcseek":
            # Synchronous bench seek: device does the whole move, then replies
            # with the result. args: target [motor_settle_ms].
            target = int(sys.argv[3]) if len(sys.argv) > 3 else 0
            param = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            mode = int(sys.argv[5]) if len(sys.argv) > 5 else 1
            s.write(bytes([MZP_CMD_FDCSEEK, 6, target & 0xFF,
                           param & 0xFF, (param >> 8) & 0xFF, mode & 0xFF]))
            s.flush()
            r = read_exact(s, 47)
            if len(r) < 47 or r[1] != 0:
                print(f"bad reply: {r.hex()}")
            else:
                what = "RESTORE" if target == 0 else f"SEEK {target}"
                mdesc = "spin_motor+index" if mode == 1 else "motor_raw+delay"
                print(f"after {what} ({mdesc}, param {param}ms):\n  {fdc_line(r)}")
        elif cmd == "bustestreset":
            s.write(bytes([MZP_CMD_BUSTEST_RESET, 2]))
            s.flush()
            r = read_exact(s, 2)
            print("bus test reset: "
                  f"{'ok' if len(r) == 2 and r[1] == 0 else 'FAIL'}")
        elif cmd == "bustest":
            do_bustest(s)
        elif cmd == "rddiag":
            do_rddiag(s)
        elif cmd == "sideswap":
            # no arg = query (2), else 0 = normal, 1 = inverted
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_SIDESWAP, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            if len(r) < 3 or r[1] != 0:
                print(f"bad reply: {r.hex()}")
            else:
                print("side select: " + ("INVERTED (ROM side 0 -> physical "
                                         "head 1)" if r[2] else
                                         "normal (ROM side 0 -> physical "
                                         "head 0)"))
        elif cmd == "fdcread":
            track = int(sys.argv[3])
            sector = int(sys.argv[4])
            side = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            dens = int(sys.argv[6]) if len(sys.argv) > 6 else 0
            do_fdcread(s, track, sector, side, dens)
        elif cmd == "streamscan":
            track = int(sys.argv[3])
            side = int(sys.argv[4])
            dens = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            capture_ms = int(sys.argv[6]) if len(sys.argv) > 6 else 220
            do_streamscan(s, track, side, dens, capture_ms)
        elif cmd == "drive":
            val = int(sys.argv[3], 0)
            mode = int(sys.argv[4]) if len(sys.argv) > 4 else 1
            s.write(bytes([MZP_CMD_DRIVE, 5, val & 0xFF, mode & 0xFF, 0]))
            s.flush()
            r = read_exact(s, 2)
            print(f"drive C0..C7 = 0x{val & 0xFF:02X}, mode {mode} "
                  f"-> {'ok' if len(r) == 2 and r[1] == 0 else 'FAIL'} "
                  f"(measure Pico pins 1..; mode 0 to release)")
        else:  # shorthand: PORT track side [dens]
            t, sd = int(sys.argv[2]), int(sys.argv[3])
            dens = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            do_scan(s, t, sd, dens)


if __name__ == "__main__":
    main()
