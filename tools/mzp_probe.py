#!/usr/bin/env python3
"""MZPico Phase-2 probe — drive the firmware's on-device MFM sector layer.

Usage:
    python mzp_probe.py PORT scan    <track> <side> [dens]
    python mzp_probe.py PORT enctest <n_sectors> <size_code> [dens]
    python mzp_probe.py PORT busdump                      # MZ-800 bus hello state
    python mzp_probe.py PORT gpioread [once]              # live C0..C7 input byte
    python mzp_probe.py PORT pindrive <byte> [mode]       # bench: drive C0..C7 pads; `pindrive 0 0` restores the bus after gpioread
    python mzp_probe.py PORT pintest                     # per-pad pull-up/down self-test
    python mzp_probe.py PORT fdcdump [loop]              # WD FDC state (1 line; loop=on-change)
    python mzp_probe.py PORT fdclog                      # recent WD command opcodes
    python mzp_probe.py PORT fdcseek [track] [param_ms] [mode]  # bench seek; mode 1=spin_motor 0=raw
    python mzp_probe.py PORT fdcread <track> <sector> [side] [dens]  # bench READ SECTOR via the FDC
    python mzp_probe.py PORT rdsoak <track> <sector> [side] [n] [legacy]  # n reads via the LIVE window at random phases; legacy=1 = old 45700-transition window
    python mzp_probe.py PORT streamscan <track> <side> [dens] [capture_ms]  # A/B: streaming vs batch capture
    python mzp_probe.py PORT bustestreset                 # reset controller-side 0xDF bus-test counters
    python mzp_probe.py PORT bustest                      # 0xDF loopback + I/O-cycle timing diagnostics
    python mzp_probe.py PORT rddiag                       # why the last Type II read succeeded / RNF'd
    python mzp_probe.py PORT sideswap [0|1]               # invert ROM side-select -> physical head (no arg = query)
    python mzp_probe.py PORT intmode [0|1|2]              # ~INT drive: 0=off 1=follow EINT(0xDF) 2=force (no arg = query)
    python mzp_probe.py PORT iotrace                      # ordered register accesses (reads+writes), newest last
    python mzp_probe.py PORT wren [0|1]                   # allow WRITE SECTOR from the MZ-800 (no arg = query)
    python mzp_probe.py PORT wrdiag                       # why the last WRITE SECTOR wrote / was refused
    python mzp_probe.py PORT fdcwrite <track> <sector> [side] [sizecode] [mode] [seed]   # !! WRITES THE DISK
    python mzp_probe.py PORT wrverify <track> <sector> [side] [sizecode] [mode] [seed]   # !! write, read back, compare
    python mzp_probe.py PORT wrsoak <passes> <sector> <t_start> <t_end> [t_step] [side]  # !! write/verify soak over the surface (range required)
    python mzp_probe.py PORT wrburst <track> [side] [nsec] [seed] [rdtrack] [rddrive]  # !! splices; read rdtrack (from rddrive) before each = CP/M A->B
    python mzp_probe.py PORT firstfail [clear]            # the FIRST failure since power-up, held unaltered
    python mzp_probe.py PORT resetdiag [clear]            # what the bus RESET pin actually does (edges, pulse lengths)
    python mzp_probe.py PORT resetpoll [0|1]              # core1 bus-RESET poller on/off (no arg = query)
    python mzp_probe.py PORT posfix [0|1]                 # head-position self-correction on/off (no arg = query)
    python mzp_probe.py PORT fmten [0|1]                  # allow WRITE TRACK from the MZ-800 (default ON)
    python mzp_probe.py PORT drive [0|1]                  # which unit (~DS0/~DS1) the bench commands drive (no arg = query)
    python mzp_probe.py PORT drvlock [0|1]                # pin every access to drive 0, ignoring port 0xDC (no arg = query)
    python mzp_probe.py PORT drvstat                      # head position + target + readiness for BOTH units at once
    python mzp_probe.py PORT density [dd|hd|auto|detect]  # density; auto = host 0xDE else latch (default); detect adds media probing
    python mzp_probe.py PORT idxbit [0|1|2]               # Index status bit: 0 never, 1 only while host declares HD (default), 2 always
    python mzp_probe.py PORT densel [0|1|auto]            # DENSEL pin override for write-current experiments (no arg = query)
    python mzp_probe.py PORT fwver                        # git hash of the RUNNING firmware
    python mzp_probe.py PORT health                       # session sanity check: build + passive bus/contact screen (run FIRST)
    python mzp_probe.py PORT readtrack <track> [side] [limit]   # READ TRACK: raw decoded bytes + address marks
    python mzp_probe.py PORT fdcformat <track> [side] [nsec] [sizecode] [interleave] [fill]  # !! FORMATS THE TRACK
    python mzp_probe.py PORT fmtverify <track> [side] [nsec] [sizecode] [interleave] [fill]  # !! format, then scan it back

  MZ-800 bus-test programs to run against `bustest` (Z80 machine code):
    write only : 3E 00 D3 DF 3C 18 FB
    write+read : 0E DF 3E 00 D3 DF ED 40 3C 18 F9
    read-check : 3E 00 D3 DF F5 DB DF D3 DE F1 3C 18 F5
                 (LD A,0 / L: OUT (0DFh),A / PUSH AF / IN A,(0DFh) /
                  OUT (0DEh),A / POP AF / INC A / JR L)
                 The echo to 0xDE lets the controller verify the byte the Z80
                 actually latched -- the only way to test the READ direction.
    python mzp_probe.py PORT <track> <side> [dens]        # shorthand for scan

    dens: 0 = DD / 250 kbit/s, 1 = HD / 500 kbit/s; when omitted, read
          commands follow the firmware's effective density (see `density`)
    size_code: 0=128, 1=256, 2=512, 3=1024 bytes/sector
    write pattern mode: 0 = ramp from seed, 1 = constant seed,
                        2 = pseudo-random from seed (default; hardest MFM)

  scan    - seek a real track, capture flux, decode its sectors on-device.
  enctest - encode a synthetic track then decode it (no disk); validates the
            MFM encoder + CRC generation via a round-trip.
  firstfail - the diagnostic for faults that RETRY away. Every other command
            here shows the last operation, so a host's successful retry erases
            the failure that caused the symptom; this latches the first one.
  wrsoak  - the test that answers "does this need write precompensation on
            inner tracks": enough writes, spread over the surface, for a rare
            fault to show up. Overwrites one sector per track. Scratch disk.
  fdcformat/fmtverify - Milestone 3.4 WRITE TRACK. Formatting is the only way
            to create the missing-clock sync marks an ID field needs, so this
            is what makes a blank disk usable. It replaces the ENTIRE track.
            fmtverify is the one to run: format, then read the sectors back.
  readtrack - READ TRACK: every byte on the track, gaps and marks included, no
            interpretation. Byte boundaries in the gaps are arbitrary by
            nature; the address-mark count at the end is the useful summary.
  fdcwrite/wrverify - the Milestone 3.3 write splice. These OVERWRITE one
            sector's data field on the disk in the drive; use a scratch disk.
            wrverify is the one to run: it writes a pattern, reads it back
            through the normal read path, and compares byte for byte.

Needs pyserial:  pip install pyserial
"""
import sys
import serial  # pip install pyserial

MZP_CMD_SCAN = 0x80
MZP_CMD_ENCTEST = 0x81
MZP_CMD_BUSDUMP = 0x82
MZP_CMD_DRIVE = 0x83
MZP_CMD_DENSITY = 0x9E
MZP_CMD_IDXBIT = 0x9F
MZP_CMD_FWVER = 0xA0
MZP_CMD_DENSEL = 0xA1
MZP_CMD_RDWIN = 0xA2
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
MZP_CMD_INTMODE = 0x8F
MZP_CMD_IOTRACE = 0x90
MZP_CMD_FDCWRITE = 0x91
MZP_CMD_WRDIAG = 0x92
MZP_CMD_WREN = 0x93
MZP_CMD_FIRSTFAIL = 0x94
MZP_CMD_FDCFORMAT = 0x95
MZP_CMD_READTRACK = 0x96
MZP_CMD_RESETDIAG = 0x97
MZP_CMD_RESETPOLL = 0x98
MZP_CMD_POSFIX = 0x99
MZP_CMD_FMTEN = 0x9A
MZP_CMD_DRVSEL = 0x9B
MZP_CMD_DRVLOCK = 0x9C
MZP_CMD_DRVSTAT = 0x9D


REG_NAMES = {0xD8: "CMD/STATUS", 0xD9: "TRACK", 0xDA: "SECTOR", 0xDB: "DATA",
             0xDC: "MOTOR/DRV", 0xDD: "PAGE/SIDE", 0xDE: "DENSITY",
             0xDF: "EINT"}


# Ports 0xD8..0xDB are complemented on the MZ-800 bus; the firmware traces the
# raw bus byte, so undo it here -- one place, matching fdc.cpp's rule.
def _true_val(port, raw):
    return (~raw) & 0xFF if 0xD8 <= port <= 0xDB else raw


def _status_str(v, type2):
    bits = STATUS_BITS_T2 if type2 else STATUS_BITS_T1
    on = [bits[k] for k in range(8) if v & (1 << k) and bits[k] != "-"]
    return "[" + " ".join(on) + "]" if on else "[]"


def do_iotrace(s):
    s.write(bytes([MZP_CMD_IOTRACE, 2]))
    s.flush()
    hdr = read_exact(s, 7)  # cmd, ack, count, total(4 LE)
    if len(hdr) < 7 or hdr[1] != 0:
        print(f"bad reply: {hdr.hex()}{busy_hint(hdr)}")
        return
    n = hdr[2]
    total = int.from_bytes(hdr[3:7], "little")
    recs = []
    for _ in range(n):
        rec = read_exact(s, 5)
        if len(rec) < 5:
            break
        recs.append(rec)
    print(f"{total} register accesses total; last {len(recs)} shown "
          f"(oldest -> newest), identical consecutive accesses coalesced as xN:")
    print(f"  {'dir':<3} {'port':<5} {'reg':<10} {'raw':<5} {'true':<5} "
          f"{'count':>6}  note")
    # STATUS layout depends on the last command class, same as the WD.
    type2 = False
    for rec in recs:
        port, val, is_rd = rec[0], rec[1], rec[2]
        rep = int.from_bytes(rec[3:5], "little")
        tv = _true_val(port, val)
        note = ""
        if port == 0xD8 and not is_rd:
            note = decode_cmd(tv)
            hi = tv >> 4
            if 0x8 <= hi <= 0xB or hi == 0xC or hi == 0xE or hi == 0xF:
                type2 = True
            elif hi <= 0x7:
                type2 = False
        elif port == 0xD8 and is_rd:
            note = _status_str(tv, type2)
        rp = f"x{rep}" if rep > 1 else ""
        print(f"  {'RD' if is_rd else 'WR':<3} 0x{port:02X}  "
              f"{REG_NAMES.get(port, '?'):<10} 0x{val:02X}  0x{tv:02X}  "
              f"{rp:>6}  {note}")


MZP_ACK_HOST_BUSY = 0x80


def busy_hint(r):
    """Explain the firmware's refusal of a bench drive command (#15)."""
    if r and len(r) >= 2 and r[1] == MZP_ACK_HOST_BUSY:
        return (" -- refused: the MZ-800 has a floppy command in flight "
                "(the bench would move its drive under it); retry when idle")
    return ""


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


def effective_density(s):
    """The density the firmware will encode/decode with RIGHT NOW: a forced
    mode wins; auto follows the selected unit's latched density. Bench read
    commands default to this so a read-back always matches what the write
    side just did -- an explicit dens argument still overrides. (fmtverify
    once wrote a track at forced-HD and scanned it back at the DD default:
    "MISSING sectors: [1..9]" on a perfectly good track.)"""
    s.write(bytes([MZP_CMD_DENSITY, 3, 4]))  # 4 = query
    s.flush()
    r = read_exact(s, 11)
    if len(r) != 11 or r[1] != 0:
        return 0
    mode = r[2]
    if mode < 2:
        return mode
    if r[9]:  # the host has declared a density on 0xDE: it outranks the latch
        return 1 if r[10] else 0
    s.write(bytes([MZP_CMD_DRVSEL, 3, 4]))  # 4 = query only
    s.flush()
    d = read_exact(s, 5)
    unit = d[2] if len(d) == 5 else 0
    return 1 if (unit < 2 and r[3 + unit]) else 0


def do_scan(s, track, side, dens):
    s.write(bytes([MZP_CMD_SCAN, 5, track, side, dens]))
    s.flush()
    hdr = read_exact(s, 7)  # cmd, ack, ns, nflux(4)
    if len(hdr) < 7 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
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
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
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
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
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
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
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
# b6 in the Type II/III layout is WRITE PROTECT on write commands (WD179x);
# it reads 0 on reads, so labelling it is safe and never wrong. Left as "-" it
# hid the decisive bit: a host WRITE TRACK refused for write-protect polls
# STATUS forever on 0x40, and the trace rendered that as "[]" -- no reason
# visible for a hang whose reason was sitting in the byte.
STATUS_BITS_T2 = ["B", "DRQ", "Lost", "CRC", "RNF", "Del", "WP", "NRdy"]


def fdc_line(r):
    (status, track, sector, data, command, flags, drive,
     at0, phys, opseq, opdone) = r[2:13]
    # drive: low nibble = selected unit, high nibble = unit actually driven.
    # Everything physical below (phys/t0/wp) describes the DRIVEN one.
    drv_sel, drv_cur = drive & 0x0F, (drive >> 4) & 0x0F
    drv_txt = "drv=%d" % drv_sel if drv_sel == drv_cur else (
        "drv=%d(driving %s)" % (drv_sel, "none" if drv_cur == 0x0F else drv_cur))
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
            f"wp={'?' if drv_cur == 0x0F else (1 if flags & 2 else 0)} "
            f"busy={1 if flags & 1 else 0} "
            f"t0={'?' if drv_cur == 0x0F else at0} "
            f"phys={'?' if phys == 0xFF else phys} seek={opdone}/{opseq} "
            f"{drv_txt}")
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
    r = read_exact(s, 104)
    if len(r) < 104 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return

    def u32(off):
        return int.from_bytes(r[off:off + 4], "little")

    capture_ok = r[2]
    flux_n, capture_us, search_us, abort_us = u32(3), u32(7), u32(11), u32(15)
    want, result, ids_seen = r[19], r[20], r[21]
    ids = list(r[22:42])  # FDC_IDS_N = 20
    status_polls, status_span_us = u32(42), u32(46)
    int_flags, int_mode = r[50], r[51]
    intrq_asserts, lost_data_events = u32(52), u32(56)
    write_attempts = u32(60)
    verify_start_ok, verify_n, seek_error = r[64], u32(65), r[69]
    verify_attempts = u32(70)
    # Parsed here, with everything else, because the READ ADDRESS branch below
    # rebinds `r` to a size code and destroys the reply buffer.
    sec_ok, sec_crc_err, sec_rnf = u32(74), u32(78), u32(82)
    last_id_cyl, last_id_head = r[86], r[87]
    sec_cyl_mismatch = u32(88)
    last_trk_reg, last_want_side = r[92], r[93]
    last_head_target, last_phys_track = r[94], r[95]
    rev_us, window_us = u32(96), u32(100)

    found = bool(result & 1)
    is_readaddr = bool(result & 0x10)
    flags = " ".join(n for b, n in
                     ((1, "found"), (2, "has_data"), (4, "crc_ok"),
                      (8, "deleted")) if result & b) or "-"
    if is_readaddr:
        print(f"last command: READ ADDRESS -> {'OK' if found else 'RNF'}")
        if found:
            t, h, r, n = ids[0], ids[1], ids[2], ids[3]
            print(f"  ID field: track={t} side={h} sector={r} "
                  f"size={128 << (n & 7)}B (N={n})  "
                  f"CRC={ids[4]:02X}{ids[5]:02X}")
    else:
        print(f"last Type II read: sector {want}  -> "
              f"{'OK' if found else 'RNF'}  [{flags}]")
    rev_txt = (f"{rev_us / 1000:.1f} ms measured" if rev_us
               else "not measured yet, 220.0 ms assumed")
    rev_for_ratio = rev_us or 220000
    print(f"  capture : started={capture_ok}  {flux_n} flux bytes in "
          f"{capture_us / 1000:.1f} ms  (~{capture_us / rev_for_ratio:.2f} "
          f"revolutions; window {window_us / 1000:.1f} ms; rev {rev_txt})")
    print(f"  decode  : {search_us / 1000:.1f} ms")
    if not found and not is_readaddr:
        if ids_seen:
            shown = ids[:min(ids_seen, 20)]
            print(f"  track HAS {ids_seen} sector IDs: "
                  f"{', '.join(str(v) for v in shown)}"
                  f"{' ...' if ids_seen > 20 else ''}")
            if want not in shown and ids_seen <= 20:
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

    # `state` is a SNAPSHOT of this instant; `asserts` is cumulative since the
    # Pico powered up. Do not read "EINT clear" as "this host never arms it":
    # CP/M 4.1 writes 0x01 to 0xDF before each data command and 0x00 after, so
    # any sample taken between commands sees it clear. The assert count is the
    # only trustworthy evidence of whether interrupts are actually happening.
    mode_name = {0: "off", 1: "EINT-gated", 2: "forced"}.get(int_mode, "?")
    istate = " ".join(n for b, n in ((1, "EINT-armed"), (2, "~INT-low"),
                                     (4, "force-int-latched"))
                      if int_flags & b) or "idle"
    print(f"  ~INT    : mode={int_mode} ({mode_name})  now={istate}  "
          f"{intrq_asserts} asserts since power-up")
    if int_mode == 1 and intrq_asserts == 0:
        print("            ^ ~INT has NEVER asserted. Either every host so far "
              "polls (the boot ROM and CP/M 2.3 never write 0xDF), or the "
              "interrupt path is broken. 'now=idle' alone proves neither -- "
              "CP/M 4.1 clears EINT after each command.")
    # Three distinct states, not two. verify_start_ok/verify_n are only
    # meaningful once at least one Verify has actually run -- reporting them
    # unconditionally reads as "Verify failed to start" when the truth is
    # "no Verify was ever requested".
    if verify_attempts == 0:
        print("  verify  : no Type I Verify has run this session "
              "(no seek carried the V bit) -- nothing to report")
    elif not verify_start_ok:
        print(f"  verify  : {verify_attempts} Verify pass(es); the last one "
              f"FAILED TO START its flux capture  seek_error={seek_error}")
    else:
        print(f"  verify  : {verify_attempts} Verify pass(es); last captured "
              f"{verify_n} flux bytes (~{verify_n / 43500:.2f} revolutions)  "
              f"seek_error={seek_error}")
    if seek_error:
        print("            ^ SEEK ERROR is latched: Verify found no ID field "
              "matching the target cylinder. The boot ROM stops here and "
              "reports a loading error WITHOUT ever issuing a READ.")
    if write_attempts:
        print(f"  WRITES  : {write_attempts} WRITE SECTOR command(s) this "
              f"session -- see `wrdiag` for their outcomes")
    if lost_data_events:
        print(f"  LOST DATA: {lost_data_events} sector(s) DRQ'd but never "
              f"drained by the host")
    total = sec_ok + sec_crc_err
    if total or sec_rnf:
        print(f"  sectors : {sec_ok} delivered clean, {sec_crc_err} delivered "
              f"with a BAD data CRC, {sec_rnf} RNF")
        if sec_crc_err:
            print("            ^ every one of those handed the host corrupt "
                  "bytes. If a listing is garbled, this is why.")
        if last_id_cyl != 0xFF:
            print(f"            last delivered: ID cyl={last_id_cyl} "
                  f"head={last_id_head}  vs  Track reg={last_trk_reg} "
                  f"phys head={last_want_side}")
            print(f"            position chain: Track reg={last_trk_reg} -> "
                  f"head_target={last_head_target} -> drive believes "
                  f"{last_phys_track} -> disk says cyl={last_id_cyl}")
            if last_head_target != last_trk_reg:
                print("            ^ head_target != Track reg: core1 folded "
                      "the step commands wrong")
            elif last_phys_track != last_head_target:
                print("            ^ drive belief != head_target: core0 never "
                      "drove the head there")
            elif last_id_cyl != last_phys_track:
                print("            ^ every belief agrees and the DISK "
                      "disagrees: the head did not physically arrive -- a "
                      "dropped STEP pulse (goto_track assumes it arrived)")
        if sec_cyl_mismatch:
            print(f"            {sec_cyl_mismatch} sector(s) came from a "
                  f"cylinder the host did NOT ask for -- rejected with RNF "
                  f"(WD ID track compare). Nonzero means the head is not "
                  f"where the host believes it is.")
        elif total:
            print("            every delivered sector's ID cylinder matched "
                  "the Track register -- the host got the track it asked for")


def do_bustest(s):
    s.write(bytes([MZP_CMD_BUSTEST, 2]))
    s.flush()
    r = read_exact(s, 148)  # 144 from firmware before #13
    if len(r) < 144 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
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
    if len(r) >= 148:
        print(f"    read  served but the Z80 had already sampled: {u32(144)}"
              f"{'   <-- the host got wrong bytes (#13)' if u32(144) else ''}")
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
        print(f"bad reply: {hdr.hex()}{busy_hint(hdr)}")
        return
    count = hdr[2] | (hdr[3] << 8)
    n = hdr[4]
    payload = read_exact(s, n * 2)
    if len(payload) < n * 2:
        print(f"short payload: got {len(payload)} want {n*2}")
        return
    ops = payload[:n]
    raws = payload[n:n*2]
    read_exact(s, 32)  # the seen-bitmap; consumed here, analysed by `health`
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
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
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


def do_rdsoak(s, track, sector, side, n, legacy):
    """Read one sector n times through the firmware's LIVE Type II read window,
    each starting at a random rotational phase -- the way a host READ lands.
    legacy=1 reproduces the pre-fix fixed-transition-count window for A/B.
    A sector whose data field the window cuts off shows up as CRC errors (and
    some RNF); a correct window gives none on readable media."""
    import random
    ok = crc = rnf = nodata = 0
    caps, fluxes, rev = [], [], 0
    for k in range(n):
        delay = random.randrange(0, 220000)
        s.write(bytes([MZP_CMD_RDWIN, 10, track, sector, side, legacy & 1])
                + delay.to_bytes(4, "little"))
        s.flush()
        r = read_exact(s, 15)
        if len(r) < 15 or r[1] != 0:
            print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
            return
        flags = r[2]
        flux_n = int.from_bytes(r[3:7], "little")
        cap_us = int.from_bytes(r[7:11], "little")
        rev = int.from_bytes(r[11:15], "little") or rev
        caps.append(cap_us)
        fluxes.append(flux_n)
        if not flags & 1:
            rnf += 1
        elif not flags & 2:
            nodata += 1
        elif not flags & 4:
            crc += 1
        else:
            ok += 1
        if (k + 1) % 20 == 0 or k + 1 == n:
            print(f"  {k + 1:4d}/{n}: ok {ok}  CRC {crc}  RNF {rnf}  "
                  f"no-data {nodata}", flush=True)
    cap_avg = sum(caps) / len(caps)
    flux_avg = sum(fluxes) / len(fluxes)
    print(f"track {track} side {side} sector {sector}, "
          f"{'LEGACY 45700-transition' if legacy else 'timed'} window:")
    print(f"  window  : {cap_avg / 1000:.1f} ms avg "
          f"({min(caps) / 1000:.1f}..{max(caps) / 1000:.1f}), "
          f"{flux_avg:.0f} transitions avg")
    if rev:
        print(f"  spindle : {rev / 1000:.1f} ms/rev measured -> window = "
              f"{cap_avg / rev:.3f} rev, track holds ~"
              f"{flux_avg * rev / cap_avg:.0f} transitions/rev")
    print(f"  result  : {ok}/{n} ok, {crc} CRC ERROR, {rnf} RNF, "
          f"{nodata} ID-without-data")


def gen_pattern(mode, seed, n):
    """Mirror of the payload generator in MZP_CMD_FDCWRITE (gw_protocol.cpp).

    Kept in lockstep so the probe can verify a written sector byte for byte
    without shipping the payload over USB in either direction.
    """
    if mode == 0:
        return bytes((seed + k) & 0xFF for k in range(n))
    if mode == 1:
        return bytes([seed & 0xFF]) * n
    out = bytearray()
    lfsr = ((seed & 0xFF) << 8) | 0xA5
    for _ in range(n):
        for _b in range(8):
            bit = (lfsr ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1
            lfsr = (lfsr >> 1) | (bit << 15)
        out.append(lfsr & 0xFF)
    return bytes(out)


SPLICE_FLAGS = [
    (0x01, "wrote"),
    (0x02, "id_found"),
    (0x04, "CYL MISMATCH"),
    (0x08, "SIZE MISMATCH"),
    (0x10, "LATE (missed Gap 2)"),
    (0x20, "NO FLUX"),
]


def do_fdcwrite(s, track, sector, side, sizecode, mode, seed):
    # Bench (no MZ-800): device seeks, then finds the sector on the spinning
    # disk and splices a new data field into it. THIS WRITES TO THE DISK.
    s.write(bytes([MZP_CMD_FDCWRITE, 8, track, sector, side,
                   sizecode & 7, mode & 0xFF, seed & 0xFF]))
    s.flush()
    r = read_exact(s, 6)  # cmd, ACK, wrote, flags, size_lo, size_hi
    if len(r) < 6 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return False
    wrote, flags = bool(r[2]), r[3]
    size = r[4] | (r[5] << 8)
    names = [n for bit, n in SPLICE_FLAGS if flags & bit] or ["(nothing)"]
    print(f"track {track} sector {sector} side {side} size {size}: "
          f"{'WROTE' if wrote else 'not written'}  [{', '.join(names)}]")
    if not wrote and not (flags & 0x02):
        print("  the sector's ID never passed the head -- wrong track/side, "
              "or the disk is not the one you think it is")
    return wrote


def do_wrverify(s, track, sector, side, sizecode, mode, seed):
    """Write a generated pattern, read it back, compare byte for byte."""
    n = 128 << (sizecode & 7)
    if not do_fdcwrite(s, track, sector, side, sizecode, mode, seed):
        print("VERIFY SKIPPED (nothing was written)")
        return
    want = gen_pattern(mode, seed, n)
    s.write(bytes([MZP_CMD_FDCREAD, 6, track, sector, side,
                   effective_density(s)]))
    s.flush()
    hdr = read_exact(s, 5)
    if len(hdr) < 5 or hdr[1] != 0:
        print(f"read-back: bad reply {hdr.hex()}")
        return
    flags, size = hdr[2], hdr[3] | (hdr[4] << 8)
    got = read_exact(s, size) if size else b""
    if not (flags & 1):
        print("read-back: RNF -- the write may have damaged the ID field")
        return
    print(f"read-back: size={size} data_crc="
          f"{'ok' if flags & 4 else 'BAD'} deleted={bool(flags & 8)}")
    if size != n:
        print(f"VERIFY FAILED: read {size} bytes, wrote {n}")
        return
    bad = [k for k in range(n) if got[k] != want[k]]
    if not bad:
        print(f"VERIFY OK: {n}/{n} bytes match"
              + ("" if flags & 4 else "  (but the CRC on disk is BAD!)"))
    else:
        print(f"VERIFY FAILED: {len(bad)}/{n} bytes differ, "
              f"first at offset {bad[0]} "
              f"(want 0x{want[bad[0]]:02X}, got 0x{got[bad[0]]:02X})")


def do_wrdiag(s):
    s.write(bytes([MZP_CMD_WRDIAG, 2]))
    s.flush()
    r = read_exact(s, 252)
    if len(r) < 252 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return
    wr_flags, last = r[2], r[3]
    ids_seen = r[4] | (r[5] << 8)
    vals = [int.from_bytes(r[6 + 4 * k:10 + 4 * k], "little") for k in range(14)]
    (splice_us, attempts, refused, ok, rnf, cyl_mm, size_mm, late,
     lost, abandoned, short_armed, trk_ok, trk_fail, trk_refused) = vals
    ab_want = r[62] | (r[63] << 8)
    ab_got = r[64] | (r[65] << 8)
    ab_track, ab_side, ab_sector = r[66], r[67], r[68]
    why, trk_op, trk_armed = r[69], r[70], r[71]
    trk_pos = r[72] | (r[73] << 8)
    trk_last = r[74] | (r[75] << 8)
    print("WRITE SECTOR state:")
    print(f"  writing={bool(wr_flags & 1)} buffer_full={bool(wr_flags & 2)} "
          f"writes_enabled={bool(wr_flags & 4)} "
          f"write_protect={bool(wr_flags & 8)}")
    names = [n for bit, n in SPLICE_FLAGS if last & bit] or ["(no write yet)"]
    print(f"  last splice: {', '.join(names)}  "
          f"ids_seen={ids_seen} search={splice_us / 1000:.1f} ms")
    print("cumulative:")
    print(f"  commands={attempts}  written={ok}  refused(WP/read-only)="
          f"{refused}")
    print(f"  RNF={rnf}  of which wrong-cylinder={cyl_mm} wrong-size={size_mm} "
          f"too-late={late}")
    print(f"  LOST DATA (host stopped feeding mid-sector)={lost}")
    print(f"  abandoned (ended before the buffer filled)={abandoned}  "
          f"short-armed (quiet stream ended the sector)={short_armed}")
    if abandoned:
        print(f"    last: wanted {ab_want} bytes, host sent {ab_got} "
              f"(track {ab_track} side {ab_side} sector {ab_sector})")
        if ab_got and ab_want != ab_got:
            print("    ! want != got means the sector SIZE assumed for that "
                  "cylinder+side is wrong, so the buffer never filled and no "
                  "splice ran -- nothing was written and nothing else counted it")
    print("WRITE/READ TRACK:")
    opname = {0: "idle", 1: "READ TRACK", 2: "WRITE TRACK"}.get(trk_op, "?")
    print(f"  state={opname} armed={bool(trk_armed)} collected={trk_pos} "
          f"last_image={trk_last}  completed={trk_ok} failed={trk_fail}")
    whyname = {0: "", 1: "~WPT (disk is write-protected)",
               2: "writes disabled (`wren 1`)",
               3: "format gate off (`fmten 1`)"}
    print(f"  refused={trk_refused}"
          + (f"  reason: {whyname.get(why, '?')}" if trk_refused else ""))
    if trk_refused and why:
        print("  ! a refused WRITE TRACK leaves STATUS=0x40 with BUSY clear; a "
              "format routine that ignores bit 6 polls it forever and the "
              "machine looks frozen")
    if cyl_mm:
        print("  ! wrong-cylinder aborts mean the head was not where the host "
              "believed -- see `rddiag` for the position chain")
    if late:
        print("  ! 'too late' means the ID decoded after its Gap 2 had passed; "
              "the splice was skipped rather than risk the next ID field")
    # Failed-write ring: WHICH writes failed. Clustering on a sector number /
    # track / side = geometry mismatch; scattered = marginal decode.
    wrf_n = int.from_bytes(r[76:80], "little")
    if wrf_n:
        shown = min(wrf_n, 8)
        print(f"failed writes: {wrf_n} total; last {shown} "
              f"(oldest -> newest):")
        for e in range(shown):  # snapshot emits oldest -> newest in slots 0..n-1
            off = 80 + e * 10
            trk, sec, side, ids = r[off], r[off + 1], r[off + 2], r[off + 3]
            fl, ctx, t10 = r[off + 4], r[off + 5], r[off + 6]
            phys = r[off + 7]
            decl = ("decl=" + ("HD" if fl & 0x80 else "DD")) if fl & 0x40 \
                else "decl=none"
            core0 = (ctx >> 4) & 3
            print(f"    trk={trk:3d} side={side} sec={sec:2d}  "
                  f"ids_seen={ids:2d}  hunted {t10 * 10} ms  "
                  f"clk={'HD' if fl & 0x20 else 'DD'} {decl}  "
                  f"motor={'on' if ctx & 2 else 'OFF!'}"
                  f"{'  NO FLUX' if fl & 0x10 else ''}"
                  f"{'  late' if fl & 8 else ''}")
            print(f"        host drv={(ctx >> 2) & 3}  core0 drv="
                  f"{'none' if core0 == 3 else core0}  "
                  f"~DS0={'released' if ctx & 0x40 else 'asserted'} "
                  f"~DS1={'released' if ctx & 0x80 else 'asserted'}  "
                  f"belief={'?' if phys == 0xFE else phys}")
    # Track-write ring: every host WRITE TRACK, with the position belief and
    # the density decision it was made under. A format writes BLIND at the
    # believed position, so 'phys=?' here is a mislabeled track on the media.
    twr_n = int.from_bytes(r[160:164], "little")
    if twr_n:
        shown = min(twr_n, 8)
        print(f"track writes: {twr_n} total; last {shown} (oldest -> newest):")
        for e in range(shown):
            off = 164 + e * 11
            ds = r[off] | (r[off + 1] << 8)
            trk, side, phys, nsec, fl = (r[off + 2], r[off + 3], r[off + 4],
                                         r[off + 5], r[off + 6])
            census = r[off + 7] | (r[off + 8] << 8)
            length = r[off + 9] | (r[off + 10] << 8)
            dens = "HD" if fl & 1 else "DD"
            decl = ("decl=" + ("HD" if fl & 4 else "DD")) if fl & 2 \
                else "decl=none"
            print(f"    t={ds / 10:7.1f}s  trk={trk:3d} side={side} "
                  f"phys={'?' if phys == 0xFE else phys}  "
                  f"{nsec:2d} sectors ~{census}B  img={length}B  "
                  f"enc={dens} {decl} censusHD={bool(fl & 8)} "
                  f"{'OK' if fl & 0x10 else 'FAILED'}")
            if phys == 0xFE:
                print("      ^ position belief UNKNOWN at a blind track write "
                      "-- this track landed on an unverified cylinder")


FF_KINDS = {0: "(none)", 1: "RNF -- sector not on this track",
            2: "data CRC error", 3: "WRONG CYLINDER",
            4: "Verify/seek error"}


def do_health(s):
    """One-shot session sanity check. Run FIRST each session and after any
    strange failure. Exists because this project has repeatedly burned days
    chasing firmware ghosts that were really the cartridge edge connector: the
    D7 data line has failed twice from a marginal joint, and flaky fingers
    (cured by reseating / IPA) corrupt commands into shapes that look exactly
    like impossible firmware bugs. This is the passive half of the bus rule
    ('run busdump first'); the definitive active measure is still the bustest
    Z80 program."""
    warns = []

    # 1. Which firmware is this, really? (A test cycle was once spent against
    # the wrong build.)
    s.write(bytes([MZP_CMD_FWVER, 2]))
    s.flush()
    hdr = read_exact(s, 3)
    if len(hdr) == 3 and hdr[1] == 0:
        v = read_exact(s, hdr[2]).decode("ascii", "replace")
        print(f"firmware : {v}")
        if v.endswith("+dirty"):
            warns.append("firmware built from a dirty tree -- not a committed state")
    else:
        print("firmware : (predates fwver -- OLD build!)")
        warns.append("firmware predates fwver: flash a current build before "
                     "trusting any result")

    # 2. Bus line health from the passive counters.
    s.write(bytes([MZP_CMD_BUSDUMP, 2]))
    s.flush()
    r = read_exact(s, 78)
    if len(r) < 78 or r[1] != 0:
        print("busdump  : bad reply")
        warns.append("busdump unreadable")
        r = None
    if r:
        wr = [int.from_bytes(r[10 + 4 * k:14 + 4 * k], "little") for k in range(8)]
        rd = [int.from_bytes(r[42 + 4 * k:46 + 4 * k], "little") for k in range(8)]
        addr_low, addr_high, dat_low, dat_high = r[74], r[75], r[76], r[77]
        total = sum(wr) + sum(rd)
        print(f"bus      : {total} register accesses since power-up")
        if total == 0:
            print("           (machine has not talked yet -- bus checks skipped)")
        else:
            # ADDRESS bits 6/7 read constant-high BY DESIGN (every port we own,
            # 0xD8-0xDF, has both set) -- only bits 0-5 are meaningful here.
            for k in range(6):
                m = 1 << k
                if not (addr_low & m):
                    warns.append(f"ADDRESS bit C{k} never sampled low")
                if not (addr_high & m):
                    warns.append(f"ADDRESS bit C{k} never sampled high")
            # WRITE-DATA union: a line that never moved MAY simply never have
            # needed to -- but with thousands of varied writes that gets
            # unlikely, so grade by traffic.
            if sum(wr) > 500:
                for k in range(8):
                    m = 1 << k
                    if not (dat_low & m):
                        warns.append(f"DATA bit C{k} never written low "
                                     f"({sum(wr)} writes) -- the D7-class "
                                     f"stuck-high signature")
                    if not (dat_high & m):
                        warns.append(f"DATA bit C{k} never written high "
                                     f"({sum(wr)} writes)")

    # 3. Command-byte corruption, judged over EVERY distinct raw command byte
    # seen since power-up (a 256-bit map), not the last 16: a copy loop's
    # last 16 commands are two or three opcodes that share bits by
    # coincidence, and that version of this check cried 'stuck line' at a bus
    # that had just passed 365k loopback cycles clean. A stuck line turns
    # Type II/III commands into aliased Type I ones (0x94 arrived as 0x14 in
    # the documented D7 incident), so across a session's whole vocabulary --
    # boot ROM + CP/M issue dozens of distinct raw bytes -- a bit that NEVER
    # takes one of its values is the signature.
    s.write(bytes([MZP_CMD_FDCLOG, 2]))
    s.flush()
    hdr = read_exact(s, 5)
    if len(hdr) == 5 and hdr[1] == 0:
        count = hdr[2] | (hdr[3] << 8)
        n = hdr[4]
        read_exact(s, n * 2)
        bm = read_exact(s, 32)
        seen = [v for v in range(256)
                if len(bm) == 32 and (bm[v >> 3] >> (v & 7)) & 1]
        print(f"commands : {count} total; {len(seen)} distinct raw bytes seen")
        if len(seen) >= 8:
            u_and, u_or = 0xFF, 0x00
            for b in seen:
                u_and &= b
                u_or |= b
            for k in range(8):
                m = 1 << k
                if u_and & m:
                    warns.append(f"command-byte bit C{k} was 1 in ALL "
                                 f"{len(seen)} distinct commands -- "
                                 f"stuck-high suspect")
                if not (u_or & m):
                    warns.append(f"command-byte bit C{k} was 0 in ALL "
                                 f"{len(seen)} distinct commands -- "
                                 f"stuck-low suspect")
        else:
            print("           (fewer than 8 distinct commands so far -- "
                  "command-line check skipped)")

    # 3b. Reads served after the Z80 had already sampled (#13): the one
    # bus fault the other counters cannot see -- a wrong byte, CRC-clean.
    s.write(bytes([MZP_CMD_BUSTEST, 2]))
    s.flush()
    r = read_exact(s, 148)
    if len(r) >= 148 and r[1] == 0:
        late = int.from_bytes(r[144:148], "little")
        print(f"late rd  : {late} read(s) served after the Z80 had sampled")
        if late:
            warns.append(f"{late} read(s) reached the bus after the Z80 sampled "
                         f"it: the host got a wrong byte each time (#13)")

    # 4. Verdict.
    print()
    if warns:
        print(f"HEALTH: {len(warns)} warning(s)")
        for w in warns:
            print(f"  ! {w}")
        print("  -> before blaming firmware: power down, RESEAT the cartridge,")
        print("     clean the edge fingers with IPA, and re-run `health`.")
        print("     Definitive bus measure: the `bustest` Z80 test program.")
    else:
        print("HEALTH: no bus anomalies in the passive counters.")
        print("  (Contacts can still fail intermittently; if symptoms persist")
        print("   with clean health, the fault is likely real firmware/media.)")


def do_resetdiag(s, clear=False):
    """What the bus-RESET pin actually does, with no polarity assumed.

    "resets seen: 0" after pressing reset has at least three causes that the
    event counter cannot tell apart: the line never moves, it pulses the other
    way (active low, not high), or the pulse is shorter than the debounce. The
    edge counts and pulse lengths here separate them.
    """
    s.write(bytes([MZP_CMD_RESETDIAG, 3, 1 if clear else 0]))
    s.flush()
    r = read_exact(s, 35)
    if len(r) < 35 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return
    line = r[2]
    (samples, rise, fall, high_max, low_min, last_high, events,
     at_ms) = [int.from_bytes(r[3 + 4 * k:7 + 4 * k], "little") for k in range(8)]
    print(f"RESET pin (GPIO29) level now: {'HIGH' if line else 'low'}")
    print(f"  core1 polls      : {samples}")
    print(f"  edges seen       : {rise} rising, {fall} falling")
    print(f"  longest HIGH     : {high_max} us"
          + (f"  (most recent {last_high} us)" if last_high else ""))
    print(f"  shortest LOW     : {low_min} us")
    print(f"  resets acted on  : {events}"
          + (f"  (last at {at_ms / 1000:.1f} s)" if events else ""))
    print()
    if samples == 0:
        print("  core1 is not polling at all -- the listen loop is not running")
    elif rise == 0 and fall == 0:
        print("  The pin NEVER MOVED. Either the machine's reset does not reach")
        print("  GPIO29 on this board, or it was not pressed while this ran.")
        print("  If the level above is 'low' the net may simply be undriven --")
        print("  bus_init() pulls it down, so an unconnected pin reads low too.")
    elif rise and not events:
        print(f"  The pin DOES pulse high, but no assertion was confirmed: the")
        print(f"  longest high was {high_max} us against a 100 us debounce.")
        print("  Shorten FDC_RESET_CONFIRM_US to match what the machine emits.")
    elif fall and not rise:
        print("  Only FALLING edges: the line rests high and pulses LOW, i.e.")
        print("  it is active low and the polarity assumption is inverted.")


def do_firstfail(s, clear=False):
    """The first failure since power-up, held unaltered.

    Everything else here reports the LAST operation, which is the wrong one the
    moment a host retries -- the retry that succeeds erases the evidence. That
    is why the rare boot-time `FD:Loading error` has never been diagnosable.
    """
    s.write(bytes([MZP_CMD_FIRSTFAIL, 3, 1 if clear else 0]))
    s.flush()
    r = read_exact(s, 67)
    if len(r) < 67 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return
    valid, kind = r[2], r[3]
    count = int.from_bytes(r[4:8], "little")
    at_ms = int.from_bytes(r[8:12], "little")
    (want, trk, tgt, phys, side, seen_c, seen_h) = r[12:19]
    ff_drive, ff_motor, cap_ok = r[19], r[20], r[21]
    flux_n = int.from_bytes(r[22:26], "little")
    cap_us = int.from_bytes(r[26:30], "little")
    ids_seen = r[30]
    ids = list(r[31:51])  # FDC_IDS_N = 20
    resyncs = int.from_bytes(r[51:55], "little")
    rs_from, rs_to = r[55], r[56]
    reset_events = int.from_bytes(r[57:61], "little")
    reset_at_ms = int.from_bytes(r[61:65], "little")
    reset_line, reset_active = r[65], r[66]

    # Read this first. The bus-RESET handling assumes the line idles LOW; if a
    # running machine shows it high, that assumption is wrong for this board and
    # the poller is holding EINT cleared, silently killing interrupts.
    print(f"bus RESET line now: {'HIGH' if reset_line else 'low'}"
          + ("  <-- machine in reset, or the polarity assumption is WRONG"
             if reset_line else " (idle, as expected)"))
    print(f"bus resets seen:   {reset_events}"
          + (f"  (last at {reset_at_ms / 1000:.1f} s)" if reset_events else "")
          + ("  [currently asserted]" if reset_active else ""))
    print(f"failures since power-up: {count}")
    print(f"head-position resyncs:   {resyncs}"
          + (f"  (last: disk said {rs_from}, we wanted {rs_to})"
             if resyncs else ""))
    if not valid:
        print("no failure latched yet -- nothing has gone wrong since boot")
        return
    print(f"\nFIRST failure, at {at_ms / 1000:.1f} s after boot:")
    print(f"  kind      : {FF_KINDS.get(kind, kind)}")
    print(f"  wanted    : sector {want} on track-reg {trk}, side {side}")
    print(f"  position  : head_target={tgt} phys_track="
          f"{'?' if phys == 0xFE else phys}"
          + (f"  disk said cyl={seen_c} head={seen_h}"
             if seen_c != 0xFF else ""))
    print(f"  drive     : {'?' if ff_drive == 0xFF else ff_drive}"
          f"   motor: host={'on' if ff_motor & 1 else 'OFF'}"
          f" spindle={'applied' if ff_motor & 2 else 'NOT APPLIED'}")
    print(f"  capture   : {'started' if cap_ok else 'DID NOT START'}, "
          f"{flux_n} flux bytes in {cap_us / 1000:.1f} ms")
    print(f"  ids on trk: {ids[:ids_seen] if ids_seen else '(none decoded)'}")
    # The point of latching this is to separate the causes, so say which.
    if at_ms < 3000:
        print("  -> happened within 3 s of boot: suspect the spindle was not "
              "yet stable, not the decode path")
    if cap_ok and flux_n == 0:
        print("  -> capture started but produced no flux: no disk, or the "
              "motor was not running")
    if kind == 3:
        print("  -> the head was on the wrong cylinder; compare head_target "
              "vs phys_track vs what the disk said to place the blame")
    elif kind == 1 and ids_seen:
        print("  -> the track decoded fine, it simply does not carry that "
              "sector number")


def scan_geometry(s, track, side, dens=0):
    """Return [(sector_id, size_code), ...] actually recorded on a track.

    The soak cannot assume a geometry. An MZ-800 CP/M disk is 9x512 on its data
    tracks but 16x256 on the boot track (cyl 0, physical head 1), and asking to
    write 512 bytes into a 256-byte sector is correctly refused by the splice's
    size compare -- which looks like a write failure but is the safety check
    working. Read the ID fields and use what is really there.
    """
    s.write(bytes([MZP_CMD_SCAN, 5, track, side, dens]))
    s.flush()
    hdr = read_exact(s, 7)
    if len(hdr) < 7 or hdr[1] != 0:
        return []
    ns = hdr[2]
    out = []
    for _ in range(ns):
        rec = read_exact(s, 5)
        if len(rec) < 5:
            break
        c, h, r, n, flags = rec
        if flags & 1:  # ID CRC ok
            out.append((r, n & 7))
    return out


def do_wrsoak(s, tracks, sides, passes, sector, sizecode, start_seed):
    # Read back at the density the writes go down at. This was a hard-coded 0
    # (DD): an HD soak on a perfectly written disk reported 160/160 'bad CRC on
    # read-back' -- every sector decoded at the wrong clock -- and cost a
    # false alarm against the DMA capture rework.
    dens = effective_density(s)
    """Write/verify soak across many tracks and sides.

    16 writes on one outer track is a demo. This is the test that answers the
    questions still open on the write path -- whether inner tracks need write
    precompensation, and whether the splice bias holds over a whole disk -- by
    doing enough of them, in the places most likely to fail, to make a rare
    fault show up. Every pass uses a different seed so a stale-buffer bug cannot
    masquerade as a successful write.
    """
    ok = bad = 0
    failures = []
    seed = start_seed

    # Probe each surface once and remember what is really on it, so a track
    # with a different format is exercised rather than reported as a failure.
    geom = {}
    for t in tracks:
        for sd in sides:
            ids = scan_geometry(s, t, sd, effective_density(s))
            if not ids:
                print(f"  track {t} side {sd}: no readable IDs -- skipping")
                continue
            # Prefer the requested sector; otherwise take the first one there.
            pick = next((x for x in ids if x[0] == sector), ids[0])
            geom[(t, sd)] = pick
            if pick[0] != sector or pick[1] != sizecode:
                print(f"  track {t} side {sd}: {len(ids)} sectors, using "
                      f"sector {pick[0]} @ {128 << pick[1]} bytes")
    if not geom:
        print("nothing readable to soak")
        return

    for p in range(passes):
        for t in tracks:
            for sd in sides:
                if (t, sd) not in geom:
                    continue
                sector, sizecode = geom[(t, sd)]
                seed = (seed + 37) & 0xFF
                n = 128 << (sizecode & 7)
                want = gen_pattern(2, seed, n)
                s.write(bytes([MZP_CMD_FDCWRITE, 8, t, sector, sd,
                               sizecode & 7, 2, seed]))
                s.flush()
                w = read_exact(s, 6)
                wrote = len(w) == 6 and w[1] == 0 and w[2] == 1
                flags = w[3] if len(w) == 6 else 0
                got = b""
                crc_ok = False
                if wrote:
                    s.write(bytes([MZP_CMD_FDCREAD, 6, t, sector, sd, dens]))
                    s.flush()
                    h = read_exact(s, 5)
                    if len(h) == 5 and h[1] == 0:
                        crc_ok = bool(h[2] & 4)
                        size = h[3] | (h[4] << 8)
                        got = read_exact(s, size) if size else b""
                good = wrote and crc_ok and got == want
                if good:
                    ok += 1
                else:
                    bad += 1
                    why = ("not written" if not wrote else
                           "bad CRC on read-back" if not crc_ok else
                           "data mismatch")
                    names = [nm for bit, nm in SPLICE_FLAGS if flags & bit]
                    failures.append((p, t, sd, why, ",".join(names)))
                    print(f"  FAIL pass {p} track {t} side {sd}: {why} "
                          f"[{','.join(names) or '-'}]")
        print(f"pass {p + 1}/{passes}: {ok} ok, {bad} failed")
    print(f"\nsoak done: {ok} ok, {bad} failed "
          f"({len(tracks)} tracks x {len(sides)} sides x {passes} passes)")
    if failures:
        print("failures (pass, track, side, why, splice flags):")
        for f in failures:
            print(f"  {f}")
        print("\nIf these cluster on high track numbers, that is the "
              "write-precompensation question answering itself. A SIZE "
              "MISMATCH is not a write fault -- it is the splice refusing to "
              "write a sector of the wrong length, before any flux.")
    else:
        print("No failures. Run `firstfail` and `wrdiag` for the "
              "controller's own view.")


FMT_RESULT = {0: "ok", 1: "WRITE PROTECTED", 2: "image too long",
              3: "write failed"}


def do_fdcformat(s, track, side, nsec, sizecode, interleave, fill):
    # !! DESTROYS THE TRACK. Builds a standard IBM System-34 image on-device and
    # writes it index-to-index through the same encoder the host's WRITE TRACK
    # command uses.
    s.write(bytes([MZP_CMD_FDCFORMAT, 8, track, side, nsec, sizecode & 7,
                   interleave, fill & 0xFF]))
    s.flush()
    r = read_exact(s, 5)
    if len(r) < 5 or r[1] != 0:
        print(f"bad/short reply: {r.hex()}{busy_hint(r)}")
        return False
    result = r[2]
    n = r[3] | (r[4] << 8)
    print(f"format track {track} side {side}: {nsec}x{128 << (sizecode & 7)} "
          f"interleave {interleave} -> {FMT_RESULT.get(result, result)} "
          f"({n} image bytes)")
    return result == 0


def do_fmtverify(s, track, side, nsec, sizecode, interleave, fill):
    """Format a track, then scan it back. The check that matters is that every
    sector formatted comes back with a valid ID CRC *and* a valid data CRC --
    the ID CRC proves the missing-clock sync marks and the F5/F7 handling are
    right, the data CRC proves the fill field and its CRC were laid down too."""
    if not do_fdcformat(s, track, side, nsec, sizecode, interleave, fill):
        return
    dens = effective_density(s)
    ids = scan_geometry(s, track, side, dens)
    print(f"read back: {len(ids)} sectors with valid ID CRCs")
    do_scan(s, track, side, dens)
    got = sorted({r for r, _n in ids})
    want = list(range(1, nsec + 1))
    missing = [x for x in want if x not in got]
    if missing:
        print(f"MISSING sectors: {missing}")
    else:
        print(f"all {nsec} sectors present and identifiable")


def do_readtrack(s, track, side, limit):
    s.write(bytes([MZP_CMD_READTRACK, 4, track, side]))
    s.flush()
    hdr = read_exact(s, 4)
    if len(hdr) < 4 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}{busy_hint(hdr)}")
        return
    n = hdr[2] | (hdr[3] << 8)
    data = read_exact(s, n) if n else b""
    print(f"track {track} side {side}: {n} raw bytes decoded\n")
    # Byte boundaries in the gaps are arbitrary (see mfm_read_track_raw), so the
    # useful view is where the address marks are, not a clean hex dump.
    show = data[:limit]
    for off in range(0, len(show), 16):
        row = show[off:off + 16]
        hexs = " ".join(f"{b:02X}" for b in row)
        txt = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print(f"  {off:05X}  {hexs:<47}  {txt}")
    if n > limit:
        print(f"  ... ({n - limit} more bytes)")
    marks = [k for k in range(len(data) - 1)
             if data[k] == 0xA1 and data[k + 1] in (0xFE, 0xFB, 0xF8)]
    print(f"\naddress marks found: {len(marks)} "
          f"({sum(1 for k in marks if data[k + 1] == 0xFE)} ID, "
          f"{sum(1 for k in marks if data[k + 1] in (0xFB, 0xF8))} data)")


def do_pintest(s):
    s.write(bytes([MZP_CMD_PINTEST, 2]))
    s.flush()
    r = read_exact(s, 4)
    if len(r) < 4 or r[1] != 0:
        print(f"bad reply: {r.hex()}{busy_hint(r)}")
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
            dens = (int(sys.argv[5]) if len(sys.argv) > 5
                    else effective_density(s))
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
                print(f"bad reply: {r.hex()}{busy_hint(r)}")
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
                print(f"bad reply: {r.hex()}{busy_hint(r)}")
            else:
                print("side select: " + ("INVERTED (ROM side 0 -> physical "
                                         "head 1)" if r[2] else
                                         "normal (ROM side 0 -> physical "
                                         "head 0)"))
        elif cmd == "iotrace":
            do_iotrace(s)
        elif cmd == "intmode":
            # no arg = query (0xFF), else 0 = off, 1 = INTRQ, 2 = INTRQ|DRQ
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 0xFF
            s.write(bytes([MZP_CMD_INTMODE, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            if len(r) < 3 or r[1] != 0:
                print(f"bad reply: {r.hex()}{busy_hint(r)}")
            else:
                names = {0: "OFF (~INT never driven)",
                         1: "follow EINT (port 0xDF bit0) -- correct/default",
                         2: "FORCE on, ignoring EINT (diagnostic; hangs a host "
                            "with no floppy ISR)"}
                print("~INT mode: " + names.get(r[2], f"? ({r[2]})"))
        elif cmd == "rdsoak":
            track = int(sys.argv[3])
            sector = int(sys.argv[4])
            side = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            n = int(sys.argv[6]) if len(sys.argv) > 6 else 100
            legacy = int(sys.argv[7]) if len(sys.argv) > 7 else 0
            do_rdsoak(s, track, sector, side, n, legacy)
        elif cmd == "fdcread":
            track = int(sys.argv[3])
            sector = int(sys.argv[4])
            side = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            dens = (int(sys.argv[6]) if len(sys.argv) > 6
                    else effective_density(s))
            do_fdcread(s, track, sector, side, dens)
        elif cmd == "wrburst":
            # !! WRITES THE DISK. Consecutive splices on one track with NO
            # read-back between them -- CP/M's cadence (the next WRITE lands
            # milliseconds after the previous WGATE release), which the soak
            # never reproduces because it reads back after every write.
            # Optional 5th arg = a DIFFERENT track to READ before each
            # splice. Write-only bursts and the soak both pass; CP/M reads its
            # source track then seeks to the destination and writes -- so the
            # read's capture (DMA ring start/stop) runs immediately before the
            # splice's own capture. This reproduces that read->seek->write
            # transition on the bench.
            track = int(sys.argv[3])
            side = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            nsec = int(sys.argv[5]) if len(sys.argv) > 5 else 18
            seed = int(sys.argv[6], 0) if len(sys.argv) > 6 else 0x5A
            rdtrack = int(sys.argv[7]) if len(sys.argv) > 7 else -1
            # 8th arg = drive to READ from before each splice (default: the
            # write drive). CP/M copies A->B: it reads the source from drive 0,
            # then SELECTS drive 1 and writes. Every earlier bench test stayed
            # on one drive; this alternates, so the drive-0-read -> drive-1-write
            # transition -- and any cross-drive RDATA bleed from the spinning
            # source drive -- is finally on the bench.
            rddrive = int(sys.argv[8]) if len(sys.argv) > 8 else -1
            wrdrive = None
            if rddrive >= 0:
                # remember the write drive so we can flip back to it
                s.write(bytes([MZP_CMD_DRVSEL, 3, 4]))  # query
                s.flush()
                q = read_exact(s, 5)
                wrdrive = q[2] if len(q) == 5 else 1
            fails = 0
            for sec in range(1, nsec + 1):
                if rdtrack >= 0:
                    if rddrive >= 0:
                        s.write(bytes([MZP_CMD_DRVSEL, 3, rddrive]))
                        s.flush(); read_exact(s, 5)
                    dens = effective_density(s)
                    s.write(bytes([MZP_CMD_FDCREAD, 6, rdtrack, 1, side, dens]))
                    s.flush()
                    hdr = read_exact(s, 5)
                    if len(hdr) == 5 and hdr[3] | (hdr[4] << 8):
                        read_exact(s, hdr[3] | (hdr[4] << 8))
                    if rddrive >= 0 and wrdrive is not None:
                        s.write(bytes([MZP_CMD_DRVSEL, 3, wrdrive]))
                        s.flush(); read_exact(s, 5)
                ok = do_fdcwrite(s, track, sec, side, 2, 2, (seed + sec) & 0xFF)
                if not ok:
                    fails += 1
            note = ""
            if rdtrack >= 0:
                note = f" (reading track {rdtrack}" + \
                    (f" from drive {rddrive}" if rddrive >= 0 else "") + \
                    " before each)"
            print(f"burst done: {nsec - fails}/{nsec} spliced on track {track} "
                  f"side {side}{note}; "
                  f"now `wrdiag` for the ring and `scan {track} {side}`")
        elif cmd in ("fdcwrite", "wrverify"):
            # THESE WRITE TO THE DISK. args: track sector [side] [sizecode]
            #                                [mode] [seed]
            track = int(sys.argv[3])
            sector = int(sys.argv[4])
            side = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            sizecode = int(sys.argv[6]) if len(sys.argv) > 6 else 2
            mode = int(sys.argv[7]) if len(sys.argv) > 7 else 2
            seed = int(sys.argv[8], 0) if len(sys.argv) > 8 else 0x5A
            if cmd == "fdcwrite":
                do_fdcwrite(s, track, sector, side, sizecode, mode, seed)
            else:
                do_wrverify(s, track, sector, side, sizecode, mode, seed)
        elif cmd == "wrdiag":
            do_wrdiag(s)
        elif cmd in ("fdcformat", "fmtverify"):
            # !! DESTROYS THE TRACK.
            # args: track [side] [n_sectors] [sizecode] [interleave] [fill]
            track = int(sys.argv[3])
            side = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            nsec = int(sys.argv[5]) if len(sys.argv) > 5 else 9
            sizecode = int(sys.argv[6]) if len(sys.argv) > 6 else 2
            il = int(sys.argv[7]) if len(sys.argv) > 7 else 2
            fill = int(sys.argv[8], 0) if len(sys.argv) > 8 else 0xE5
            if cmd == "fdcformat":
                do_fdcformat(s, track, side, nsec, sizecode, il, fill)
            else:
                do_fmtverify(s, track, side, nsec, sizecode, il, fill)
        elif cmd == "readtrack":
            track = int(sys.argv[3])
            side = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            limit = int(sys.argv[5]) if len(sys.argv) > 5 else 256
            do_readtrack(s, track, side, limit)
        elif cmd == "posfix":
            # no arg = query (2), else 0 = off, 1 = on
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_POSFIX, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            print("head-position self-correction: "
                  + ("ON" if len(r) == 3 and r[2] else
                     "OFF (seek path as it was before it was added)"))
        elif cmd == "drive":
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 4
            s.write(bytes([MZP_CMD_DRVSEL, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 5)
            if len(r) == 5:
                unit, ds0, ds1 = r[2], r[3], r[4]
                where = "~DS0" if unit == 0 else ("~DS1" if unit == 1 else "no select line on this board")
                print("selected unit: %d (%s)" % (unit, where))
                print("  (the lines below show the LAST APPLIED select -- the "
                      "new unit is selected by the next actuating command)")
                # ~DSn is active low: 0 = asserted.
                print("  ~DS0 (GPIO24) = %d  %s" % (ds0, "ASSERTED" if ds0 == 0 else "released"))
                print("  ~DS1 (GPIO25) = %d  %s" % (ds1, "ASSERTED" if ds1 == 0 else "released"))
                if ds0 == 0 and ds1 == 0:
                    print("  !! BOTH select lines asserted -- the old drive was never released.")
                elif unit < 2 and ds0 and ds1:
                    print("  note: neither line asserted (nothing has selected yet this session).")
                if unit > 1:
                    print("  STATUS reports NOT READY and nothing is actuated.")
        elif cmd == "drvstat":
            s.write(bytes([MZP_CMD_DRVSTAT, 2]))
            s.flush()
            r = read_exact(s, 11)
            if len(r) == 11:
                print("selected unit %d   ~DS0=%d %s   ~DS1=%d %s"
                      % (r[2], r[3], "ASSERTED" if r[3] == 0 else "released",
                         r[4], "ASSERTED" if r[4] == 0 else "released"))
                print("  unit  head  target  ready  spun  dens  selected")
                for u in range(2):
                    trk, tgt, fl = r[5 + u * 3], r[6 + u * 3], r[7 + u * 3]
                    print("   %d    %5s  %6d  %5s  %4s  %4s  %s"
                          % (u, "?" if trk == 0xFF else trk, tgt,
                             "yes" if fl & 1 else "NO",
                             "yes" if fl & 2 else "no",
                             "HD" if fl & 8 else "DD",
                             "yes" if fl & 4 else ""))
                h0, h1 = r[5], r[8]
                if h0 != 0xFF and h1 != 0xFF and h0 == h1:
                    print("  (both heads on the same cylinder -- park them apart to test independence)")
        elif cmd == "density":
            # 0 = force DD, 1 = force HD, 2 = auto (default), 3 = query.
            names = {"dd": 0, "hd": 1, "auto": 2, "detect": 3}
            val = names.get(sys.argv[3].lower(), None) if len(sys.argv) > 3 else 4
            if val is None:
                raise SystemExit("density takes dd, hd, auto or detect")
            s.write(bytes([MZP_CMD_DENSITY, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 11)
            if len(r) == 11:
                mode = {0: "FORCED DD", 1: "FORCED HD",
                        2: "auto (host 0xDE, else per-drive latch; no probing)",
                        3: "detect (auto + media probing for 0xDE-less hosts)"
                        }.get(r[2], "?")
                flips = r[5] | (r[6] << 8) | (r[7] << 16) | (r[8] << 24)
                print("density mode: %s" % mode)
                print("  host 0xDE declaration: %s"
                      % (("HD" if r[10] else "DD") if r[9]
                         else "none since power-up/reset"))
                print("  unit 0 latched: %s   unit 1 latched: %s"
                      % ("HD" if r[3] else "DD", "HD" if r[4] else "DD"))
                print("  auto-detect flips since power-up: %d" % flips)
                if flips > 4:
                    print("  ! many flips -- if this climbs on one disk, the "
                          "media is marginal at both clocks, not mixed-density")
        elif cmd == "health":
            do_health(s)
        elif cmd == "fwver":
            s.write(bytes([MZP_CMD_FWVER, 2]))
            s.flush()
            hdr = read_exact(s, 3)
            if len(hdr) == 3 and hdr[1] == 0:
                v = read_exact(s, hdr[2])
                print("firmware build: %s" % v.decode("ascii", "replace"))
            else:
                print("firmware predates the fwver command (pre-101d57f+)")
        elif cmd == "densel":
            # DENSEL pin override: 0/1 = force that level, auto = follow the
            # effective density (default); no arg = query.
            arg = sys.argv[3].lower() if len(sys.argv) > 3 else None
            val = {None: 3, "auto": 2, "0": 0, "1": 1}.get(arg)
            if val is None:
                raise SystemExit("densel takes 0, 1 or auto")
            s.write(bytes([MZP_CMD_DENSEL, 3, val]))
            s.flush()
            r = read_exact(s, 4)
            if len(r) == 4 and r[1] == 0:
                ov = "follows density" if r[2] == 0xFF else f"FORCED {r[2]}"
                lvl = "never driven" if r[3] == 0xFF else str(r[3])
                print(f"DENSEL override: {ov}   pin now: {lvl}  "
                      f"(1 = the level we call HD)")
        elif cmd == "idxbit":
            # Type I STATUS Index bit: 0 = never, 1 = only while the host has
            # declared HD on 0xDE (default), 2 = always; no arg = query.
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 3
            s.write(bytes([MZP_CMD_IDXBIT, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            names = {0: "hardwired 0 (pre-HD behaviour, everywhere)",
                     1: "live from ~INDEX ONLY while the host declares HD "
                        "(default: DD/boot ROM keep pre-HD semantics)",
                     2: "live from ~INDEX always (experiment)"}
            print("Type I STATUS Index bit (b1): "
                  + (names.get(r[2], "?") if len(r) == 3 else "bad reply"))
        elif cmd == "drvlock":
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_DRVLOCK, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            print("drive select: "
                  + ("PINNED to drive 0 -- port 0xDC's drive field ignored"
                     if len(r) == 3 and r[2] else "follows port 0xDC (multi-drive)"))
        elif cmd == "fmten":
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_FMTEN, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            print("host WRITE TRACK (format): "
                  + ("enabled (default)" if len(r) == 3 and r[2] else
                     "DISABLED -- a host format will be refused, and a format "
                     "routine that ignores the WP status bit will appear to "
                     "hang the machine"))
        elif cmd == "resetpoll":
            # no arg = query (2), else 0 = off, 1 = on
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_RESETPOLL, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            if len(r) < 3 or r[1] != 0:
                print(f"bad reply: {r.hex()}{busy_hint(r)}")
            else:
                print("bus RESET poller: " + ("ON" if r[2] else
                                              "OFF (core1 does not sample "
                                              "GPIO29 at all)"))
        elif cmd == "resetdiag":
            do_resetdiag(s, clear=(len(sys.argv) > 3
                                   and sys.argv[3] == "clear"))
        elif cmd == "firstfail":
            # optional "clear" to re-arm the latch for a fresh run
            do_firstfail(s, clear=(len(sys.argv) > 3
                                   and sys.argv[3] == "clear"))
        elif cmd == "wrsoak":
            # !! WRITES THE DISK, repeatedly, across the whole surface.
            # args: [passes] [sector] [t_start] [t_end] [t_step] [sides]
            # The range is REQUIRED (#16): with defaults, a bare `wrsoak`
            # overwrote 60 sectors including cylinder 0 on whichever drive a
            # previous `drive N` had left selected.
            if len(sys.argv) <= 6:
                print("wrsoak WRITES THE DISK: give it explicitly --\n"
                      "  wrsoak <passes> <sector> <t_start> <t_end> [t_step] [side]\n"
                      "  e.g. wrsoak 3 2 40 76 4   (scratch disk only)")
                sys.exit(2)
            passes = int(sys.argv[3])
            sector = int(sys.argv[4])
            t0 = int(sys.argv[5])
            t1 = int(sys.argv[6])
            step = int(sys.argv[7]) if len(sys.argv) > 7 else 4
            sides = ([int(sys.argv[8])] if len(sys.argv) > 8 else [0, 1])
            do_wrsoak(s, list(range(t0, t1 + 1, step)), sides, passes,
                      sector, 2, 0x11)
        elif cmd == "wren":
            # no arg = query (2), else 0 = read-only, 1 = writes enabled
            val = int(sys.argv[3]) if len(sys.argv) > 3 else 2
            s.write(bytes([MZP_CMD_WREN, 3, val & 0xFF]))
            s.flush()
            r = read_exact(s, 3)
            if len(r) < 3 or r[1] != 0:
                print(f"bad reply: {r.hex()}{busy_hint(r)}")
            else:
                print("MZ-800 writes: " + ("ENABLED" if r[2] else
                                           "disabled (WRITE PROTECT returned "
                                           "to the host)"))
        elif cmd == "streamscan":
            track = int(sys.argv[3])
            side = int(sys.argv[4])
            dens = (int(sys.argv[5]) if len(sys.argv) > 5
                    else effective_density(s))
            capture_ms = int(sys.argv[6]) if len(sys.argv) > 6 else 220
            do_streamscan(s, track, side, dens, capture_ms)
        elif cmd == "pindrive":
            # Bench C0..C7 pad-drive test (MZP_CMD_DRIVE). This was `drive`
            # too, which made it unreachable behind the unit-select branch
            # above -- and `pindrive 0 0` is the documented way to restore
            # normal bus operation after `gpioread` isolates the pads.
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
            dens = (int(sys.argv[4]) if len(sys.argv) > 4
                    else effective_density(s))
            do_scan(s, t, sd, dens)


if __name__ == "__main__":
    # A mistyped argument should say so, not dump a traceback. These commands
    # get run at a prompt while a machine is mid-boot, and the usual mistake is
    # a mis-ordered or duplicated positional -- which surfaces as an int()
    # ValueError several frames down and reads as a tool bug.
    try:
        main()
    except (ValueError, IndexError) as e:
        print(f"bad arguments: {e}\n")
        print(__doc__)
        sys.exit(2)
