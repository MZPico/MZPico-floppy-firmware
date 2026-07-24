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
    # cmd, ack, last_raw[8], wr[8]*2, rd[8]*2, addr_low, addr_high, dat_low, dat_high
    hdr = read_exact(s, 46)
    if len(hdr) < 46 or hdr[1] != 0:
        print(f"bad/short reply: {hdr.hex()}")
        return
    last_raw = hdr[2:10]
    wr = [int.from_bytes(hdr[10 + 2 * k:12 + 2 * k], "little") for k in range(8)]
    rd = [int.from_bytes(hdr[26 + 2 * k:28 + 2 * k], "little") for k in range(8)]
    addr_low, addr_high, dat_low, dat_high = hdr[42], hdr[43], hdr[44], hdr[45]
    print("port  reads  writes  last_raw_write")
    for k in range(8):
        print(f"0x{0xD8 + k:02X}  {rd[k]:5d}  {wr[k]:6d}  0x{last_raw[k]:02X}")

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


STATUS_BITS = ["B", "Idx", "T00", "CRC", "Seek", "HL", "WP", "NRdy"]


def decode_cmd(c):
    hi = c >> 4
    names = {0x0: "RESTORE", 0x1: "SEEK", 0x2: "STEP", 0x3: "STEP",
             0x4: "STEP-IN", 0x5: "STEP-IN", 0x6: "STEP-OUT", 0x7: "STEP-OUT",
             0x8: "READ-SEC", 0x9: "READ-SEC(M)", 0xA: "WRITE-SEC",
             0xB: "WRITE-SEC(M)", 0xC: "READ-ADDR", 0xD: "FORCE-INT",
             0xE: "READ-TRACK", 0xF: "WRITE-TRACK"}
    return names.get(hi, "?")


def fdc_line(r):
    (status, track, sector, data, command, flags, drive,
     at0, phys, opseq, opdone) = r[2:13]
    sb = "".join(n + " " for k, n in enumerate(STATUS_BITS) if status & (1 << k))
    return (f"ST={status:02X}[{sb.strip()}] TRK={track:<2} SEC={sector:<2} "
            f"DAT={data:02X} CMD={command:02X}({decode_cmd(command)}) "
            f"mot={'1' if flags & 0x80 else '0'} sd={1 if flags & 4 else 0} "
            f"wp={1 if flags & 2 else 0} busy={1 if flags & 1 else 0} "
            f"t0={at0} phys={'?' if phys == 0xFF else phys} seek={opdone}/{opseq}")


def do_fdcdump(s, loop):
    def sample():
        s.write(bytes([MZP_CMD_FDCDUMP, 2]))
        s.flush()
        r = read_exact(s, 13)
        if len(r) < 13 or r[1] != 0:
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
    ops = read_exact(s, n)
    print(f"total commands: {count}; last {n} (oldest->newest):")
    for c in ops:
        print(f"  0x{c:02X}  {decode_cmd(c)}")


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
            r = read_exact(s, 13)
            if len(r) < 13 or r[1] != 0:
                print(f"bad reply: {r.hex()}")
            else:
                what = "RESTORE" if target == 0 else f"SEEK {target}"
                mdesc = "spin_motor+index" if mode == 1 else "motor_raw+delay"
                print(f"after {what} ({mdesc}, param {param}ms):\n  {fdc_line(r)}")
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
