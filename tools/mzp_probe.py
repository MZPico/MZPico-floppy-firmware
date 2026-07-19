#!/usr/bin/env python3
"""MZPico Phase-2 probe — drive the firmware's on-device MFM sector layer.

Usage:
    python mzp_probe.py PORT scan    <track> <side> [dens]
    python mzp_probe.py PORT enctest <n_sectors> <size_code> [dens]
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


def main():
    if len(sys.argv) < 4:
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
        else:  # shorthand: PORT track side [dens]
            t, sd = int(sys.argv[2]), int(sys.argv[3])
            dens = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            do_scan(s, t, sd, dens)


if __name__ == "__main__":
    main()
