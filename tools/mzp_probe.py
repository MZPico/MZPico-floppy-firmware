#!/usr/bin/env python3
"""MZPico Phase-2 probe: ask the firmware to decode one track's MFM sectors
on-device and print the layout. Validates the on-device MFM decoder against a
real disk (compare with `gw read` of the same disk).

Usage:
    python mzp_probe.py COM5 <track> <side> [dens]
        dens: 0 = DD / 250 kbit/s (default), 1 = HD / 500 kbit/s

Needs pyserial:  pip install pyserial

The firmware self-selects the drive, spins the motor, seeks, captures ~220 ms of
raw flux, decodes on-device, and replies. Allow a few seconds.

Reply framing: 7-byte header [cmd, ack, ns, nflux(4 LE)] then ns * 5 bytes
{cyl, head, sec, n, flags}, flags bit0=id-CRC-ok bit1=data-CRC-ok
bit2=has-data bit3=deleted-mark.
"""
import sys
import serial  # pip install pyserial

MZP_CMD_SCAN = 0x80


def read_exact(s, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = s.read(n - len(buf))
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    port, track, side = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    dens = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    with serial.Serial(port, 115200, timeout=8) as s:
        s.reset_input_buffer()
        s.write(bytes([MZP_CMD_SCAN, 5, track, side, dens]))
        s.flush()

        hdr = read_exact(s, 7)
        if len(hdr) < 7:
            print(f"timeout / short reply ({len(hdr)} bytes): {hdr.hex()}")
            sys.exit(2)
        cmd, ack, ns = hdr[0], hdr[1], hdr[2]
        nflux = int.from_bytes(hdr[3:7], "little")
        if ack != 0:
            print(f"device NAK: cmd=0x{cmd:02x} ack={ack}")
            sys.exit(3)

        print(f"track {track} side {side} dens {dens}: "
              f"{ns} sectors, {nflux} flux transitions\n")
        print(f"{'#':>2}  {'C':>3} {'H':>3} {'R':>3} {'N':>2} {'size':>5}  "
              f"idCRC dataCRC mark")
        good = 0
        for k in range(ns):
            rec = read_exact(s, 5)
            if len(rec) < 5:
                print("  (short sector record)")
                break
            c, h, r, n, flags = rec
            id_ok = bool(flags & 1)
            data_ok = bool(flags & 2)
            has_data = bool(flags & 4)
            deleted = bool(flags & 8)
            size = 128 << (n & 7)
            mark = "deleted" if deleted else ("DAM" if has_data else "-")
            print(f"{k:>2}  {c:>3} {h:>3} {r:>3} {n:>2} {size:>5}  "
                  f"{'ok' if id_ok else 'BAD':>5} "
                  f"{('ok' if data_ok else 'BAD') if has_data else '-':>7}  {mark}")
            if id_ok and data_ok:
                good += 1
        print(f"\n{good}/{ns} sectors fully valid (ID+data CRC).")
        if ns == 0:
            print("No sectors decoded — if flux count looks sane, try the other "
                  "density (dens=1), or check the track/side.")


if __name__ == "__main__":
    main()
