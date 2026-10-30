#!/usr/bin/env python3
"""Cuts a window out of an AdvantageKit .wpilog so it still replays.

Usage:
    python tools/trim_wpilog.py in.wpilog out.wpilog START END

START and END are log timestamps in seconds (the time AdvantageScope shows). AdvantageKit logs a
value only when it changes, so the cut log opens with every entry's last value before START,
stamped on the window's first cycle. Replay then starts from the same inputs the robot had.
"""

import struct
import sys


def records(data):
    """Yields (entry id, timestamp us, payload, raw record bytes) for each record."""
    pos = 12 + struct.unpack_from("<I", data, 8)[0]
    n = len(data)
    while pos < n:
        start = pos
        hdr = data[pos]
        pos += 1
        id_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        if pos + id_len + size_len + ts_len > n:
            return
        eid = int.from_bytes(data[pos : pos + id_len], "little")
        pos += id_len
        size = int.from_bytes(data[pos : pos + size_len], "little")
        pos += size_len
        ts = int.from_bytes(data[pos : pos + ts_len], "little")
        pos += ts_len
        if pos + size > n:
            return  # cut off at the end of a log that was not closed cleanly
        yield eid, ts, data[pos : pos + size], data[start : pos + size]
        pos += size


def encode(eid, ts, payload):
    def le(v):
        return v.to_bytes(max(1, (v.bit_length() + 7) // 8), "little")

    i, s, t = le(eid), le(len(payload)), le(ts)
    hdr = (len(i) - 1) | (len(s) - 1) << 2 | (len(t) - 1) << 4
    return bytes([hdr]) + i + s + t + payload


def ticks_per_second(data):
    """/Timestamp value units per second (AdvantageKit 27.0.0-alpha-7 and later: ns, and marked)."""
    for eid, _, payload, _ in records(data):
        if eid == 0 and payload and payload[0] == 0:
            (ln,) = struct.unpack_from("<I", payload, 5)
            if payload[9 : 9 + ln] == b"/Timestamp":
                return 1e9 if b'"unit":"nanoseconds"' in payload else 1e6
    return 1e6


def trim(data, start_s, end_s):
    header = data[: 12 + struct.unpack_from("<I", data, 8)[0]]
    ticks = ticks_per_second(data)
    start_us, end_us = int(start_s * ticks), int(end_s * ticks)
    names = {}
    timestamp_id = None
    last = {}  # entry id to its latest payload before the window
    controls = []  # start and metadata records before the window
    out = [header]
    in_window = False
    for eid, ts, payload, raw in records(data):
        if eid == 0:
            if payload and payload[0] == 0:
                (entry,) = struct.unpack_from("<I", payload, 1)
                (ln,) = struct.unpack_from("<I", payload, 5)
                names[entry] = payload[9 : 9 + ln].decode()
                if names[entry] == "/Timestamp":
                    timestamp_id = entry
            if payload and payload[0] == 1:
                continue  # finish: keep every entry open
            (out if in_window else controls).append(raw)
            continue
        if eid == timestamp_id:
            (value,) = struct.unpack_from("<q", payload)
            if not in_window and value >= start_us:
                in_window = True
                out += controls
                out.append(raw)
                out += [encode(e, ts, p) for e, p in last.items()]
                continue
            if in_window and value >= end_us:
                break
        if in_window:
            out.append(raw)
        elif eid != timestamp_id:
            last[eid] = payload
    if not in_window:
        raise SystemExit("no cycles at or after START")
    return b"".join(out)


def main():
    if len(sys.argv) != 5:
        raise SystemExit(__doc__)
    src, dst, start_s, end_s = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    with open(src, "rb") as f:
        data = f.read()
    if data[:6] != b"WPILOG":
        raise SystemExit("not a wpilog")
    out = trim(data, start_s, end_s)
    with open(dst, "wb") as f:
        f.write(out)
    print(f"{dst}: {len(out) / 1e6:.1f} MB ({start_s:g} s to {end_s:g} s)")


if __name__ == "__main__":
    main()
