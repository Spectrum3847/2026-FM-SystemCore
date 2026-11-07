"""Rewrite an AdvantageKit 27 alpha-4 (microsecond) WPILOG as alpha-7 (nanosecond) so it replays.

AdvantageKit alpha-7 replays only logs whose /Timestamp is in nanoseconds. The /Timestamp values
are multiplied by 1000 and its unit becomes "nanoseconds"; everything else, record timestamps
included (microseconds in every WPILOG), is copied byte for byte. A log already in nanoseconds is
left as it is.

usage: python tools/akit_log_to_ns.py in.wpilog out.wpilog
       python tools/akit_log_to_ns.py --in-place a.wpilog [b.wpilog ...]
"""

import json
import struct
import sys


def _nbytes(v):
    return max(1, (v.bit_length() + 7) // 8)


def _records(data):
    """Yields (entry id, timestamp, payload) for each complete record after the header.

    A log cut off by a power loss ends partway through a record. That record is dropped: written
    back with the bytes it has, its size would say it is complete, and a replay would read it as a
    value of the wrong length and crash.
    """
    extra_len = struct.unpack_from("<I", data, 8)[0]
    pos = 12 + extra_len
    n = len(data)
    while pos < n:
        hdr = data[pos]
        pos += 1
        id_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        eid = int.from_bytes(data[pos : pos + id_len], "little")
        pos += id_len
        size = int.from_bytes(data[pos : pos + size_len], "little")
        pos += size_len
        ts = int.from_bytes(data[pos : pos + ts_len], "little")
        pos += ts_len
        if pos + size > n:
            return
        yield eid, ts, bytes(data[pos : pos + size])
        pos += size


def _start(payload):
    """(entry, name, type bytes, metadata) of a start control record, else None."""
    if not payload or payload[0] != 0:
        return None
    (entry,) = struct.unpack_from("<I", payload, 1)
    o = 5
    (ln,) = struct.unpack_from("<I", payload, o)
    name = payload[o + 4 : o + 4 + ln].decode()
    o += 4 + ln
    (lt,) = struct.unpack_from("<I", payload, o)
    typ = payload[o + 4 : o + 4 + lt]
    o += 4 + lt
    (lm,) = struct.unpack_from("<I", payload, o)
    meta = payload[o + 4 : o + 4 + lm].decode()
    return entry, name, typ, meta


def _timestamp_unit(meta):
    try:
        return json.loads(meta).get("unit")
    except ValueError:
        return None


def convert(data):
    """The converted log, or None if it is already in nanoseconds."""
    if data[:6] != b"WPILOG":
        raise ValueError("not a wpilog")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    out = bytearray(data[: 12 + extra_len])
    timestamp_id = None

    def emit(eid, ts, payload):
        id_len, size_len, ts_len = _nbytes(eid), _nbytes(len(payload)), _nbytes(ts)
        out.append((id_len - 1) | ((size_len - 1) << 2) | ((ts_len - 1) << 4))
        out.extend(eid.to_bytes(id_len, "little"))
        out.extend(len(payload).to_bytes(size_len, "little"))
        out.extend(ts.to_bytes(ts_len, "little"))
        out.extend(payload)

    for eid, ts, payload in _records(data):
        start = _start(payload) if eid == 0 else None
        if start and start[1] == "/Timestamp":
            entry, name, typ, meta = start
            if _timestamp_unit(meta) == "nanoseconds":
                return None
            timestamp_id = entry
            nb = name.encode()
            mb = b'{"source":"AdvantageKit","unit":"nanoseconds"}'
            payload = (
                payload[:5]
                + struct.pack("<I", len(nb))
                + nb
                + struct.pack("<I", len(typ))
                + typ
                + struct.pack("<I", len(mb))
                + mb
            )
        elif eid != 0 and eid == timestamp_id and len(payload) == 8:
            payload = struct.pack("<q", struct.unpack("<q", payload)[0] * 1000)
        emit(eid, ts, payload)
    if timestamp_id is None:
        raise ValueError("no /Timestamp entry")
    return bytes(out)


def main():
    args = sys.argv[1:]
    if args and args[0] == "--in-place":
        pairs = [(p, p) for p in args[1:]]
    elif len(args) == 2:
        pairs = [(args[0], args[1])]
    else:
        sys.exit(__doc__)
    for src, dst in pairs:
        data = open(src, "rb").read()
        converted = convert(data)
        if converted is None:
            print(f"{src}: already in nanoseconds")
            if dst != src:
                open(dst, "wb").write(data)
            continue
        open(dst, "wb").write(converted)
        print(f"{src}: converted to nanoseconds -> {dst}")


if __name__ == "__main__":
    main()
