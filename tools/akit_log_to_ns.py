"""Rewrite an AdvantageKit 27 alpha-4 (microsecond) WPILOG as alpha-7 (nanosecond) so it replays.

Record timestamps and the /Timestamp values are multiplied by 1000 and /Timestamp's unit metadata
becomes "nanoseconds". Everything else is copied byte for byte.

usage: python tools/akit_log_to_ns.py in.wpilog out.wpilog
"""
import struct
import sys

src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()
assert data[:6] == b"WPILOG"
extra_len = struct.unpack_from("<I", data, 8)[0]
out = bytearray(data[: 12 + extra_len])
pos = 12 + extra_len
n = len(data)
timestamp_id = None
starts = 0


def nbytes(v):
    return max(1, (v.bit_length() + 7) // 8)


def emit(eid, ts, payload):
    id_len, size_len, ts_len = nbytes(eid), nbytes(len(payload)), nbytes(ts)
    assert id_len <= 4 and size_len <= 4 and ts_len <= 8
    out.append((id_len - 1) | ((size_len - 1) << 2) | ((ts_len - 1) << 4))
    out.extend(eid.to_bytes(id_len, "little"))
    out.extend(len(payload).to_bytes(size_len, "little"))
    out.extend(ts.to_bytes(ts_len, "little"))
    out.extend(payload)


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
    payload = bytes(data[pos : pos + size])
    pos += size
    if eid == 0 and payload and payload[0] == 0:  # start record
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
        starts += 1
        if name == "/Timestamp":
            timestamp_id = entry
            meta = '{"source":"AdvantageKit","unit":"nanoseconds"}'
            nb, mb = name.encode(), meta.encode()
            payload = (
                payload[:5]
                + struct.pack("<I", len(nb)) + nb
                + struct.pack("<I", len(typ)) + typ
                + struct.pack("<I", len(mb)) + mb
            )
    elif eid == timestamp_id and size == 8:
        payload = struct.pack("<q", struct.unpack("<q", payload)[0] * 1000)
    emit(eid, ts * 1000, payload)

assert timestamp_id is not None, "no /Timestamp entry"
open(dst, "wb").write(out)
print(f"{starts} entries, {len(out) / 1e6:.1f} MB written to {dst}")
