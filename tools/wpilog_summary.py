#!/usr/bin/env python3
"""Summarise the localization comparison in an AdvantageKit .wpilog, no dependencies.

Usage:
    python tools/wpilog_summary.py logs/akit_XXXX.wpilog [--keys]

Prints, for every pose source, how many observations it produced, how many were accepted, the
most common rejection reasons, and (in simulation) its error against the truth pose -- both per
frame and for its shadow track. That is the "which source do we trust" table; AdvantageScope is
the place to look at the tracks themselves.

The WPILOG format is documented at
https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc
"""

import collections
import math
import statistics
import struct
import sys


def read_wpilog(path):
    data = open(path, "rb").read()
    if data[:6] != b"WPILOG":
        raise ValueError("not a wpilog")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    pos = 12 + extra_len
    entries = {}  # id -> (name, type)
    records = collections.defaultdict(list)  # name -> [(t, payload)]
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
        payload = data[pos : pos + size]
        pos += size
        if eid == 0:
            if payload and payload[0] == 0:
                (entry,) = struct.unpack_from("<I", payload, 1)
                o = 5
                (ln,) = struct.unpack_from("<I", payload, o)
                name = payload[o + 4 : o + 4 + ln].decode()
                o += 4 + ln
                (lt,) = struct.unpack_from("<I", payload, o)
                typ = payload[o + 4 : o + 4 + lt].decode()
                entries[entry] = (name, typ)
            continue
        if eid in entries:
            records[entries[eid][0]].append((ts / 1e6, payload))
    types = {name: typ for name, typ in entries.values()}
    return records, types


def decode(typ, payload):
    try:
        return _decode(typ, payload)
    except (struct.error, IndexError, UnicodeDecodeError):
        return None  # a record cut off at the end of a log that was not closed cleanly


def _decode(typ, payload):
    if typ == "double":
        return struct.unpack("<d", payload)[0]
    if typ == "boolean":
        return payload[0] != 0
    if typ == "int64":
        return struct.unpack("<q", payload)[0]
    if typ == "string":
        return payload.decode(errors="replace")
    if typ == "double[]":
        return list(struct.unpack("<%dd" % (len(payload) // 8), payload))
    if typ == "int64[]":
        return list(struct.unpack("<%dq" % (len(payload) // 8), payload))
    if typ == "string[]":
        (count,) = struct.unpack_from("<I", payload, 0)
        out, o = [], 4
        for _ in range(count):
            (ln,) = struct.unpack_from("<I", payload, o)
            out.append(payload[o + 4 : o + 4 + ln].decode(errors="replace"))
            o += 4 + ln
        return out
    if typ == "struct:Pose2d":
        return struct.unpack("<3d", payload)
    if typ == "struct:Pose2d[]":
        return [struct.unpack_from("<3d", payload, i) for i in range(0, len(payload), 24)]
    if typ == "struct:Pose3d":
        return struct.unpack("<7d", payload)  # x, y, z, qw, qx, qy, qz
    if typ == "struct:Pose3d[]":
        return [struct.unpack_from("<7d", payload, i) for i in range(0, len(payload), 56)]
    return None


def series(records, types, key):
    for prefix in ("/RealOutputs/", "/ReplayOutputs/", "/", ""):
        name = prefix + key if prefix else key
        if name in records:
            return [(t, decode(types[name], p)) for t, p in records[name]]
    return []


def main():
    path = sys.argv[1]
    records, types = read_wpilog(path)
    if "--keys" in sys.argv:
        for k in sorted(records):
            print(k, types[k], len(records[k]))
        return

    sources = sorted(
        {
            k.split("/Localization/Sources/")[1].rsplit("/", 1)[0]
            for k in records
            if "/Localization/Sources/" in k and k.endswith("/Verdicts")
        }
    )
    print(f"{'source':<16}{'frames':>8}{'accepted':>10}{'fused':>7}"
          f"{'frameErr(m)':>13}{'shadowErr(m)':>14}  top rejections")
    for s in sources:
        errs = series(records, types, f"Localization/Sources/{s}/ErrorVsTruthMeters")
        fused = series(records, types, f"Localization/Sources/{s}/FusedThisLoop")
        shadow = series(records, types, f"Localization/Shadow/{s}/ErrorVsTruthMeters")
        acc = series(records, types, f"Localization/Sources/{s}/AcceptedCount")
        rej = series(records, types, f"Localization/Sources/{s}/RejectedCount")
        acc = [a for a in acc if a[1] is not None]
        rej = [r for r in rej if r[1] is not None]
        accepted = acc[-1][1] if acc else 0
        total = accepted + (rej[-1][1] if rej else 0)
        prefix = f"/Localization/Sources/{s}/RejectionCounts/"
        rejections = collections.Counter()
        for k in records:
            if prefix in k:
                vals = series(records, types, k.split("Outputs/", 1)[-1])
                if vals:
                    rejections[k.split(prefix, 1)[1]] = vals[-1][1]
        frame_errs = [e for _, es in errs for e in (es or []) if e is not None and not math.isnan(e)]
        shadow_errs = [e for _, e in shadow if e is not None and not math.isnan(e)]
        # FusedThisLoop is only logged when it changes, so report whether it was ever true.
        fused_loops = "yes" if any(f for _, f in fused) else "no"
        fe = f"{statistics.median(frame_errs):.3f}" if frame_errs else "-"
        se = f"{statistics.mean(shadow_errs):.3f}" if shadow_errs else "-"
        top = ", ".join(f"{r} x{c}" for r, c in rejections.most_common(3))
        print(f"{s:<16}{total:>8}{accepted:>10}{fused_loops:>7}{fe:>13}{se:>14}  {top}")

    for key in ("FusedErrorVsTruthMeters", "OdometryErrorVsTruthMeters"):
        vals = [v for _, v in series(records, types, "Localization/" + key) if v is not None]
        if vals:
            print(f"{key}: mean {statistics.mean(vals):.3f} m, max {max(vals):.3f} m, final {vals[-1]:.3f} m")
    for key in ("Vision/PoseHeadingSeeded", "Vision/PoseSeedConfirmed", "Vision/ChassisSource"):
        vals = series(records, types, key)
        if vals:
            print(f"{key}: final {vals[-1][1]}")
    shot_summary(records, types)


def shot_summary(records, types):
    """The shot gate: how each launch began feeding, and how long it was held first."""
    reasons = series(records, types, "Shot/FeedReason")
    if not reasons:
        return
    # Each launch is a run of FeedReason values between NotLaunching ones. Outputs are written
    # only when they change, so a transition is a record.
    launches = []
    start = None
    first_feed = None
    for t, r in reasons:
        if r == "NotLaunching":
            if start is not None:
                launches.append((start, first_feed))
            start, first_feed = None, None
            continue
        if start is None:
            start = t
        if r != "Held" and first_feed is None:
            first_feed = (t, r)
    if start is not None:
        launches.append((start, first_feed))
    blocked = series(records, types, "Shot/BlockedReason")
    print(f"Shot gate: {len(launches)} launches")
    for s, feed in launches:
        end = feed[0] if feed else s + 1e9
        held_by = {b for t, b in blocked if s <= t < end}
        before = [b for t, b in blocked if t <= s]
        if before:
            held_by.add(before[-1])  # the value carried into the launch's first loop
        why = sorted(held_by - {"None"})
        if feed:
            print(f"  t={s:7.2f}s  fed after {feed[0] - s:5.2f} s ({feed[1]})"
                  + (f", held by {', '.join(why)}" if why else ""))
        else:
            print(f"  t={s:7.2f}s  never fed" + (f", held by {', '.join(why)}" if why else ""))
    for key in ("Shot/LaunchingLoops", "Shot/HeldLoops", "Shot/Volleys"):
        vals = series(records, types, key)
        if vals:
            print(f"  {key}: {vals[-1][1]}")
    for key in ("Sim/Launched", "Sim/Scored"):
        vals = series(records, types, key)
        if vals:
            print(f"  {key}: {vals[-1][1]}")
    rows = series(records, types, "ShotLog/Index")
    if rows:
        print(f"  Shot log: {len(rows)} volleys (python tools/shot_log.py <log> for the rows)")


if __name__ == "__main__":
    main()
