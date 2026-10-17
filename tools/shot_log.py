#!/usr/bin/env python3
"""Prints the shot log of an AdvantageKit .wpilog as CSV: one row per volley.

Usage:
    python tools/shot_log.py logs/akit_XXXX.wpilog [--replay] > shots.csv

Each row is written by ShotLog (src/main/java/frc/robot/subsystems/ShotLog.java) on the loop the
shot gate opened the feed, with an end row (ShotLog/End/*) when it closed. AdvantageKit writes a
value only when it changes, so a key that did not change since the last shot has no record at
this shot's time: every column is that key's last value at or before the row's ShotLog/Index
record. End columns are joined on ShotLog/End/Index == ShotLog/Index. Schema: docs/shot-log.md.

--replay reads /ReplayOutputs/ (what the replayed code wrote) instead of /RealOutputs/.
"""

import bisect
import csv
import math
import sys

from wpilog_summary import decode, read_wpilog

# Column order; any other ShotLog/ key found in the log is appended after these.
COLUMNS = [
    "Index", "TimestampSeconds", "MatchTimeSeconds", "Alliance", "Mode", "SuperState", "Kind",
    "SetShot", "FeedShot", "Model", "FeedReason", "Bypassed", "SecondsWaited", "DistanceMeters",
    "LookaheadDistanceMeters", "RadialVelocityMs", "TangentialVelocityMs", "RobotSpeedMs",
    "TimeOfFlightSeconds", "InRange", "WantedRPM", "ActualRPM", "WantedHoodDeg", "ActualHoodDeg",
    "HeadingErrorDeg", "AimToleranceDeg", "HoodTrimDeg", "AimTrimDeg", "PoseTrusted",
    "SecondsSinceFusedEstimate", "SeedConfirmed", "Pose",
]
END_COLUMNS = ["BurstSeconds", "FlywheelDips", "MinRPM"]


def last_at(series, times, t):
    """Latest value at or before t, or None."""
    i = bisect.bisect_right(times, t + 1e-9) - 1
    return series[i][1] if i >= 0 else None


def fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return "" if math.isnan(v) else f"{v:.4f}".rstrip("0").rstrip(".")
    if isinstance(v, tuple):  # Pose2d: x, y, heading
        return f"{v[0]:.3f} {v[1]:.3f} {math.degrees(v[2]):.1f}"
    return str(v)


def main():
    path = sys.argv[1]
    prefix = "/ReplayOutputs/ShotLog/" if "--replay" in sys.argv else "/RealOutputs/ShotLog/"
    records, types = read_wpilog(path)
    keys = {n[len(prefix):]: n for n in records if n.startswith(prefix)}
    if "Index" not in keys:
        print("no ShotLog/Index in this log (no volley was fed, or it predates the shot log)",
              file=sys.stderr)
        return
    data = {}
    for k, name in keys.items():
        s = [(t, decode(types[name], p)) for t, p in records[name]]
        data[k] = (s, [t for t, _ in s])

    extra = sorted(k for k in keys if k not in COLUMNS and not k.startswith("End/"))
    row_cols = COLUMNS + extra
    ends = {}
    if "End/Index" in data:
        for t, idx in data["End/Index"][0]:
            ends[idx] = t

    out = csv.writer(sys.stdout, lineterminator="\n")
    out.writerow(row_cols + END_COLUMNS)
    for t, idx in data["Index"][0]:
        row = []
        for c in row_cols:
            row.append(fmt(last_at(*data[c], t)) if c in data else "")
        end_t = ends.get(idx)
        for c in END_COLUMNS:
            key = "End/" + c
            row.append(fmt(last_at(*data[key], end_t)) if end_t is not None and key in data
                       else "")
        out.writerow(row)


if __name__ == "__main__":
    main()
