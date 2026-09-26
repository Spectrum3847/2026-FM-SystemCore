#!/usr/bin/env python3
"""Checks that an AdvantageKit replay reproduced the robot's outputs.

Usage:
    python tools/replay_compare.py logs/akit_X_sim.wpilog [key ...]

A replay log holds both the original /RealOutputs/... and the regenerated /ReplayOutputs/...
With no code changes they should match; after a change (a gate threshold, a std-dev tier) the
differences are the effect of the change. Default keys are the localization tracks.
"""

import math
import sys

from wpilog_summary import decode, read_wpilog

DEFAULT_KEYS = [
    "Localization/FusedPose",
    "Localization/OdometryPose",
    "Localization/Shadow/LL-Back/MT1/Pose",
    "Localization/Shadow/orin-front/Pose",
    "Localization/Shadow/Quest/Pose",
    "Vision/ChassisSource",
    "Swerve/SystemState",
]


def sampled(records, types, name):
    return [(t, decode(types[name], p)) for t, p in records.get(name, [])]


def value_at(series, t):
    """Latest value at or before t (AKit only logs changes)."""
    lo, hi = 0, len(series) - 1
    if not series or series[0][0] > t:
        return None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return series[lo][1]


def distance(a, b):
    if isinstance(a, tuple) and isinstance(b, tuple) and len(a) == 3:
        return math.hypot(a[0] - b[0], a[1] - b[1])
    return 0.0 if a == b else 1.0


def main():
    path = sys.argv[1]
    keys = sys.argv[2:] or DEFAULT_KEYS
    records, types = read_wpilog(path)
    worst_all = 0.0
    for key in keys:
        real = sampled(records, types, "/RealOutputs/" + key)
        replay = sampled(records, types, "/ReplayOutputs/" + key)
        if not real or not replay:
            print(f"{key}: missing ({len(real)} real, {len(replay)} replay)")
            continue
        worst = 0.0
        worst_t = None
        for t, v in replay:
            r = value_at(real, t)
            if r is None:
                continue
            d = distance(r, v)
            if d > worst:
                worst, worst_t = d, t
        worst_all = max(worst_all, worst)
        where = f" at t={worst_t:.2f}" if worst_t is not None else ""
        print(f"{key}: {len(replay)} replayed samples, worst difference {worst:.6f}{where}")
    print("REPLAY MATCHES" if worst_all < 1e-6 else "REPLAY DIFFERS")


if __name__ == "__main__":
    main()
