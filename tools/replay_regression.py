#!/usr/bin/env python3
"""Compares two replays of the same log: the base branch's against a PR's.

Usage:
    python tools/replay_regression.py BASE_sim.wpilog PR_sim.wpilog

Both are replay logs of the fixture (src/test/resources/replay). Fails when the PR moves the
fused pose or changes the shot gate's verdicts past the limits below. Replaying the same code
twice gives identical outputs, so any difference is the PR's.
"""

import math
import sys

from replay_compare import value_at
from wpilog_summary import decode, read_wpilog

MAX_POSE_DIFF_M = 0.05
MAX_HEADING_DIFF_DEG = 2.0
# Loops a verdict may differ for: 10 loops is 0.1 s at FM's 100 Hz.
MAX_VERDICT_LOOPS = 10

POSE_KEYS = ["Localization/FusedPose"]
VERDICT_KEYS = ["Shot/FeedReason", "Shot/BlockedReason"]


def replayed(records, types, key):
    name = "/ReplayOutputs/" + key
    return [(t, decode(types[name], p)) for t, p in records.get(name, [])]


def loop_times(records):
    return [t for t, _ in records.get("/Timestamp", [])]


def compare_pose(base, pr, times):
    worst_m, worst_deg, worst_t = 0.0, 0.0, None
    for t in times:
        a, b = value_at(base, t), value_at(pr, t)
        if a is None or b is None:
            continue
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        h = abs(math.degrees(math.remainder(a[2] - b[2], 2 * math.pi)))
        if d > worst_m:
            worst_m, worst_t = d, t
        worst_deg = max(worst_deg, h)
    ok = worst_m <= MAX_POSE_DIFF_M and worst_deg <= MAX_HEADING_DIFF_DEG
    where = f" at t={worst_t:.2f}" if worst_t is not None else ""
    return ok, (f"worst {worst_m:.4f} m{where}, {worst_deg:.2f} deg "
                f"(limits {MAX_POSE_DIFF_M} m, {MAX_HEADING_DIFF_DEG} deg)")


def compare_verdict(base, pr, times):
    diffs = [t for t in times if value_at(base, t) != value_at(pr, t)]
    msg = f"{len(diffs)} loops differ (limit {MAX_VERDICT_LOOPS})"
    if diffs:
        t = diffs[0]
        msg += f", first at t={t:.2f}: {value_at(base, t)} -> {value_at(pr, t)}"
    return len(diffs) <= MAX_VERDICT_LOOPS, msg


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    base_records, base_types = read_wpilog(sys.argv[1])
    pr_records, pr_types = read_wpilog(sys.argv[2])
    times = loop_times(pr_records)
    failed = False
    for key in POSE_KEYS + VERDICT_KEYS:
        base = replayed(base_records, base_types, key)
        pr = replayed(pr_records, pr_types, key)
        if not base or not pr:
            ok, msg = False, f"missing ({len(base)} base, {len(pr)} PR samples)"
        elif key in POSE_KEYS:
            ok, msg = compare_pose(base, pr, times)
        else:
            ok, msg = compare_verdict(base, pr, times)
        print(f"{'PASS' if ok else 'FAIL'}  {key}: {msg}")
        failed |= not ok
    print("REPLAY UNCHANGED" if not failed else "REPLAY CHANGED")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
