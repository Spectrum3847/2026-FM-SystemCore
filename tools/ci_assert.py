#!/usr/bin/env python3
"""Fails a scripted sim run (FM_SIM_EXIT=1) that crashed, lost the pose or overran its loop.

Usage:
    python tools/ci_assert.py LOG CONSOLE [--auto]

LOG is the run's .wpilog, CONSOLE its captured console output. Checks:
  - the console shows no crash (an uncaught exception or the robot program quitting; errors WPILib
    reports with a stack trace and carries on from are not crashes), and the script ran to its end;
  - with --auto, no path the auto started stopped short of its end before autonomous ended
    (autos are built to run until the end of autonomous cuts them off);
  - the fused pose's error against the sim's true pose, once the vision seed is confirmed and
    outside the moment after the script places the robot;
  - the share of loops that overran by 25% (SystemLoadMonitor's definition).
"""

import math
import re
import statistics
import sys

from wpilog_summary import read_wpilog, series

# About twice the worst of drive, shoot and auto on ubuntu-latest: p95 0.073 m, max 0.090 m,
# overruns 1.3% (mostly startup loops).
MAX_FUSED_ERROR_P95_M = 0.15
MAX_FUSED_ERROR_M = 0.30
MAX_OVERRUN_SHARE = 0.05
OVERRUN_FACTOR = 1.25
PATH_END_TOLERANCE_M = 0.30
# The fused pose takes about 0.1 s to follow the robot when the script teleports it.
PLACE_SETTLE_S = 0.25

# What WPILib's RobotBase prints when robot code throws or startCompetition returns, what Java
# prints when another thread dies, and Robot's CrashTracker (only called on the way to a crash). Not
# stack trace lines alone: DriverStationErrors.reportError prints one for errors the program
# survives, such as Rotation2d's "x and y components of Rotation2d are zero".
CRASH = re.compile(
    r"Unhandled exception|quit unexpectedly|Unexpected return from startCompetition"
    r"|Exception in thread|CrashTracker"
)


def enabled_windows(records, types, mode=None):
    """[(start, end)] the robot was enabled (in `mode`, if given)."""
    modes = series(records, types, "DriverStation/RobotMode")
    windows, start = [], None
    for t, on in series(records, types, "DriverStation/Enabled"):
        if on and start is None:
            start = t
        elif not on and start is not None:
            windows.append((start, t))
            start = None
    if start is not None:
        windows.append((start, math.inf))
    if mode is None:
        return windows
    return [w for w in windows if value_at(modes, w[0] + 0.05) == mode]


def value_at(vals, t):
    before = [v for ts, v in vals if ts <= t]
    return before[-1] if before else None


def check_console(text):
    hits = [m.group(0).strip() for m in CRASH.finditer(text)]
    if hits:
        return False, f"{len(hits)} crash lines, first: {hits[0]}"
    return True, "no uncaught exceptions"


def check_ended(records, types):
    windows = enabled_windows(records, types)
    if not windows:
        return False, "never enabled"
    if windows[-1][1] == math.inf:
        return False, "log ends while enabled: the script did not finish"
    ts = series(records, types, "Timestamp")
    return True, f"script finished, log ends at {ts[-1][1] / 1e6:.1f} s"


def check_auto(records, types):
    autos = enabled_windows(records, types, "AUTONOMOUS")
    if not autos:
        return False, "autonomous never enabled"
    start, end = autos[0]
    current = series(records, types, "PathPlanner/CurrentPose")
    started, finished, cut = 0, 0, 0
    path = None
    for t, poses in series(records, types, "PathPlanner/ActivePath"):
        if t < start:
            continue
        # A path ends with an empty ActivePath, or when the next one is logged.
        if path is not None:
            here, goal = value_at(current, t), path[-1]
            miss = math.hypot(here[0] - goal[0], here[1] - goal[1]) if here else math.inf
            if t >= end:
                cut += 1
            elif miss > PATH_END_TOLERANCE_M:
                return False, f"path {started} stopped at t={t:.2f} s, {miss:.2f} m from its end"
            else:
                finished += 1
        path = poses or None
        started += bool(poses)
    if path is not None:
        return False, f"path {started} still running when the log ends"
    if started == 0:
        return False, "no path ran"
    return True, (f"{started} paths: {finished} finished, {cut} cut off by the end of "
                  f"autonomous ({end - start:.1f} s)")


def check_fused_error(records, types):
    seeded = [t for t, v in series(records, types, "Vision/PoseSeedConfirmed") if v]
    if not seeded:
        return False, "pose seed never confirmed"
    # Disabled after the script, the sim puts the robot back on the auto's start pose.
    windows = enabled_windows(records, types)
    end = windows[-1][1] if windows else math.inf
    placed = [t for t, v in series(records, types, "SimScript/PlaceRobot") if v]
    errs = [v for t, v in series(records, types, "Localization/FusedErrorVsTruthMeters")
            if seeded[0] <= t < end and v is not None and not math.isnan(v)
            and not any(p <= t < p + PLACE_SETTLE_S for p in placed)]
    if not errs:
        return False, "no FusedErrorVsTruthMeters after the seed"
    p95 = statistics.quantiles(errs, n=20)[-1] if len(errs) > 1 else errs[0]
    msg = (f"fused error after seed: mean {statistics.mean(errs):.3f} m, p95 {p95:.3f} m, "
           f"max {max(errs):.3f} m (limits {MAX_FUSED_ERROR_P95_M}, {MAX_FUSED_ERROR_M})")
    return p95 <= MAX_FUSED_ERROR_P95_M and max(errs) <= MAX_FUSED_ERROR_M, msg


def check_overruns(records, types):
    period = float(value_at(series(records, types, "RealMetadata/LoopPeriodSeconds"), math.inf)
                   or 0.02)
    # AdvantageKit records one cycle at the end of robot init. The gap from it to the first loop
    # is init's tail and the sim's startup (seconds on a CI runner), not a loop.
    ts = [v / 1e6 for _, v in series(records, types, "Timestamp")][1:]
    dts = [b - a for a, b in zip(ts, ts[1:])]
    if not dts:
        return False, "no loops"
    over = sum(dt > period * OVERRUN_FACTOR for dt in dts)
    share = over / len(dts)
    msg = (f"{over} of {len(dts)} loops over {period * OVERRUN_FACTOR * 1000:.1f} ms "
           f"({share:.1%}, limit {MAX_OVERRUN_SHARE:.0%}), worst {max(dts) * 1000:.0f} ms")
    return share <= MAX_OVERRUN_SHARE, msg


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 2:
        raise SystemExit(__doc__)
    log, console = args
    records, types = read_wpilog(log)
    with open(console, encoding="utf-8", errors="replace") as f:
        text = f.read()
    checks = [("exit", check_console(text)), ("script", check_ended(records, types))]
    if "--auto" in sys.argv:
        checks.append(("auto", check_auto(records, types)))
    checks.append(("pose", check_fused_error(records, types)))
    checks.append(("loop", check_overruns(records, types)))
    failed = False
    for name, (ok, msg) in checks:
        print(f"{'PASS' if ok else 'FAIL'}  {name:<7}{msg}")
        failed |= not ok
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
