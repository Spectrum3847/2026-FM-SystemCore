#!/usr/bin/env python3
"""Exports what TagEkf eats from a match log, for the replay tuner (TagEkfTuner).

Usage:
    python tools/export_tagekf_events.py LOG.wpilog [...]     # writes LOG.tagekf.txt beside each

One event per line, in log (loop) order, enabled time only (plus 2 s before the first enable for
the seed). The tuner sorts frames by capture time itself.
    O t gyroRad d0 d1 d2 d3 a0 a1 a2 a3     odometry sample (module distance m, angle rad; FL FR BL BR)
    D t                                       FailedDaqs rose this loop (Swerve's daqFailed)
    G t ok                                    GyroConnected changed
    T t pitchDeg rollDeg pitchRate rollRate axG ayG azG   tilt, tilt rates (deg/s), Pigeon acceleration, per loop
    F camera t n id txDeg tyDeg distM ...     one Limelight MT1 frame's raw fiducials
    R camera loopT hex                        one Orin PhotonPipelineResult, PhotonLib's bytes
    K camera fx fy cx cy d0 ... d7            an Orin camera's intrinsics (last logged), first in the file
    P camera t x y thetaRad tagCount          MT1 multi-tag pose (seed / lost-check reference)
Limelight fiducials and Orin results are logged from TXMAN1 Q30 onward.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wpilog_summary import decode, read_wpilog  # noqa: E402

CAMERAS = ("LL-Back", "LL-Left", "LL-Right")
ORINS = ("orin-left", "orin-right")
STRIDE = 5  # PoseSourceInputs.FIDUCIAL_STRIDE


def latest(series, t):
    """Value of a change-only series at time t (binary search), or None."""
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


def export(path):
    records, types = read_wpilog(path)

    def series(key):
        if key not in types:
            return []
        return [(t, v) for t, p in records.get(key, []) if (v := decode(types[key], p)) is not None]

    spans, start = [], None
    for t, v in series("/DriverStation/Enabled"):
        if v and start is None:
            start = t
        elif not v and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, math.inf))
    if not spans:
        print(f"{path.name}: never enabled")
        return
    lo = spans[0][0] - 2.0

    def keep(t):
        return t >= lo and (t < spans[0][0] or any(a <= t <= b for a, b in spans))

    events = []
    ts = dict(series("/Swerve/OdometryTimestamps"))
    yw = dict(series("/Swerve/OdometryYawRadians"))
    ds = dict(series("/Swerve/OdometryModuleDistances"))
    an = dict(series("/Swerve/OdometryModuleAngles"))
    for k, st in ts.items():
        y, d, a = yw.get(k), ds.get(k), an.get(k)
        if not st or not y or not d or not a or len(y) != len(st) or len(d) != 4 * len(st) or len(a) != len(d):
            continue
        for j, s in enumerate(st):
            if keep(k):
                vals = [y[j]] + d[4 * j : 4 * j + 4] + a[4 * j : 4 * j + 4]
                events.append((k, 1, "O " + " ".join(f"{v:.6f}" for v in [s] + vals)))
    for t, _ in series("/Swerve/FailedDaqs")[1:]:
        if keep(t):
            events.append((t, 0, f"D {t:.6f}"))
    for t, v in series("/Swerve/GyroConnected"):
        events.append((t, 0, f"G {t:.6f} {int(bool(v))}"))
    pitch = series("/Swerve/GyroPitchDegrees")
    roll = series("/Swerve/GyroRollDegrees")
    pr, rr = series("/Swerve/GyroPitchRateDegPerSec"), series("/Swerve/GyroRollRateDegPerSec")
    ax, ay, az = series("/Swerve/AccelXG"), series("/Swerve/AccelYG"), series("/Swerve/AccelZG")
    for k in ts:
        if keep(k):
            vals = [latest(s, k) for s in (pitch, roll, pr, rr, ax, ay, az)]
            if None not in vals:
                events.append((k, 0, f"T {k:.6f} " + " ".join(f"{v:.4f}" for v in vals)))
    for cam in CAMERAS:
        src = f"/Localization/Sources/{cam}/MT1/"
        stamps = dict(series(src + "Timestamps"))
        fids = dict(series(src + "Fiducials"))
        poses = dict(series(src + "Poses"))
        counts = dict(series(src + "TagCounts"))
        for k, st in stamps.items():
            if not st or not keep(k):
                continue
            f = fids.get(k)
            if f:
                tags = [f[i : i + 4] for i in range(0, len(f) - STRIDE + 1, STRIDE)]
                body = " ".join(f"{int(g[0])} {g[1]:.4f} {g[2]:.4f} {g[3]:.4f}" for g in tags)
                events.append((k, 2, f"F {cam} {st[-1]:.6f} {len(tags)} {body}"))
            ps, cs = poses.get(k), counts.get(k)
            if ps and cs:
                for p, c, tt in zip(ps, cs, st):
                    if c >= 2:
                        qw, qx, qy, qz = p[3:7]
                        th = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                        events.append((k, 2, f"P {cam} {tt:.6f} {p[0]:.4f} {p[1]:.4f} {th:.5f} {c}"))
    orin = 0
    for cam in ORINS:
        # AdvantageKit logs byte[][] as Results/length plus Results/<i>, each only when it changes:
        # a new record of Results/<i> while length > i is a new result.
        base = f"/Vision/Orin/{cam}/Results/"
        length = series(base + "length")
        i = 0
        while base + str(i) in types:
            for t, p in records.get(base + str(i), []):
                n = latest(length, t)
                if n is not None and n > i and keep(t) and p:
                    events.append((t, 2, f"R {cam} {t:.6f} {p.hex()}"))
                    orin += 1
            i += 1
    events.sort(key=lambda e: (e[0], e[1]))
    for cam in ORINS:
        k = [v for _, v in series(f"/Vision/Orin/{cam}/CameraMatrix") if v]
        d = [v for _, v in series(f"/Vision/Orin/{cam}/DistCoeffs") if v]
        if k:
            fx, fy, cx, cy = k[-1][0], k[-1][4], k[-1][2], k[-1][5]
            vals = [fx, fy, cx, cy] + (list(d[-1]) if d else [])
            events.insert(0, (0, 0, f"K {cam} " + " ".join(f"{v:.6f}" for v in vals)))
    out = path.with_suffix(".tagekf.txt")
    out.write_text("\n".join(e[2] for e in events) + "\n")
    n = sum(1 for e in events if e[2][0] == "F")
    print(f"{path.name}: {len(events)} events, {n} Limelight frames, {orin} Orin results -> {out.name}")


if __name__ == "__main__":
    for a in sys.argv[1:]:
        export(Path(a))
