#!/usr/bin/env python3
"""Wheel slip in a match log: where the four modules disagree with each other and with the gyro.

Usage:
    python tools/slip_log.py LOG.wpilog [LOG.wpilog ...] [--half-base-m 0.276] [--window 50]

Module displacements (250 Hz odometry samples) are summed over windows of --window samples
(50 = 0.2 s) so encoder noise averages out, then, as TagEkf does per sample:
  - the chassis twist is solved from the four modules by least squares,
  - scrub = RMS of each module's displacement minus what that twist predicts (modules disagree),
  - yaw slip = |wheel rotation - gyro rotation| x module radius (wheels and gyro disagree).
Also reports the wheel/gyro yaw gain (geometry or scrub-in-turns check), slip by speed and by
acceleration, slip right after Pigeon hits, and which module carries it. Enabled time only.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wpilog_summary import decode, read_wpilog  # noqa: E402


def series(records, types, key):
    if key not in types:
        return []
    return [(t, decode(types[key], p)) for t, p in records.get(key, [])]


def enabled_spans(records, types):
    spans, start = [], None
    for t, v in series(records, types, "/DriverStation/Enabled"):
        if v and start is None:
            start = t
        elif not v and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, math.inf))
    return spans


def samples(records, types, spans):
    """(time, [4 distances], [4 angles], yaw) per odometry sample, enabled only."""
    ds = dict(series(records, types, "/Swerve/OdometryModuleDistances"))
    an = dict(series(records, types, "/Swerve/OdometryModuleAngles"))
    yw = dict(series(records, types, "/Swerve/OdometryYawRadians"))
    out = []
    for t, stamps in series(records, types, "/Swerve/OdometryTimestamps"):
        d, a, y = ds.get(t), an.get(t), yw.get(t)
        if not stamps or d is None or a is None or y is None:
            continue
        for k, st in enumerate(stamps):
            if 4 * k + 4 > len(d) or 4 * k + 4 > len(a) or k >= len(y):
                break
            if any(lo <= st <= hi for lo, hi in spans):
                out.append((st, d[4 * k : 4 * k + 4], a[4 * k : 4 * k + 4], y[k]))
    return out


def windows(smp, n):
    """Per window: (t, dt, per-module (dx, dy) in robot frame, gyro rotation)."""
    out = []
    i = 0
    while i + n < len(smp):
        sx = [0.0] * 4
        sy = [0.0] * 4
        ok = True
        for j in range(i + 1, i + n + 1):
            prev, cur = smp[j - 1], smp[j]
            if cur[0] - prev[0] > 0.05:  # gap (disable, CAN drop)
                ok = False
                break
            for m in range(4):
                dd = cur[1][m] - prev[1][m]
                if abs(dd) > 0.2:  # ponytail: dead-CAN zero; OdometryGuard owns that case
                    ok = False
                    break
                sx[m] += dd * math.cos(cur[2][m])
                sy[m] += dd * math.sin(cur[2][m])
            if not ok:
                break
        if ok:
            dg = math.remainder(smp[i + n][3] - smp[i][3], math.tau)
            out.append((smp[i][0], smp[i + n][0] - smp[i][0], sx, sy, dg))
        i += n
    return out


def analyze(path, half, n):
    records, types = read_wpilog(path)
    spans = enabled_spans(records, types)
    smp = samples(records, types, spans)
    if len(smp) <= n:
        print(f"{path.name}: no enabled odometry")
        return None
    mx = [half, half, -half, -half]
    my = [half, -half, half, -half]
    r = math.hypot(half, half)

    rows = []  # t, dt, translation, scrub, yaw slip, per-module residual, w, dg
    for t, dt, dx, dy, dg in windows(smp, n):
        vx, vy = sum(dx) / 4, sum(dy) / 4  # symmetric square: normal equations decouple
        w = sum(-my[i] * dx[i] + mx[i] * dy[i] for i in range(4)) / (4 * r * r)
        res = [math.hypot(dx[i] - (vx - w * my[i]), dy[i] - (vy + w * mx[i])) for i in range(4)]
        rows.append((t, dt, math.hypot(vx, vy), math.sqrt(sum(x * x for x in res) / 4),
                     abs(w - dg) * r, res, w, dg))

    drove = sum(x[2] for x in rows)
    scrub = sum(x[3] for x in rows)
    yaws = sum(x[4] for x in rows)
    ww = sum(x[6] * x[6] for x in rows)
    gain = sum(x[6] * x[7] for x in rows) / ww if ww else 0

    print(f"\n== {path.name}: {len(rows) * n / 250:.0f} s enabled odometry, drove {drove:.1f} m, "
          f"turned {sum(abs(x[7]) for x in rows) / math.tau:.1f} rev")
    print(f"  gyro/wheel yaw gain {gain:.3f}  (wheels see {100 * (1 / gain - 1) if gain else 0:+.1f}% more turn than "
          "the gyro: scrub in turns, or module radius off)")
    print(f"  scrub (modules disagree):  {scrub:6.2f} m = {100 * scrub / drove:4.1f}% of distance")
    print(f"  yaw slip (wheels vs gyro): {yaws:6.2f} m = {100 * yaws / drove:4.1f}%")
    print(f"  yaw slip after the gain fit: "
          f"{100 * sum(abs(x[6] * gain - x[7]) * r for x in rows) / drove:4.1f}%")
    per = [sum(x[5][i] for x in rows) for i in range(4)]
    print("  scrub by module FL FR BL BR: " + "  ".join(f"{100 * p / sum(per):.0f}%" for p in per))

    def bucket(label, key, edges):
        cells = []
        for lo, hi in zip(edges, edges[1:]):
            sel = [x for x in rows if lo <= key(x) < hi]
            d = sum(x[2] for x in sel)
            if d > 1:
                cells.append(f"{lo:g}-{hi:g}: {100 * sum(x[3] for x in sel) / d:.1f}/"
                             f"{100 * sum(x[4] for x in sel) / d:.1f}% ({d:.0f} m)")
        print(f"  by {label} (scrub/yaw %): " + "  ".join(cells))

    bucket("speed m/s", lambda x: x[2] / x[1], [0, 0.5, 1, 2, 3, 4, 99])
    bucket("turn rad/s", lambda x: abs(x[7]) / x[1], [0, 0.5, 1, 2, 4, 99])
    acc = {}
    for a, b in zip(rows, rows[1:]):
        acc[b[0]] = abs(b[2] / b[1] - a[2] / a[1]) / b[1]
    rows = [x for x in rows if x[0] in acc]
    bucket("accel m/s^2", lambda x: acc[x[0]], [0, 2, 4, 8, 99])

    ax = series(records, types, "/Swerve/AccelXG")
    ay = dict(series(records, types, "/Swerve/AccelYG"))
    hits = [t for t, v in ax if t in ay and any(lo <= t <= hi for lo, hi in spans)
            and v is not None and ay[t] is not None and math.hypot(v, ay[t]) > 1.5]
    if hits:
        hitset = set()
        for h in hits:
            for x in rows:
                if -0.1 <= x[0] - h <= 0.3:
                    hitset.add(x[0])
        sel = [x for x in rows if x[0] in hitset]
        print(f"  Pigeon |a_xy| > 1.5 g: {len(hits)} samples; windows within 0.3 s after hold "
              f"{100 * sum(x[3] + x[4] for x in sel) / (scrub + yaws):.0f}% of all slip in "
              f"{100 * len(sel) / len(rows):.0f}% of the time")
    worst = sorted(rows, key=lambda x: -(x[3] + x[4]))[:5]
    print("  worst windows (t: scrub mm, yaw mm): " + "; ".join(
        f"{x[0]:.1f}: {1000 * x[3]:.0f}, {1000 * x[4]:.0f}" for x in worst))
    return drove, scrub, yaws


def main():
    args = sys.argv[1:]
    opts = {"--half-base-m": 21.75 * 0.0254 / 2, "--window": 50}
    for k in list(opts):
        if k in args:
            i = args.index(k)
            opts[k] = type(opts[k])(args[i + 1])
            del args[i : i + 2]
    tot = [0, 0, 0]
    for a in args:
        out = analyze(Path(a), opts["--half-base-m"], opts["--window"])
        if out:
            tot = [x + y for x, y in zip(tot, out)]
    if len(args) > 1 and tot[0]:
        print(f"\nALL: drove {tot[0]:.0f} m, scrub {100 * tot[1] / tot[0]:.1f}%, "
              f"yaw slip {100 * tot[2] / tot[0]:.1f}%")


if __name__ == "__main__":
    main()
