#!/usr/bin/env python3
"""Builds once, then runs the tests, scripted sims and replays in parallel; prints one summary.

Usage:
    python tools/check.py                          # tests + drive sim
    python tools/check.py --sims drive,shoot,auto  # more sims (auto is TBTB Left)
    python tools/check.py --log robotlogs/X.wpilog # also replay a robot log: its enabled part
                                                   # (10 s before the first enable to 5 s after
                                                   # the last disable); --full-log for all of it
    python tools/check.py --quick                  # unit tests only: skips simTest (~2 min)
    python tools/check.py --no-tests               # no Gradle tests at all

Each job's full output is in build/check/<job>.txt. Exits 1 if any job failed. Loop overruns
only warn: the jobs share the laptop's CPU.
"""

import argparse
import os
import shutil
import struct
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import replay_compare
import trim_wpilog

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "build" / "check"
WPILIB_JDK = Path("C:/Users/Public/wpilib/2027_alpha5/jdk")
SIM_SCRIPTS = {"drive": "drive", "shoot": "shoot", "chords": "chords", "auto": "auto:TBTB Left"}
JOB_TIMEOUT_S = 600
# Measured on TXMAN1 Q30: a cut replay's fused pose agrees with the full replay's 0.4 s in.
LEAD_IN_S = 10.0
WARM_UP_S = 2.0
TAIL_S = 5.0


def java_home():
    home = os.environ.get("JAVA_HOME")
    if home:
        return Path(home)
    if WPILIB_JDK.exists():
        return WPILIB_JDK
    sys.exit("Set JAVA_HOME to the WPILib 2027 JDK")


def gradle(args, log, env):
    exe = str(ROOT / ("gradlew.bat" if os.name == "nt" else "gradlew"))
    return run([exe, *args], log, env)


def run(cmd, log, env, timeout=JOB_TIMEOUT_S):
    with open(log, "w", encoding="utf-8", errors="replace") as f:
        try:
            p = subprocess.run(
                cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=timeout
            )
            return p.returncode
        except subprocess.TimeoutExpired:
            f.write(f"\nTIMED OUT after {timeout} s\n")
            return 124


def robot_cmd(env):
    jni = ROOT / "build" / "jni" / "release"
    env["PATH"] = str(jni) + os.pathsep + env.get("PATH", "")
    env["HALSIM_EXTENSIONS"] = ""
    java = java_home() / "bin" / ("java.exe" if os.name == "nt" else "java")
    return [
        str(java),
        f"-Djava.library.path={jni}",
        "--add-opens",
        "java.base/jdk.internal.vm=ALL-UNNAMED",
        "--add-opens",
        "java.base/java.lang=ALL-UNNAMED",
        "--enable-native-access=ALL-UNNAMED",
        "-jar",
        # Gradle names the jar after the project folder, which differs in a worktree.
        str(next((ROOT / "build" / "libs").glob("*-all.jar"))),
    ]


def tool(args, log, env):
    return run([sys.executable, str(ROOT / "tools" / args[0]), *args[1:]], log, env)


def job_tests(tasks, env):
    rc = gradle(tasks, OUT / "tests.txt", env)
    return rc == 0, " + ".join(tasks) if rc == 0 else "see tests.txt"


def job_sim(name, env):
    env = dict(env)
    folder = OUT / f"sim-{name}"
    shutil.rmtree(folder, ignore_errors=True)
    folder.mkdir(parents=True)
    env["FM_SIM_SCRIPT"] = SIM_SCRIPTS[name]
    env["FM_SIM_LOG_DIR"] = str(folder)
    console = OUT / f"sim-{name}.txt"
    rc = run(robot_cmd(env), console, env)
    logs = sorted(folder.glob("*.wpilog"))
    if rc != 0 or not logs:
        return False, f"exit {rc}, see {console.name}"
    args = ["ci_assert.py", str(logs[-1]), str(console)] + (["--auto"] if name == "auto" else [])
    report = OUT / f"sim-{name}-assert.txt"
    ok = tool(args, report, env) == 0
    fails = [l for l in report.read_text(encoding="utf-8").splitlines() if l.startswith("FAIL")]
    # Jobs share the laptop's CPU, so loop timing here says nothing about the robot; CI judges it.
    if not ok and fails and all(l.startswith("FAIL  loop") for l in fails):
        ok = True
        report.write_text(
            report.read_text(encoding="utf-8").replace("FAIL  loop", "WARN  loop (laptop busy)"),
            encoding="utf-8",
        )
    return ok, last_lines(report)


def enabled_window(data):
    """(first enable, last disable) in log seconds, or None if the robot never enabled."""
    enabled_id = None
    first = last = end = None
    enabled = False
    for eid, ts, payload, _ in trim_wpilog.records(data):
        if eid == 0:
            if payload and payload[0] == 0:
                (entry,) = struct.unpack_from("<I", payload, 1)
                (ln,) = struct.unpack_from("<I", payload, 5)
                if payload[9 : 9 + ln] == b"/DriverStation/Enabled":
                    enabled_id = entry
        elif eid == enabled_id:
            now = bool(payload[0])
            if now and not enabled:
                first = ts if first is None else first
                last = None
            elif enabled and not now:
                last = ts
            enabled = now
            end = ts
    if first is None:
        return None
    return first / 1e6, (last if last is not None else end) / 1e6


def job_replay(log, full_log, env):
    env = dict(env)
    folder = OUT / "replay"
    folder.mkdir(parents=True, exist_ok=True)
    src = folder / Path(log).name
    data = Path(log).read_bytes()
    window = None if full_log else enabled_window(data)
    if window:
        start, end = window[0] - LEAD_IN_S, window[1] + TAIL_S
        src.write_bytes(trim_wpilog.trim(data, max(start, 0.0), end))
    else:
        src.write_bytes(data)
    out = src.with_name(src.stem + "_sim.wpilog")
    out.unlink(missing_ok=True)
    env["FM_REPLAY"] = "1"
    env["AKIT_LOG_PATH"] = str(src)
    console = OUT / f"replay-{src.stem}.txt"
    rc = run(robot_cmd(env), console, env)
    if rc != 0 or not out.exists():
        return False, f"exit {rc}, see {console.name}"
    compare = OUT / f"replay-{src.stem}-compare.txt"
    # Odometry and the shadow tracks accumulate from their own start, so a cut log can't reproduce
    # them (TXMAN1 Q30: LL-Back's MT1 shadow off 0.24 m cut, exact in full); the fused pose can.
    keys = [
        k
        for k in replay_compare.DEFAULT_KEYS
        if not (window and (k.endswith("OdometryPose") or "/Shadow/" in k))
    ]
    after = ["--after", str(window[0] - LEAD_IN_S + WARM_UP_S)] if window else []
    ok = tool(["replay_compare.py", str(out), *after, *keys], compare, env) == 0
    scope = f"{start:.0f}-{end:.0f} s (enabled part)" if window else "whole log"
    # A difference is the effect of the change, not a failure.
    differs = "REPLAY DIFFERS" in compare.read_text(encoding="utf-8")
    verdict = "differs from" if differs else "matches"
    return ok, f"replayed {scope}, {verdict} the robot; details in {compare.name}"


def last_lines(path, n=4):
    lines = [l for l in path.read_text(encoding="utf-8", errors="replace").splitlines() if l.strip()]
    return "\n      ".join(lines[-n:])


def timed(name, fn, *args):
    start = time.monotonic()
    try:
        ok, detail = fn(*args)
    except Exception as e:  # a crashed job is a failed job, not a crashed check
        ok, detail = False, f"{type(e).__name__}: {e}"
    return name, ok, time.monotonic() - start, detail


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--sims", default="drive", help="comma list of " + ",".join(SIM_SCRIPTS))
    ap.add_argument("--log", action="append", default=[], help="robot .wpilog to replay")
    ap.add_argument("--no-tests", action="store_true")
    ap.add_argument("--quick", action="store_true", help="unit tests only, no simTest")
    ap.add_argument("--full-log", action="store_true", help="replay whole logs, not the enabled part")
    args = ap.parse_args()

    sims = [s for s in args.sims.split(",") if s]
    for s in sims:
        if s not in SIM_SCRIPTS:
            sys.exit(f"unknown sim {s}")
    OUT.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["JAVA_HOME"] = str(java_home())

    start = time.monotonic()
    if gradle(["shadowJar"], OUT / "build.txt", env) != 0:
        print(f"FAIL  build ({time.monotonic() - start:.0f} s), see build/check/build.txt")
        sys.exit(1)
    print(f"PASS  build ({time.monotonic() - start:.0f} s)", flush=True)

    jobs = [("sim " + s, job_sim, s, env) for s in sims]
    jobs += [("replay " + Path(l).name, job_replay, l, args.full_log, env) for l in args.log]
    if not args.no_tests:
        tasks = ["test"] if args.quick else ["test", "simTest"]
        jobs.append(("tests", job_tests, tasks, env))
    failed = False
    with ThreadPoolExecutor(max_workers=len(jobs) or 1) as pool:
        futures = [pool.submit(timed, name, fn, *a) for name, fn, *a in jobs]
        for f in futures:
            name, ok, secs, detail = f.result()
            failed |= not ok
            print(f"{'PASS' if ok else 'FAIL'}  {name} ({secs:.0f} s)\n      {detail}", flush=True)
    print(f"{'FAILED' if failed else 'ALL PASSED'} in {time.monotonic() - start:.0f} s; "
          "output in build/check/")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
