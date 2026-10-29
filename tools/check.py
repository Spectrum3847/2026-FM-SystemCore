#!/usr/bin/env python3
"""Builds once, then runs the tests, scripted sims and replays in parallel; prints one summary.

Usage:
    python tools/check.py                          # tests + drive sim
    python tools/check.py --sims drive,shoot,auto  # more sims (auto is TBTB Left)
    python tools/check.py --log robotlogs/X.wpilog # also replay a robot log
    python tools/check.py --quick                  # unit tests only: skips simTest (~2 min)
    python tools/check.py --no-tests               # no Gradle tests at all

Each job's full output is in build/check/<job>.txt. Exits 1 if any job failed. Loop overruns
only warn: the jobs share the laptop's CPU.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "build" / "check"
WPILIB_JDK = Path("C:/Users/Public/wpilib/2027_alpha5/jdk")
SIM_SCRIPTS = {"drive": "drive", "shoot": "shoot", "chords": "chords", "auto": "auto:TBTB Left"}
JOB_TIMEOUT_S = 600


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
        str(ROOT / "build" / "libs" / "2026-FM-SystemCore-all.jar"),
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


def job_replay(log, env):
    env = dict(env)
    folder = OUT / "replay"
    folder.mkdir(parents=True, exist_ok=True)
    src = folder / Path(log).name
    shutil.copyfile(log, src)
    out = src.with_name(src.stem + "_sim.wpilog")
    out.unlink(missing_ok=True)
    env["FM_REPLAY"] = "1"
    env["AKIT_LOG_PATH"] = str(src)
    console = OUT / f"replay-{src.stem}.txt"
    rc = run(robot_cmd(env), console, env)
    if rc != 0 or not out.exists():
        return False, f"exit {rc}, see {console.name}"
    compare = OUT / f"replay-{src.stem}-compare.txt"
    ok = tool(["replay_compare.py", str(out)], compare, env) == 0
    return ok, last_lines(compare)


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
    jobs += [("replay " + Path(l).name, job_replay, l, env) for l in args.log]
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
