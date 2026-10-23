#!/usr/bin/env python3
"""Holds Error Prone warnings at or below a baseline.

Usage:
    ./gradlew compileJava --rerun 2>&1 | tee build/compile.txt
    python tools/errorprone_baseline.py build/compile.txt [--update]

Error Prone's ERROR checks already fail the compile. Its warnings are counted per file and check
and compared against tools/errorprone-baseline.txt: a count that goes up fails, so new code
stays clean while the existing warnings are worked down. --update rewrites the baseline (do it
when a count goes down, in the same PR).
"""

import collections
import os
import re
import sys

BASELINE = os.path.join(os.path.dirname(__file__), "errorprone-baseline.txt")
WARNING = re.compile(r"^(.*?\.java):\d+: warning: \[(\w+)\]")


def counts(text):
    found = collections.Counter()
    for line in text.splitlines():
        m = WARNING.match(line.strip())
        if m:
            path = m.group(1).replace("\\", "/")
            path = path[path.index("src/") :] if "src/" in path else path
            found[(path, m.group(2))] += 1
    return found


def read_baseline():
    base = collections.Counter()
    if os.path.exists(BASELINE):
        for line in open(BASELINE, encoding="utf-8"):
            if line.strip() and not line.startswith("#"):
                path, check, n = line.rsplit(None, 2)
                base[(path, check)] = int(n)
    return base


def write_baseline(found):
    with open(BASELINE, "w", encoding="utf-8", newline="\n") as f:
        f.write("# Error Prone warnings allowed per file and check. tools/errorprone_baseline.py\n")
        for (path, check), n in sorted(found.items()):
            f.write(f"{path} {check} {n}\n")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 1:
        raise SystemExit(__doc__)
    text = open(args[0], encoding="utf-8", errors="replace").read()
    if "BUILD SUCCESSFUL" not in text:
        raise SystemExit("compile did not succeed")
    found = counts(text)
    if "--update" in sys.argv:
        write_baseline(found)
        print(f"baseline: {sum(found.values())} warnings in {len(found)} file/check pairs")
        return
    base = read_baseline()
    worse = {k: n for k, n in found.items() if n > base.get(k, 0)}
    better = {k: n for k, n in base.items() if found.get(k, 0) < n}
    for (path, check), n in sorted(worse.items()):
        print(f"NEW  {path}: {check} x{n - base.get((path, check), 0)}")
    if better:
        print(f"{len(better)} file/check pairs improved: run with --update to lower the baseline")
    print(f"Error Prone: {sum(found.values())} warnings, baseline {sum(base.values())}")
    sys.exit(1 if worse else 0)


if __name__ == "__main__":
    main()
