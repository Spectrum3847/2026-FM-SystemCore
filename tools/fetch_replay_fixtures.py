#!/usr/bin/env python3
"""Builds the replay fixtures listed in src/test/resources/replay/fixtures.json.

Usage:
    python tools/fetch_replay_fixtures.py OUT_DIR

Each fixture is a window of a real match log in a GitHub release (the redacted copies: see the
release notes). Downloads each release asset once, checks the log against its SHA-256, cuts the
window with trim_wpilog.py and writes OUT_DIR/<name>.wpilog. A fixture already in OUT_DIR is
kept and one no longer listed is removed, so CI can cache OUT_DIR. Only the list is in git.
"""

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
import zipfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
FIXTURES = TOOLS.parent / "src" / "test" / "resources" / "replay" / "fixtures.json"


def download(url, dest):
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=300) as response, open(dest, "wb") as out:
                shutil.copyfileobj(response, out, 1 << 20)
            return
        except OSError as e:
            if attempt == 2:
                raise
            print(f"retrying {url}: {e}")
            time.sleep(5 * (attempt + 1))


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    spec = json.loads(FIXTURES.read_text(encoding="utf-8"))
    names = {f["name"] + ".wpilog" for f in spec["fixtures"]}
    for stale in out.glob("*.wpilog"):
        if stale.name not in names:
            print(f"removing {stale.name}: no longer listed")
            stale.unlink()

    todo = [f for f in spec["fixtures"] if not (out / (f["name"] + ".wpilog")).exists()]
    with tempfile.TemporaryDirectory() as tmp:
        archives = {}
        for f in todo:
            asset = f["asset"]
            if asset not in archives:
                path = Path(tmp) / asset
                print(f"downloading {asset}")
                download(spec["release"] + "/" + asset, path)
                archives[asset] = zipfile.ZipFile(path)
            full = Path(tmp) / "full.wpilog"
            with archives[asset].open(f["member"]) as src, open(full, "wb") as dst:
                shutil.copyfileobj(src, dst, 1 << 20)
            digest = sha256(full)
            if digest != f["sha256"]:
                raise SystemExit(f"{f['member']}: SHA-256 {digest}, expected {f['sha256']}")
            # Cut to a temporary name first, so a failed cut never lands in the cache.
            cut = Path(tmp) / "cut.wpilog"
            subprocess.run(
                [sys.executable, str(TOOLS / "trim_wpilog.py"), str(full), str(cut),
                 str(f["start"]), str(f["end"])],
                check=True,
            )
            shutil.move(cut, out / (f["name"] + ".wpilog"))
        for archive in archives.values():
            archive.close()

    for f in spec["fixtures"]:
        path = out / (f["name"] + ".wpilog")
        print(f"{path.name}: {path.stat().st_size / 1e6:.1f} MB, {f['start']} s to {f['end']} s")


if __name__ == "__main__":
    main()
