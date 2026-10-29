#!/usr/bin/env python3
"""Tests for the log tools, on tiny .wpilog files written here.

    python -m unittest discover -s tools -p "test_*.py"

Each log is built in the test, with the key names robot code used before and after the 2026-10
renames, so the old-name fallbacks are exercised without committing a binary log.
"""

import os
import struct
import subprocess
import sys
import tempfile
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

import make_log_key_guide  # noqa: E402
import wpilog_summary  # noqa: E402


class Log:
    """Writes a WPILOG file: one start record per key, then data records."""

    def __init__(self):
        self.ids = {}
        self.out = bytearray(b"WPILOG" + struct.pack("<HI", 0x0100, 0))

    def _record(self, entry, t, payload):
        hdr = 3 | (3 << 2) | (7 << 4)  # 4-byte id and size, 8-byte timestamp
        self.out += bytes([hdr]) + struct.pack("<IIQ", entry, len(payload), int(t * 1e6)) + payload

    def _start(self, name, typ):
        if name not in self.ids:
            entry = len(self.ids) + 1
            self.ids[name] = (entry, typ)
            n, ty = name.encode(), typ.encode()
            payload = b"\x00" + struct.pack("<I", entry) + struct.pack("<I", len(n)) + n
            payload += struct.pack("<I", len(ty)) + ty + struct.pack("<I", 0)
            self._record(0, 0, payload)
        return self.ids[name][0]

    def put(self, name, t, value, typ=None):
        if typ is None:
            typ = {bool: "boolean", int: "int64", float: "double", str: "string"}[type(value)]
        if typ == "double":
            payload = struct.pack("<d", value)
        elif typ == "int64":
            payload = struct.pack("<q", value)
        elif typ == "boolean":
            payload = bytes([1 if value else 0])
        elif typ == "string":
            payload = value.encode()
        elif typ == "struct:Pose2d":
            payload = struct.pack("<3d", *value)
        else:
            raise ValueError(typ)
        self._record(self._start(name, typ), t, payload)
        return self

    def write(self, path):
        with open(path, "wb") as f:
            f.write(self.out)
        return path


def shooting_log(prefix, gate, shot_log):
    """Two loops of a launch, fed on the second, and one shot-log row."""
    log = Log()
    for t in (1.0, 1.02, 1.04, 2.0):
        log.put("/Timestamp", t, int(t * 1e6))
    log.put(prefix + gate + "FeedReason", 1.0, "Held")
    log.put(prefix + gate + "FeedReason", 1.02, "Ready")
    log.put(prefix + gate + "FeedReason", 2.0, "NotLaunching")
    log.put(prefix + gate + "BlockedReason", 1.0, "HoodNotReady")
    log.put(prefix + gate + "Volleys", 2.0, 1)
    log.put(prefix + shot_log + "Index", 1.02, 1)
    log.put(prefix + shot_log + "DistanceMeters", 1.02, 3.25)
    return log


def run(tool, *args):
    return subprocess.run(
        [sys.executable, os.path.join(TOOLS, tool), *args],
        capture_output=True,
        text=True,
        cwd=TOOLS,
    )


class LogToolsTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def path(self, name):
        return os.path.join(self.dir.name, name)

    def test_series_reads_new_names_and_falls_back_to_old(self):
        new = shooting_log("/RealOutputs/", "Shooting/Gate/", "Shooting/Log/").write(self.path("n"))
        old = shooting_log("/RealOutputs/", "Shot/", "ShotLog/").write(self.path("o"))
        for p in (new, old):
            records, types = wpilog_summary.read_wpilog(p)
            self.assertEqual([1], [v for _, v in wpilog_summary.series(records, types, "Shooting/Log/Index")])
            self.assertEqual(3, len(wpilog_summary.series(records, types, "Shooting/Gate/FeedReason")))

    def test_shot_summary_counts_the_launch_in_old_and_new_logs(self):
        for name, gate, rows in (("n", "Shooting/Gate/", "Shooting/Log/"), ("o", "Shot/", "ShotLog/")):
            p = shooting_log("/RealOutputs/", gate, rows).write(self.path(name))
            out = run("wpilog_summary.py", p).stdout
            self.assertIn("Shot gate: 1 launches", out, name)
            self.assertIn("fed after  0.02 s (Ready), held by HoodNotReady", out, name)
            self.assertIn("Shot log: 1 volleys", out, name)

    def test_shot_log_writes_a_row_from_old_and_new_prefixes(self):
        for name, gate, rows in (("n", "Shooting/Gate/", "Shooting/Log/"), ("o", "Shot/", "ShotLog/")):
            p = shooting_log("/RealOutputs/", gate, rows).write(self.path(name))
            lines = run("shot_log.py", p).stdout.strip().splitlines()
            self.assertEqual(2, len(lines), name)  # header and one row
            header, row = lines[0].split(","), lines[1].split(",")
            self.assertEqual("3.25", row[header.index("DistanceMeters")], name)

    def replay(self, name, gate, verdicts, x=0.0):
        log = Log()
        for i in range(10):
            t = 1.0 + i * 0.01
            log.put("/Timestamp", t, int(t * 1e6))
            log.put("/ReplayOutputs/Localization/FusedPose", t, (x, 0.0, 0.0), "struct:Pose2d")
            log.put("/ReplayOutputs/" + gate + "FeedReason", t, verdicts[i])
            log.put("/ReplayOutputs/" + gate + "BlockedReason", t, "None")
        return log.write(self.path(name))

    def test_replay_regression_compares_a_base_from_before_the_rename(self):
        ready = ["Ready"] * 10
        base = self.replay("base", "Shot/", ready)
        same = self.replay("same", "Shooting/Gate/", ready)
        result = run("replay_regression.py", base, same)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertIn("REPLAY UNCHANGED", result.stdout)

        moved = self.replay("moved", "Shooting/Gate/", ready, x=0.5)
        result = run("replay_regression.py", base, moved)
        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("FAIL  Localization/FusedPose", result.stdout)

    def test_key_guide_renders_runtime_parts_as_names(self):
        render = make_log_key_guide.render
        self.assertEqual("Mechanisms/<name>/Voltage", render('"Mechanisms/" + config.getName() + "/Voltage"'))
        self.assertEqual("CAN/<label>", render('"CAN/" + label'))
        self.assertEqual("Shooting/Calc/Target", render('"Shooting/Calc/Target"'))
        consts = make_log_key_guide.string_constants(
            'private static final String PREFIX = "Vision/Jetson/";')
        self.assertEqual("Vision/Jetson/Rewind/Label", render('PREFIX + "Rewind/Label"', consts))

    def test_key_guide_reads_unit_and_comment_beside_a_call(self):
        lines = [
            "    // How far the robot is from the hub.",
            '    Telemetry.log("Shooting/Calc/DistanceMeters", d, "meters");',
        ]
        text = "\n".join(lines)
        entries = list(make_log_key_guide.outputs("X.java", text, lines))
        self.assertEqual(1, len(entries))
        e = entries[0]
        self.assertEqual("/RealOutputs/Shooting/Calc/DistanceMeters", e["key"])
        self.assertEqual("meters", e["unit"])
        self.assertEqual("How far the robot is from the hub.", e["doc"])


if __name__ == "__main__":
    unittest.main()
