#!/usr/bin/env python3
"""Generates FM's Elastic dashboard layout: src/main/deploy/elastic-layout.json.

    python tools/make_elastic_layout.py

Edit the tabs here, not the JSON. The robot reads the generated file at startup and publishes
exactly the outputs it references to NetworkTables (see Telemetry.addDashboardKeysFromElasticLayout),
so a widget added here gets its data with no other change, and nothing else is sent unless the
Telemetry/MirrorLogsToNT switch is on.

Modeled on the 2026 offseason bot's layout (Pre-Match, Match, Shooting, Power, Diagnostics), with
FM's mechanisms, and a Localization tab for the pose-source testbed in place of the turret tab.
The grid is 128 px; positions below are in grid cells.

Fits above the Driver Station: 14 columns by 6 rows (1792 x 768), the size the offseason bot's
layout settled on after its last two rows turned out to be hidden behind the DS window along the
bottom of the same screen ("Fit the dashboard above the Driver Station", 2026-09-05). The check at
the bottom asserts every widget is inside that grid and that none overlap, so nothing can drift
back below the fold. To use a seventh row, change ROWS -- nothing else.
"""

import json
import re
import os

CELL = 128
COLS, ROWS = 14, 6

GREEN, RED, GREY = 4283215696, 4294198070, 4288585374


def out(key):
    return "/AdvantageKit/RealOutputs/" + key


def meta(key):
    return "/AdvantageKit/RealMetadata/" + key


def widget(kind, title, c, r, w, h, **props):
    return {"title": title, "x": c * CELL, "y": r * CELL, "width": w * CELL, "height": h * CELL,
            "type": kind, "properties": props}


def text(title, topic, c, r, w=2, h=1, data_type="double"):
    return widget("Text Display", title, c, r, w, h, topic=topic, period=0.1, data_type=data_type,
                  show_submit_button=False)


def big(title, topic, c, r, w=4, h=1):
    return widget("Large Text Display", title, c, r, w, h, topic=topic, period=0.1,
                  data_type="string")


def light(title, topic, c, r, w=2, h=1, good_when_true=True):
    return widget("Boolean Box", title, c, r, w, h, topic=topic, period=0.1, data_type="boolean",
                  true_color=GREEN if good_when_true else RED,
                  false_color=RED if good_when_true else GREEN,
                  true_icon="None", false_icon="None")


def toggle(title, topic, c, r, w=2, h=1):
    return widget("Toggle Switch", title, c, r, w, h, topic=topic, period=0.1,
                  data_type="boolean")


def graph(title, topic, c, r, w, h, lo, hi, color=4294951936):
    return widget("Graph", title, c, r, w, h, topic=topic, period=0.05, data_type="double",
                  time_displayed=20, min_value=lo, max_value=hi, color=color, line_width=2)


def bar(title, topic, c, r, lo, hi, w=3, h=1):
    return widget("Number Bar", title, c, r, w, h, topic=topic, period=0.1, data_type="double",
                  min_value=lo, max_value=hi, divisions=5, inverted=False,
                  orientation="horizontal")


def field(c, r, w, h):
    return widget("Field", "Field2d", c, r, w, h, topic="/SmartDashboard/Field2d", period=0.1,
                  field_game="Rebuilt", robot_width=0.85, robot_length=0.85,
                  show_other_objects=True, show_trajectories=True, field_rotation=0,
                  robot_color=RED, trajectory_color=4294967295, show_robot_outside_widget=True)


def chooser(title, topic, c, r, w=7, h=1):
    return widget("Split Button Chooser", title, c, r, w, h, topic=topic, period=0.1,
                  sort_options=False)


def alerts(c, r, w, h):
    return widget("Alerts", "Alerts", c, r, w, h, topic="/SmartDashboard/Alerts", period=0.1)


def battery(c, r, w=3, h=1):
    return widget("Voltage View", "Battery", c, r, w, h, topic=out("BatteryLogger/BatteryVoltage"),
                  period=0.1, data_type="double", min_value=6, max_value=13, divisions=8,
                  inverted=False, orientation="horizontal")


def match_time(title, topic, c, r, w, h):
    return widget("Match Time", title, c, r, w, h, topic=topic, period=0.1, data_type="double",
                  time_display_mode="Minutes and Seconds", red_start_time=20,
                  yellow_start_time=30)


MECHANISMS = ["Hood", "Launcher", "FuelIntake", "IndexerBed", "IndexerTower",
              "IntakeExtension", "IntakeExtensionRight"]

SOURCES = ["LL-Back/MT1", "LL-Back/MT2", "LL-Left/MT1", "LL-Left/MT2", "LL-Right/MT1",
           "LL-Right/MT2", "SC0/MT1", "SC0/MT2", "SC1/MT1", "SC1/MT2", "Orin-TopLeft",
           "Orin-TopRight", "Quest"]

# The main CAN bus follows CanBuses.USE_CANIVORE: the CANivore, or else the drivetrain's SystemCore
# port (the busiest one). Rerun this script after flipping it.
_canbuses = open(os.path.join(os.path.dirname(__file__), "..", "src", "main", "java", "frc",
                              "spectrumLib", "hardware", "CanBuses.java"), encoding="utf-8").read()
USE_CANIVORE = re.search(r"USE_CANIVORE = (true|false);", _canbuses).group(1) == "true"
MAIN_BUS, MAIN_BUS_LABEL = ("CANivore", "CANivore") if USE_CANIVORE else ("SystemCoreCAN1",
                                                                          "Drive CAN 1")

pre_match = [
    widget("FMSInfo", "FMSInfo", 0, 0, 4, 1, topic="/FMSInfo", period=0.1),
    battery(4, 0),
    field(0, 1, 7, 3),
    chooser("Auto Chooser", "/SmartDashboard/Auto Chooser", 0, 4),
    chooser("Hub Model", "/SmartDashboard/Hub Model Chooser", 0, 5, 4),
    big("Super State", out("SuperStructure/CurrentSuperState"), 4, 5, 3),
    alerts(7, 0, 4, 2),
    light("Pose Seed Confirmed", out("Vision/PoseSeedConfirmed"), 11, 0, 3),
    text("Start Pose Err (m)", out("Auton/StartPoseErrorMeters"), 11, 1, 2),
    text("Hdg Err", out("Auton/StartHeadingErrorDeg"), 13, 1, 1),
    light(MAIN_BUS_LABEL, out(MAIN_BUS + "/StatusOK"), 7, 2),
    light("SystemCore CAN 0", out("SystemCoreCAN0/StatusOK"), 9, 2),
    light("CAN Config Budget Spent", out("CANConfig/BudgetExhausted"), 11, 2,
          good_when_true=False),
    text("Vision Age", out("Vision/SecondsSinceFusedEstimate"), 13, 2, 1),
    light("LL Back", out("Vision/LL-Back/Connected"), 7, 3),
    light("LL Left", out("Vision/LL-Left/Connected"), 9, 3),
    light("LL Right", out("Vision/LL-Right/Connected"), 11, 3),
    light("Auto File", out("Auton/AutoFileFound"), 13, 3, 1),
    light("Hood", out("Hood/MotorConnected"), 7, 4),
    light("Launcher", out("Launcher/MotorConnected"), 9, 4),
    light("Fuel Intake", out("FuelIntake/MotorConnected"), 11, 4),
    light("Auto Warm", out("Auton Warmed Up"), 13, 4, 1),
    light("Indexer Bed", out("IndexerBed/MotorConnected"), 7, 5),
    light("Indexer Tower", out("IndexerTower/MotorConnected"), 9, 5),
    light("Intake Ext L", out("IntakeExtension/MotorConnected"), 11, 5),
    light("Ext R", out("IntakeExtensionRight/MotorConnected"), 13, 5, 1),
]

match = [
    field(0, 0, 7, 3),
    match_time("Shift Time", out("Match Data/TimeLeftInShift"), 0, 3, 3, 2),
    battery(3, 3, 4),
    match_time("Match Time", out("Match Data/MatchTime"), 3, 4, 4, 1),
    big("Super State", out("SuperStructure/CurrentSuperState"), 0, 5, 7),
    alerts(7, 0, 4, 2),
    light("Pose Seed Confirmed", out("Vision/PoseSeedConfirmed"), 11, 0, 3),
    text("Vision Age (s)", out("Vision/SecondsSinceFusedEstimate"), 11, 1, 3),
    text("Distance (m)", out("ShotCalc/DistanceMeters"), 7, 2),
    text("Launcher RPM", out("Launcher/RPM"), 9, 2),
    text("Hood (deg)", out("Hood/PositionDegrees"), 11, 2, 3),
    text("Swerve State", out("Swerve/SystemState"), 7, 3, 3, data_type="string"),
    text("Intake State", out("FuelIntake/SystemState"), 10, 3, 4, data_type="string"),
    text("Hood Trim (deg)", out("ShotCalc/HoodAngleOffsetDegrees"), 7, 4),
    text("Aim Trim (deg)", out("ShotCalc/DriveAngleOffsetDegrees"), 9, 4),
    text("Intake Extension (rot)", out("IntakeExtension/Position"), 11, 4, 3),
    light("Shot Ready", out("Shot/Ready"), 7, 5, 3),
    big("Shot Blocked By", out("Shot/BlockedReason"), 10, 5, 4),
]

shooting = [
    graph("Launcher RPM", out("Launcher/RPM"), 0, 0, 7, 3, 0, 6000),
    graph("Hood Position (deg)", out("Hood/PositionDegrees"), 0, 3, 7, 2, 0, 60, 4283215696),
    text("Launcher Current (A)", out("Launcher/StatorCurrent"), 0, 5, 2),
    text("Hood Current (A)", out("Hood/StatorCurrent"), 2, 5, 2),
    text("Feed Reason", out("Shot/FeedReason"), 4, 5, 2, data_type="string"),
    text("Shots", out("ShotLog/Index"), 6, 5, 1, data_type="int"),
    text("Distance (m)", out("ShotCalc/DistanceMeters"), 7, 0, 3),
    text("Wanted Flywheel RPM", out("ShotCalc/FlywheelSpeedRPM"), 7, 1, 3),
    text("Wanted Hood (deg)", out("ShotCalc/HoodAngleDeg"), 7, 2, 3),
    text("Wanted Heading (deg)", out("ShotCalc/DriveAngleDeg"), 7, 3, 3),
    text("Time Of Flight (s)", out("ShotCalc/TimeOfFlight"), 7, 4, 3),
    text("Exit Speed (m/s)", out("ShotCalc/ExitSpeedMs"), 7, 5, 3),
    light("Feed Shot", out("ShotCalc/FeedShot"), 10, 0),
    text("Hood Trim (deg)", out("ShotCalc/HoodAngleOffsetDegrees"), 12, 0),
    text("Aim Trim (deg)", out("ShotCalc/DriveAngleOffsetDegrees"), 10, 1),
    text("Yaw Offset (deg)", out("ShotCalc/YawOffsetDeg"), 12, 1),
    big("Shot Blocked By", out("Shot/BlockedReason"), 10, 2, 2),
    big("Set Shot", out("ShotCalc/SetShot"), 12, 2, 2),
    chooser("Hub Model", "/SmartDashboard/Hub Model Chooser", 10, 3, 4),
    light("Shot Ready", out("Shot/Ready"), 10, 4),
    big("Super State", out("SuperStructure/CurrentSuperState"), 12, 4, 2),
    text("Launcher Temp (C)", out("Launcher/Temp"), 10, 5),
    text("Hood Temp (C)", out("Hood/Temp"), 12, 5),
]

# Localization: each source's fuse switch and last verdict. Three blocks of six rows (four cells
# each), then the seed and fusion status down the right-hand two columns.
localization = []
for i, s in enumerate(SOURCES):
    c = 4 * (i // 6)
    r = i % 6
    base = "Localization/Sources/" + s
    localization += [
        toggle(s, "/" + base + "/Enabled", c, r, 2),
        text("last verdict", out(base + "/LastVerdict"), c + 2, r, 2, data_type="string"),
    ]
localization += [
    light("Quest", out("Vision/Quest/Connected"), 8, 2),
    light("Orin TopLeft", out("Vision/Orin-TopLeft/Connected"), 10, 2),
    light("Orin TopRight", out("Vision/Orin-TopRight/Connected"), 8, 3),
    light("Jetson", out("Vision/Jetson/Connected"), 10, 3),
    light("LL Back", out("Vision/LL-Back/Connected"), 8, 4),
    light("LL Left", out("Vision/LL-Left/Connected"), 10, 4),
    light("LL Right", out("Vision/LL-Right/Connected"), 8, 5),
    text("Gross Heading Fixes", out("Vision/GrossHeadingCorrections"), 10, 5),
    light("Seed Confirmed", out("Vision/PoseSeedConfirmed"), 12, 0),
    toggle("Use MT2 After Seed", "/Vision/ChassisUseMT2", 12, 1),
    text("LLs Fusing", out("Vision/ChassisSource"), 12, 2, data_type="string"),
    text("Vision Age (s)", out("Vision/SecondsSinceFusedEstimate"), 12, 3),
    text("Seed Progress", out("Vision/SeedConfirmProgress"), 12, 4),
    big("Super State", out("SuperStructure/CurrentSuperState"), 12, 5, 2),
]

power = [
    graph("Battery Voltage", out("BatteryLogger/BatteryVoltage"), 0, 0, 7, 2, 6, 13),
    graph("Total Supply Current (A)", out("BatteryLogger/Current"), 0, 2, 7, 2, 0, 300,
          4294198070),
    bar(MAIN_BUS_LABEL + " Bus (%)", out(MAIN_BUS + "/BusUtilization"), 0, 4, 0, 100, 4),
    text("Energy Used (Wh)", out("BatteryLogger/Energy"), 4, 4, 3),
    light("Browned Out", out("SystemStats/BrownedOut"), 0, 5, 3, good_when_true=False),
    bar("IndexerTower (A)", out("BatteryLogger/Current/Mechanisms/IndexerTower"), 3, 5, 0, 120, 4),
]
for i, m in enumerate(["Launcher", "Hood", "Intake", "IntakeExtension", "IndexerBed",
                       "SwerveDrive"]):
    power.append(bar(m + " (A)", out("BatteryLogger/Current/Mechanisms/" + m), 7, i, 0, 120, 4))
for i, f in enumerate(["Launcher Top Right", "Launcher Bottom Left", "Launcher Bottom Right",
                       "Intake Right", "IndexerBed Follower 1", "IndexerTower Follower"]):
    power.append(bar(f + " (A)", out("Followers/" + f + "/SupplyCurrent"), 11, i, 0, 80, 3))

diagnostics = [
    graph("Loop Time (s, budget 0.010)", out("Scheduler/robotPeriodic"), 0, 0, 7, 2, 0, 0.02),
    text("Loop Mean (ms)", out("System/Loop/MeanPeriodMs"), 0, 2),
    bar("Loops Over Budget (%)", out("System/Loop/OverrunPercent"), 2, 2, 0, 100),
    text("Loop Max (ms)", out("System/Loop/MaxPeriodMs"), 5, 2),
    graph("Controller CPU (%)", out("System/CpuPercent"), 0, 3, 7, 2, 0, 100, 4294198070),
    text("Program CPU (%)", out("System/ProcessPercent"), 0, 5),
    text("Main Thread (%)", out("System/MainThreadPercent"), 2, 5),
    text("GC (ms/s)", out("System/Gc/MsPerSecond"), 4, 5),
    text("Heap MB", out("System/HeapUsedMB"), 6, 5, 1),
    alerts(7, 0, 4, 2),
    widget("Scheduler", "Scheduler", 11, 0, 3, 2, topic="/SmartDashboard/Scheduler", period=0.1),
    bar(MAIN_BUS_LABEL + " Bus (%)", out(MAIN_BUS + "/BusUtilization"), 7, 2, 0, 100, 4),
    text("CAN Config Spent (s)", out("CANConfig/BudgetSpentSeconds"), 11, 2, 3),
    text(MAIN_BUS_LABEL, out(MAIN_BUS + "/Status"), 7, 3, 2, data_type="string"),
    text("SC CAN 0", out("SystemCoreCAN0/Status"), 9, 3, 2, data_type="string"),
    text("CAN Failed Configs", out("CANConfig/FailedCalls"), 11, 3, 3),
    toggle("Mirror All Logs To NT", "/SmartDashboard/Telemetry/MirrorLogsToNT", 7, 4, 4),
    text("Mem Avail (MB)", out("System/MemAvailableMB"), 11, 4, 3),
    text("Git SHA", meta("GitSHA"), 7, 5, 4, data_type="string"),
    text("Git Dirty", meta("GitDirty"), 11, 5, 3, data_type="string"),
]

TABS = [("Pre-Match", pre_match), ("Match", match), ("Shooting", shooting),
        ("Localization", localization), ("Power", power), ("Diagnostics", diagnostics)]


def check(name, widgets):
    grid = {}
    for w in widgets:
        c0, r0 = w["x"] // CELL, w["y"] // CELL
        for c in range(c0, c0 + w["width"] // CELL):
            for r in range(r0, r0 + w["height"] // CELL):
                assert 0 <= c < COLS and 0 <= r < ROWS, f"{name}: {w['title']} off grid"
                assert (c, r) not in grid, f"{name}: {w['title']} overlaps {grid[(c, r)]}"
                grid[(c, r)] = w["title"]


def main():
    for name, widgets in TABS:
        check(name, widgets)
    layout = {"version": 1, "grid_size": CELL,
              "tabs": [{"name": n, "grid_layout": {"layouts": [], "containers": w}}
                       for n, w in TABS]}
    path = os.path.join(os.path.dirname(__file__), "..", "src", "main", "deploy",
                        "elastic-layout.json")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(layout, f, indent=2)
        f.write("\n")
    print(f"wrote {os.path.normpath(path)}: " + ", ".join(f"{n} ({len(w)})" for n, w in TABS))


if __name__ == "__main__":
    main()
