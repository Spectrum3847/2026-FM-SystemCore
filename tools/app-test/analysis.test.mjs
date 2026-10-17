/**
 * Tests for the pure analysis functions. Run with `node --test tools/app-test/*.test.mjs`.
 *
 * These cover the maths that produces numbers people will act on. A missed breaker trip or a
 * motor that dropped out unreported is worse than no tool at all. UI and transport are not covered.
 */
import { test } from "node:test";
import assert from "node:assert/strict";

import { parseWpilog } from "../../src/main/deploy/app/lib/wpilog.js";
import {
    LogModel,
    analysisKeep,
    timeAtOrAbove,
    fitInternalResistance,
    enabledWindows,
    clipToEnabled,
    withFollowers,
} from "../../src/main/deploy/app/lib/log-model.js";
import { simulateBreaker, tripTime, CB185_120 } from "../../src/main/deploy/app/lib/breaker.js";
import { staleTraces, alertEvents, connectionDropouts } from "../../src/main/deploy/app/lib/analyze-can.js";
import { LogWriter } from "./helpers/make-log.mjs";

const ENABLED = "/DriverStation/Enabled";

test("tripTime is infinite within rating and shrinks as current rises", () => {
    assert.equal(tripTime(CB185_120, 100), Infinity);
    assert.equal(tripTime(CB185_120, 120), Infinity);
    assert.ok(tripTime(CB185_120, 180) < tripTime(CB185_120, 144));
    assert.ok(tripTime(CB185_120, 600) < tripTime(CB185_120, 300));
});

test("breaker does not trip on current within its rating", () => {
    const samples = Array.from({ length: 5000 }, (_, i) => [i * 0.02, 90]);
    const r = simulateBreaker(samples);
    assert.equal(r.trips.length, 0);
    assert.equal(r.peakPct, 0);
});

test("breaker trips on a sustained heavy overload", () => {
    // 300 A trips in ~50 s per the curve; give it 80 s.
    const samples = Array.from({ length: 4000 }, (_, i) => [i * 0.02, 300]);
    const r = simulateBreaker(samples);
    assert.ok(r.trips.length >= 1, "expected at least one trip");
    assert.ok(r.trips[0] > 30 && r.trips[0] < 70, `trip at ${r.trips[0]}s should be near the 50 s curve point`);
});

test("breaker ignores gaps longer than a second so idle time is not treated as cooling", () => {
    const samples = [[0, 400], [600, 400], [600.02, 400]];
    const r = simulateBreaker(samples);
    assert.equal(r.trips.length, 0, "a 600 s gap must not be integrated as 600 s of overload");
});

test("timeAtOrAbove counts samples and seconds at a threshold", () => {
    const s = [[0, 10], [0.02, 90], [0.04, 90], [0.06, 10], [0.08, 95]];
    const r = timeAtOrAbove(s, 80);
    assert.equal(r.samples, 3);
    assert.equal(r.fraction, 3 / 5);
    assert.ok(r.seconds > 0.05 && r.seconds < 0.09);
});

test("timeAtOrAbove uses magnitude, since stator current is signed", () => {
    assert.equal(timeAtOrAbove([[0, -90]], 80).samples, 1);
});

test("fitInternalResistance recovers a known slope", () => {
    // V = 12.5 - I * 0.015
    const current = [];
    const voltage = [];
    for (let i = 0; i < 200; i++) {
        const t = i * 0.02;
        const amps = 20 + (i % 50) * 6;
        current.push([t, amps]);
        voltage.push([t, 12.5 - amps * 0.015]);
    }
    const r = fitInternalResistance(voltage, current);
    assert.ok(r, "expected a fit");
    assert.ok(Math.abs(r.ohms - 0.015) < 0.001, `got ${r.ohms}`);
    assert.ok(Math.abs(r.openCircuitVolts - 12.5) < 0.05, `got ${r.openCircuitVolts}`);
});

test("fitInternalResistance refuses to guess from too little data", () => {
    assert.equal(fitInternalResistance([[0, 12]], [[0, 5]]), null);
});

test("enabledWindows pairs DriverStation/Enabled transitions and closes an open window", () => {
    const log = { channel: (n) => (n === ENABLED ? [[1, true], [5, false], [9, true]] : []), lastTs: 12 };
    assert.deepEqual(enabledWindows(log), [[1, 5], [9, 12]]);
});

test("clipToEnabled drops disabled-time samples", () => {
    const s = [[0, 1], [2, 2], [6, 3], [10, 4]];
    assert.deepEqual(clipToEnabled(s, [[1, 7]]), [[2, 2], [6, 3]]);
});

test("wpilog parser reads a three-bit timestamp length", () => {
    // Header 0x40: 1-byte id, 1-byte size, 5-byte timestamp. A two-bit mask would read 1 byte and
    // desync the record after it.
    const w = new LogWriter();
    w.start("/RealOutputs/A", "double");
    const raw = (id, tsUs, value) => {
        const b = Buffer.alloc(1 + 1 + 1 + 5 + 8);
        b.writeUInt8(0x40, 0);
        b.writeUInt8(id, 1);
        b.writeUInt8(8, 2);
        b.writeUIntLE(tsUs, 3, 5);
        b.writeDoubleLE(value, 8);
        return b;
    };
    const t = 5_000_000_000; // past the 4-byte limit
    w.chunks.push(raw(1, t, 1), raw(1, t + 20_000, 2));
    const log = parseWpilog(w.buffer());
    assert.equal(log.error, null);
    assert.deepEqual(log.channel("/RealOutputs/A").map(([, v]) => v), [1, 2]);
});

test("parser rejects a file that is not a wpilog", () => {
    const buf = new TextEncoder().encode("NOTALOG_and_then_some_padding_bytes");
    assert.equal(parseWpilog(buf).error, "not a WPILOG file");
});

test("parser reports a file shorter than the header instead of throwing", () => {
    assert.equal(parseWpilog(new Uint8Array(4)).error, "not a WPILOG file");
});

test("a corrupt string[] record is skipped and the records after it still parse", () => {
    const w = new LogWriter();
    w.start("/RealOutputs/Names", "string[]");
    const bad = Buffer.alloc(8);
    bad.writeUInt32LE(0xffffffff, 0); // claims four billion strings in an 8-byte payload
    w.put("/RealOutputs/Names", "raw", 1, bad);
    w.put("/RealOutputs/After", "double", 2, 7);
    const log = parseWpilog(w.buffer());
    assert.equal(log.error, null);
    assert.deepEqual(log.channel("/RealOutputs/Names"), []);
    assert.deepEqual(log.channel("/RealOutputs/After"), [[2, 7]]);
});

/** An AdvantageKit-shaped log: one mechanism with two followers, one of which drops out. */
function fmLog() {
    const w = new LogWriter();
    w.put(ENABLED, "boolean", 0, true);
    w.put(ENABLED, "boolean", 10, false);
    w.series("/Mechanisms/Hood/StatorCurrentAmps", "double", { from: 0, to: 10, hz: 50, fn: () => 30 });
    w.series("/Mechanisms/Hood/SupplyCurrentAmps", "double", { from: 0, to: 10, hz: 50, fn: () => 10 });
    w.series("/Mechanisms/Hood/FollowerSupplyCurrentAmps", "double[]", { from: 0, to: 10, hz: 50, fn: () => [8, 6] });
    w.series("/Mechanisms/Hood/FollowerConnected", "boolean[]", { from: 0, to: 10, hz: 50, fn: (t) => [true, t < 4 || t > 6] });
    w.series("/Mechanisms/Hood/Connected", "boolean", { from: 0, to: 10, hz: 50, fn: (t) => t < 7 || t > 8 });
    w.series("/Runtime/BatteryVoltage", "double", { from: 0, to: 10, hz: 50, fn: () => 12.4 });
    w.series("/RealOutputs/BatteryLogger/BatteryVoltage", "double", { from: 0, to: 10, hz: 10, fn: () => 12.4 });
    w.series("/RealOutputs/BatteryLogger/Current", "double", { from: 0, to: 10, hz: 10, fn: () => 40 });
    w.put("/RealOutputs/SystemCoreCAN1/BusUtilization", "double", 1, 35);
    w.put("/RealOutputs/SystemCoreCAN3/BusUtilization", "double", 1, 20);
    w.put("/RealOutputs/Alerts/Warnings", "string[]", 2, ["Low battery"]);
    w.put("/RealOutputs/Alerts/Warnings", "string[]", 3, ["Low battery", "Hood follower"]);
    w.put("/RealOutputs/Alerts/Errors", "string[]", 5, ["CAN bus 3 down"]);
    w.put("/RealOutputs/Unread/Thing", "double", 1, 1);
    return parseWpilog(w.buffer(), { keep: analysisKeep });
}

test("LogModel discovers AdvantageKit mechanisms and adds follower current to supply", () => {
    const m = new LogModel(fmLog());
    assert.deepEqual(m.mechanisms.map((x) => x.name), ["Hood"]);
    const hood = m.mechanisms[0];
    assert.equal(hood.followerCount, 2);
    assert.equal(hood.supply[0][1], 24);
    assert.equal(hood.stator[0][1], 30);
});

test("LogModel reports a follower that dropped while enabled", () => {
    const m = new LogModel(fmLog());
    assert.deepEqual(m.mechanisms[0].deadFollowers, [1]);
    assert.ok(m.warnings.some((w) => w.includes("Hood #2")));
});

test("LogModel prefers the every-loop Runtime battery voltage", () => {
    const m = new LogModel(fmLog());
    assert.ok(m.batteryVoltage().length > m.ch("BatteryLogger/BatteryVoltage").length);
    assert.equal(m.batteryVoltageKey, "/Runtime/BatteryVoltage");
});

test("canBuses lists every bus that logged utilization", () => {
    assert.deepEqual(new LogModel(fmLog()).canBuses(), ["SystemCoreCAN1", "SystemCoreCAN3"]);
});

test("analysisKeep drops channels no analysis page reads", () => {
    const log = fmLog();
    assert.equal(log.has("/RealOutputs/Unread/Thing"), false);
    assert.equal(log.counts.get("/RealOutputs/Unread/Thing"), 1);
});

test("alertEvents reports each alert once, when it became active", () => {
    const events = alertEvents(new LogModel(fmLog()));
    assert.deepEqual(events.map((e) => [e.t, e.level, e.text]), [
        [2, "warning", "Low battery"],
        [3, "warning", "Hood follower"],
        [5, "error", "CAN bus 3 down"],
    ]);
});

test("connectionDropouts finds the leader's disconnected window", () => {
    const [d] = connectionDropouts(new LogModel(fmLog()));
    assert.equal(d.motor, "Hood");
    assert.ok(Math.abs(d.totalSec - 1) < 0.05);
});

test("withFollowers leaves the leader alone when there are no followers", () => {
    const leader = [[0, 5]];
    assert.equal(withFollowers(leader, []), leader);
});

test("staleTraces ignores the disabled gap between auto and teleop", () => {
    const w = new LogWriter();
    w.put(ENABLED, "boolean", 0, true);
    w.put(ENABLED, "boolean", 15, false);
    w.put(ENABLED, "boolean", 18, true);
    w.put(ENABLED, "boolean", 30, false);
    w.series("/Mechanisms/Hood/StatorCurrentAmps", "double", { from: 0, to: 15, hz: 10, fn: () => 5 });
    w.series("/Mechanisms/Hood/StatorCurrentAmps", "double", { from: 18, to: 30, hz: 10, fn: () => 5 });
    assert.deepEqual(staleTraces(new LogModel(parseWpilog(w.buffer()))), []);
});

test("staleTraces still finds a real dropout inside one enabled window", () => {
    const w = new LogWriter();
    w.put(ENABLED, "boolean", 0, true);
    w.put(ENABLED, "boolean", 30, false);
    w.series("/Mechanisms/Hood/StatorCurrentAmps", "double", { from: 0, to: 10, hz: 10, fn: () => 5 });
    w.series("/Mechanisms/Hood/StatorCurrentAmps", "double", { from: 14, to: 30, hz: 10, fn: () => 5 });
    const [stale] = staleTraces(new LogModel(parseWpilog(w.buffer())));
    assert.ok(stale, "expected the 4 s gap to be reported");
    assert.equal(stale.count, 1);
    assert.ok(Math.abs(stale.gaps[0][2] - 4) < 0.2);
});
