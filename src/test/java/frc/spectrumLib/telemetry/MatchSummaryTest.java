package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.telemetry.MatchSummary.Write;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.driverstation.Alert.Level;

class MatchSummaryTest {
    /** The loop period RobotLoop reports in tests. */
    private static final double DT = 0.02;

    private static final MatchSummary.MatchInfo Q12 =
            new MatchSummary.MatchInfo("TEST", "QUALIFICATION", 12, 1, "BLUE", 2, 0.0);

    @TempDir Path dir;

    /** One loop's Driver Station and power state. */
    private static final class Loop {
        double volts = 12.5;
        boolean brownedOut = false;
        boolean ds = true;
        boolean enabled = false;
        boolean auto = false;
        boolean teleop = false;
        boolean eStopped = false;
        boolean fms = true;
    }

    private double now = 10.0;

    private Write step(MatchSummary s, Loop l) {
        return step(s, l, DT);
    }

    private Write step(MatchSummary s, Loop l, double dt) {
        now += dt;
        return s.update(
                now, l.volts, l.brownedOut, l.ds, l.enabled, l.auto, l.teleop, l.eStopped, l.fms);
    }

    private static JSONObject json(MatchSummary s) throws ParseException {
        return json(s, Write.TELEOP_END);
    }

    private static JSONObject json(MatchSummary s, Write write) throws ParseException {
        return (JSONObject) new JSONParser().parse(s.toJson(Q12, 0, write));
    }

    private static JSONObject obj(JSONObject o, String... path) {
        for (String k : path) {
            o = (JSONObject) o.get(k);
        }
        return o;
    }

    private static double num(JSONObject o, String key) {
        return ((Number) o.get(key)).doubleValue();
    }

    @Test
    void batteryMinimumAndTimeUnderEachThreshold() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        l.volts = 0.0; // no reading yet: ignored
        step(s, l);
        l.volts = 7.5;
        for (int i = 0; i < 10; i++) {
            step(s, l);
        }
        l.volts = 6.8;
        for (int i = 0; i < 5; i++) {
            step(s, l);
        }
        double minAt = now - 4 * DT;
        l.volts = 12.0;
        step(s, l, 3.0); // a stall: credits at most MAX_STEP_SECONDS, and only while low

        var b = obj(json(s), "battery");
        assertEquals(6.8, num(b, "minVolts"), 1e-9);
        assertEquals(minAt, num(b, "minAt"), 1e-3);
        assertEquals(15 * DT, num(b, "secondsUnder8V"), 1e-6);
        assertEquals(5 * DT, num(b, "secondsUnder7V"), 1e-6);
    }

    @Test
    void countsEachBrownoutOnce() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        l.brownedOut = true;
        step(s, l);
        step(s, l);
        l.brownedOut = false;
        step(s, l);
        l.brownedOut = true;
        step(s, l);
        var b = obj(json(s), "brownouts");
        assertEquals(2, num(b, "count"));
        assertEquals(3 * DT, num(b, "seconds"), 1e-6);
    }

    @Test
    void overrunsAndLongestLoopCountOnlyWhileEnabled() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        step(s, l, 0.5); // disabled stall (PathPlanner warmup): ignored
        l.enabled = true;
        l.teleop = true;
        step(s, l);
        step(s, l, 0.030); // over 1.25 periods
        step(s, l, 0.024); // under
        step(s, l, 0.110);
        step(s, l);
        var loop = obj(json(s), "loop");
        assertEquals(5, num(loop, "enabledLoops"));
        assertEquals(2, num(loop, "overruns"));
        assertEquals(110, num(loop, "longestMs"), 1e-6);
        assertEquals(25, num(loop, "overrunMs"), 1e-6);
    }

    @Test
    void driverStationDropsCountOnlyAfterItFirstConnects() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        l.ds = false; // booting, no DS yet
        step(s, l);
        step(s, l);
        l.ds = true;
        step(s, l);
        l.ds = false;
        step(s, l);
        step(s, l);
        l.ds = true;
        step(s, l);
        var ds = obj(json(s), "ds");
        assertEquals(1, num(ds, "drops"));
        assertEquals(2 * DT, num(ds, "secondsWithout"), 1e-6);
        assertEquals(2, ((JSONArray) ds.get("events")).size());
    }

    @Test
    void devicesDropRebootAndTakeTheirBusDownTogether() throws ParseException {
        var s = new MatchSummary(null);
        var hood = new AtomicBoolean(true);
        var launcher = new AtomicBoolean(true);
        var intake = new AtomicBoolean(true);
        var hoodReset = new AtomicBoolean(false);
        double[] hoodTemp = {30};
        s.addDevice("Hood", "CANivore", hood::get, hoodReset::get, () -> hoodTemp[0]);
        s.addDevice("Launcher", "CANivore", launcher::get, null, null);
        s.addDevice("Ghost", "CANivore", () -> false, null, null); // never on the bus
        s.addDevice("Stuck", "CANivore", () -> true, null, null);
        s.addDevice("Stuck2", "CANivore", () -> true, null, null);
        var pigeon = new AtomicBoolean(true);
        s.addDevice("Pigeon", "CANivore", pigeon::get, null, null);
        s.addDevice("Intake", "SystemCoreCAN0", intake::get, null, null);
        var l = new Loop();
        step(s, l);

        // Hood alone drops: a device drop, not a bus drop.
        hood.set(false);
        hoodTemp[0] = 0; // what a dropped device reads
        step(s, l);
        hood.set(true);
        hoodTemp[0] = 41;
        step(s, l);

        // The CANivore drops: every device that was ever on it goes quiet at once. One device that
        // still reads connected (an input that never goes false) does not hide it.
        hood.set(false);
        launcher.set(false);
        pigeon.set(false);
        step(s, l);
        step(s, l);
        hood.set(true);
        launcher.set(true);
        pigeon.set(true);
        step(s, l);

        // Hood reboots.
        hoodReset.set(true);
        step(s, l);
        hoodReset.set(false);
        step(s, l);

        var j = json(s);
        var canivore = obj(j, "can", "CANivore");
        assertEquals(6, num(canivore, "devices"));
        assertEquals(1, num(canivore, "drops"));
        assertEquals(2 * DT, num(canivore, "downSeconds"), 1e-6);
        assertEquals(0, num(obj(j, "can", "SystemCoreCAN0"), "drops"));

        var h = obj(j, "devices", "Hood");
        assertEquals(2, num(h, "drops"));
        assertEquals(1, num(h, "reboots"));
        assertEquals(5, ((JSONArray) h.get("events")).size());
        assertEquals(1, num(obj(j, "devices", "Launcher"), "drops"));
        assertNull(obj(j, "devices").get("Intake")); // nothing happened to it
        assertEquals(List.of("Ghost"), j.get("neverConnected"));
        assertEquals(41, num(obj(j, "peakTempC"), "Hood"), 1e-9);
    }

    @Test
    void theAutoEndIsRecordedAsTheAutoReportedIt() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        s.autoStarted("TBTB");
        l.enabled = true;
        l.auto = true;
        step(s, l);
        // Still running: no reason yet, whatever the summary might guess.
        assertNull(obj(json(s, Write.AUTO_END), "auto").get("ended"));
        step(s, l, 3.0);
        s.autoEnded("DISABLED_BY_FMS_EARLY", 3.02);
        l.enabled = false;
        step(s, l);
        var auto = obj(json(s), "auto");
        assertEquals("TBTB", auto.get("routine"));
        assertEquals("DISABLED_BY_FMS_EARLY", auto.get("ended"));
        assertEquals(3.02, num(auto, "seconds"), 1e-9);

        // A second auto (a practice restart) starts clean.
        s.autoStarted("BBBB");
        assertNull(obj(json(s), "auto").get("ended"));
    }

    @Test
    void anFmsMatchIsWrittenWhenAutoEndsAndAgainWhenTeleopEnds() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        assertNull(step(s, l)); // pre-match, disabled
        l.enabled = true;
        l.auto = true;
        assertNull(step(s, l));
        l.enabled = false; // the gap between auto and teleop
        assertEquals(Write.AUTO_END, step(s, l));
        assertEquals("autoEnd", json(s, Write.AUTO_END).get("writtenOn"));
        l.auto = false;
        l.teleop = true;
        assertNull(step(s, l));
        l.enabled = true;
        assertNull(step(s, l));
        assertNull(step(s, l));
        l.enabled = false;
        assertEquals(Write.TELEOP_END, step(s, l));
        assertNull(step(s, l)); // once
    }

    @Test
    void aDeadRobotAfterAutoStillHasItsAutoEndFile() {
        // The robot never enables in teleop (dead on the field, or rebooted): the auto-end write
        // is the file.
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        l.enabled = true;
        l.auto = true;
        step(s, l);
        l.enabled = false;
        assertEquals(Write.AUTO_END, step(s, l));
        l.auto = false;
        l.teleop = true;
        for (int i = 0; i < 100; i++) {
            assertNull(step(s, l));
        }
    }

    @Test
    void anEStopInAutoIsWrittenAtOnce() {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        l.enabled = true;
        l.auto = true;
        step(s, l);
        // E-stop (or A-stop) disables the robot the same loop: the E-stop is the reason.
        l.eStopped = true;
        l.enabled = false;
        assertEquals(Write.E_STOP, step(s, l));
        assertNull(step(s, l)); // once while it stays E-stopped
    }

    @Test
    void anEStopInTeleopIsWrittenAtOnce() {
        var s = new MatchSummary(null);
        var l = new Loop();
        l.teleop = true;
        l.enabled = true;
        step(s, l);
        l.eStopped = true;
        assertEquals(Write.E_STOP, step(s, l));
        l.enabled = false;
        assertEquals(Write.TELEOP_END, step(s, l)); // the disable that follows rewrites it
    }

    @Test
    void aDriverStationDropInAnFmsMatchIsWritten() {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        l.teleop = true;
        l.enabled = true;
        step(s, l);
        // The DS drops: the robot disables, and the match type may go with it.
        l.ds = false;
        l.enabled = false;
        l.fms = false;
        assertEquals(Write.DS_LOST, step(s, l));
        assertNull(step(s, l));
        // It comes back and the match goes on: teleop's end is written again.
        l.ds = true;
        l.fms = true;
        l.enabled = true;
        assertNull(step(s, l));
        l.enabled = false;
        assertEquals(Write.TELEOP_END, step(s, l));
    }

    @Test
    void aDriverStationDropBeforeItFirstConnectsIsNotWritten() {
        var s = new MatchSummary(null);
        var l = new Loop();
        l.ds = false; // booting
        assertNull(step(s, l));
        assertNull(step(s, l));
    }

    @Test
    void practiceRunsAreNeverWritten() {
        var s = new MatchSummary(null);
        var l = new Loop();
        l.fms = false; // match type NONE
        l.auto = true;
        step(s, l);
        l.enabled = true;
        assertNull(step(s, l));
        l.enabled = false;
        assertNull(step(s, l));
        l.auto = false;
        l.teleop = true;
        l.enabled = true;
        assertNull(step(s, l));
        l.eStopped = true;
        assertNull(step(s, l));
        l.ds = false;
        l.enabled = false;
        assertNull(step(s, l));
    }

    @Test
    void alertsAreCountedByLevelAndListedOncePerText() throws ParseException {
        var s = new MatchSummary(null);
        var l = new Loop();
        step(s, l);
        s.alertRaised(Level.HIGH, "CANivore down");
        s.alertRaised(Level.HIGH, "CANivore down");
        s.alertRaised(Level.MEDIUM, "Loop overrunning");
        s.alertRaised(Level.LOW, "Logging to internal storage");
        Thread other = new Thread(() -> s.alertRaised(Level.HIGH, "off thread"));
        other.start();
        try {
            other.join();
        } catch (InterruptedException e) {
            throw new AssertionError(e);
        }
        var a = obj(json(s), "alerts");
        assertEquals(2, num(a, "errorsRaised"));
        assertEquals(1, num(a, "warningsRaised"));
        assertEquals(1, num(a, "infosRaised"));
        var raised = (JSONArray) a.get("raised");
        assertEquals(2, raised.size());
        var first = (JSONObject) raised.get(0);
        assertEquals("CANivore down", first.get("text"));
        assertEquals(2, num(first, "count"));
    }

    @Test
    void anAlertClearedAndSetAgainWithinALoopIsNotRaisedAgain() throws ParseException {
        var s = new MatchSummary(null);
        step(s, new Loop());
        var alert = new Alert("MatchSummaryTest flapping alert", Level.HIGH);
        try {
            for (int i = 0; i < 5; i++) {
                alert.set(false); // code that clears its alerts each loop, then sets them
                alert.set(true);
                Alert.periodic();
            }
            alert.set(false);
            Alert.periodic();
            alert.set(true);
            Alert.periodic();
        } finally {
            alert.close();
        }
        long count = -1;
        for (Object o : (JSONArray) obj(json(s), "alerts").get("raised")) {
            var r = (JSONObject) o;
            if ("MatchSummaryTest flapping alert".equals(r.get("text"))) {
                count = ((Number) r.get("count")).longValue();
            }
        }
        assertEquals(2, count);
    }

    @Test
    void visionCountsAndMatchIdentityAreInTheFile() throws ParseException {
        var s = new MatchSummary(null);
        long[] counts = {120, 30};
        s.addVisionSource("LL-Back/MT2", () -> counts[0], () -> counts[1]);
        step(s, new Loop());
        var j = json(s);
        assertEquals(MatchSummary.VERSION, num(j, "version"));
        assertEquals(12, num(obj(j, "match"), "number"));
        assertEquals("QUALIFICATION", obj(j, "match").get("type"));
        assertEquals(120, num(obj(j, "vision", "LL-Back/MT2"), "accepted"));
        assertEquals(30, num(obj(j, "vision", "LL-Back/MT2"), "rejected"));
    }

    @Test
    void aBadMatchStaysAFewKilobytes() {
        var s = new MatchSummary(null);
        var flapping = new AtomicBoolean(true);
        for (int i = 0; i < 25; i++) {
            s.addDevice("Motor" + i, "CANivore", flapping::get, null, () -> 40.0);
            s.addVisionSource("Camera" + i, () -> 100, () -> 100);
        }
        var l = new Loop();
        for (int i = 0; i < 400; i++) {
            flapping.set(i % 2 == 0);
            step(s, l);
            s.alertRaised(Level.HIGH, "alert " + i);
        }
        String text = s.toJson(Q12, 0, Write.TELEOP_END);
        assertTrue(text.length() < 16_000, "summary is " + text.length() + " bytes");
    }

    @Test
    void writesBesideThisBootsLogEvenWhenTheClockIsBehind() throws IOException {
        // SystemCore booted with its clock behind: this boot's log is stamped before older ones.
        File older = dir.resolve("akit_26-10-16_txman1_q33.wpilog").toFile();
        File current = dir.resolve("akit_26-10-16_txman1_q34.wpilog").toFile();
        File replay = dir.resolve("akit_26-10-16_txman1_q33_sim.wpilog").toFile();
        Files.writeString(older.toPath(), "x");
        Files.writeString(current.toPath(), "x");
        Files.writeString(replay.toPath(), "x");
        assertTrue(older.setLastModified(3_000_000L));
        assertTrue(current.setLastModified(1_000_000L));
        assertTrue(replay.setLastModified(4_000_000L));
        var atBoot = Set.of(older.getName(), replay.getName());

        File log = SummaryFileWriter.thisBootsLog(dir.toFile(), atBoot);
        assertEquals(current, log);
        File target = SummaryFileWriter.summaryFor(log);
        assertEquals("akit_26-10-16_txman1_q34.summary.json", target.getName());

        // Nothing listed at boot (the sim): the newest log, never a replay output.
        assertEquals(older, SummaryFileWriter.thisBootsLog(dir.toFile(), null));
        // This boot's log not there (yet): no file rather than the wrong one.
        assertNull(
                SummaryFileWriter.thisBootsLog(
                        dir.toFile(), Set.of(current.getName(), older.getName())));
    }

    @Test
    void writesAtomicallyAndLastWriteWins() throws IOException {
        Path target = dir.resolve("akit_q34.summary.json");
        assertTrue(SummaryFileWriter.writeAtomically(target, "{\"version\":1}"));
        assertTrue(SummaryFileWriter.writeAtomically(target, "{\"version\":1,\"n\":2}"));
        assertEquals("{\"version\":1,\"n\":2}", Files.readString(target));
        assertFalse(dir.resolve("akit_q34.summary.json.tmp").toFile().exists());
    }

    @Test
    void aFailedWriteLeavesNoTemporaryFile() throws IOException {
        // A folder in the target's place, with something in it: the rename over it fails.
        Path target = dir.resolve("akit_q34.summary.json");
        Files.createDirectory(target);
        Files.writeString(target.resolve("inside"), "x");
        assertThrows(
                IOException.class, () -> SummaryFileWriter.writeAtomically(target, "{\"v\":1}"));
        assertFalse(dir.resolve("akit_q34.summary.json.tmp").toFile().exists());
    }

    @Test
    void theWriterThreadStartsWithTheWriterNotOnTheFirstWrite() throws Exception {
        Files.writeString(dir.resolve("akit_q34.wpilog"), "x");
        var writer = new SummaryFileWriter(dir.toFile());
        // Started by the constructor (robot init, before the main thread goes real-time).
        assertTrue(
                Thread.getAllStackTraces().keySet().stream()
                        .anyMatch(t -> t.getName().equals("MatchSummary") && t.isAlive()));
        writer.submit("{\"version\":1}");
        long deadline = System.nanoTime() + 5_000_000_000L;
        while (writer.lastResult().isEmpty() && System.nanoTime() < deadline) {
            Thread.sleep(10);
        }
        assertEquals("akit_q34.summary.json", writer.lastResult());
        assertEquals("{\"version\":1}", Files.readString(dir.resolve("akit_q34.summary.json")));
    }
}
