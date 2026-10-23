package frc.spectrumLib.params;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ParamStoreTest {
    @TempDir Path tmp;

    private Path deploy;
    private Path local;
    private ParamStore store;
    private ParamGroup cams;

    @BeforeEach
    void setUp() throws IOException {
        deploy = Files.createDirectories(tmp.resolve("deploy/params"));
        local = tmp.resolve("local");
        store = new ParamStore();
        cams = new ParamGroup("cams", store::register);
    }

    private void file(Path path, String json) throws IOException {
        Files.createDirectories(path.getParent());
        Files.writeString(path, json);
    }

    private Path deployFile() {
        return deploy.resolve("cams.json");
    }

    private Path robotFile(String robot) {
        return deploy.resolve("robots").resolve(robot).resolve("cams.json");
    }

    /** This robot's local folder: {@code <local>/<robot>/}. */
    private Path robotLocal() {
        return local.resolve("FM2026");
    }

    private Path localFile() {
        return robotLocal().resolve("cams.json");
    }

    private void configure() {
        store.configure(new ParamStore.Settings(deploy, "FM2026", local));
    }

    private boolean hasProblem(ParamStore.Level level, String text) {
        return store.problems().values().stream()
                .anyMatch(p -> p.level() == level && p.text().contains(text));
    }

    // ── Precedence ──────────────────────────────────────────────────────────

    @Test
    void defaultWhenNoFiles() {
        configure();
        DoubleParam p = cams.number("rate", 2.0, "s");
        assertEquals(2.0, p.get());
        assertEquals(ParamSource.DEFAULT, p.source());
        assertTrue(store.problems().isEmpty(), "missing files are not a problem");
    }

    @Test
    void eachLayerBeatsTheOneBelow() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1, \"c\": 1}");
        file(robotFile("FM2026"), "{\"b\": 2, \"c\": 2}");
        file(localFile(), "{\"c\": 3}");
        configure();
        DoubleParam a = cams.number("a", 0, "");
        DoubleParam b = cams.number("b", 0, "");
        DoubleParam c = cams.number("c", 0, "");
        DoubleParam d = cams.number("d", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(ParamSource.DEPLOY, a.source());
        assertEquals(2.0, b.get());
        assertEquals(ParamSource.ROBOT, b.source());
        assertEquals(3.0, c.get());
        assertEquals(ParamSource.LOCAL, c.source());
        assertEquals(0.0, d.get());
        assertEquals(ParamSource.DEFAULT, d.source());
    }

    @Test
    void anotherRobotsFileIsIgnored() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(robotFile("PM2026"), "{\"a\": 9}");
        configure();
        assertEquals(1.0, cams.number("a", 0, "").get());
    }

    @Test
    void declaredBeforeConfigureIsResolvedAgain() throws IOException {
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(ParamSource.DEFAULT, a.source());
        file(deployFile(), "{\"a\": 1}");
        configure();
        assertEquals(1.0, a.get());
        assertEquals(ParamSource.DEPLOY, a.source());
    }

    @Test
    void valueMayBeWrappedInAnObject() throws IOException {
        file(localFile(), "{\"a\": {\"value\": 4, \"replaced\": 1, \"savedAt\": \"x\"}}");
        configure();
        assertEquals(4.0, cams.number("a", 0, "").get());
    }

    @Test
    void allTypes() throws IOException {
        file(deployFile(), "{\"n\": 1.5, \"i\": 3, \"i2\": 4.0, \"b\": false, \"s\": \"hi\"}");
        configure();
        assertEquals(1.5, cams.number("n", 0, "").get());
        assertEquals(3, cams.integer("i", 0, "").get());
        assertEquals(4, cams.integer("i2", 0, "").get());
        assertEquals(false, cams.bool("b", true).get());
        assertEquals("hi", cams.string("s", "").get());
    }

    // ── Bad files and values ────────────────────────────────────────────────

    @Test
    void badDeployFileFallsBackToDefaultWithAnError() throws IOException {
        file(deployFile(), "{\"a\": 1,,}");
        configure();
        DoubleParam a = cams.number("a", 7, "");
        assertEquals(7.0, a.get());
        assertEquals(ParamSource.DEFAULT, a.source());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "params/cams.json is unreadable"));
    }

    @Test
    void badLocalFileFallsBackToTheRobotAndDeployLayers() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1}");
        file(robotFile("FM2026"), "{\"b\": 2}");
        file(localFile(), "not json at all");
        configure();
        DoubleParam a = cams.number("a", 0, "");
        DoubleParam b = cams.number("b", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(2.0, b.get());
        assertEquals(ParamSource.ROBOT, b.source());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "unreadable"));
    }

    @Test
    void badRobotFileStillLetsTheLocalFileApply() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(robotFile("FM2026"), "[1, 2]");
        file(localFile(), "{\"a\": 3}");
        configure();
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "not a JSON object"));
    }

    @Test
    void emptyFileIsUnreadable() throws IOException {
        file(deployFile(), "");
        configure();
        assertEquals(5.0, cams.number("a", 5, "").get());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "unreadable"));
    }

    @Test
    void wrongTypeOrOutOfBoundsValueIsSkipped() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1, \"i\": 2}");
        file(robotFile("FM2026"), "{\"a\": \"fast\", \"b\": 500, \"i\": 2.5}");
        configure();
        DoubleParam a = cams.number("a", 0, "");
        DoubleParam b = cams.number("b", 0, "", 0, 10);
        IntParam i = cams.integer("i", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(ParamSource.DEPLOY, a.source());
        assertEquals(1.0, b.get());
        assertEquals(2, i.get());
        assertTrue(hasProblem(ParamStore.Level.WARNING, "a expected a number"));
        assertTrue(hasProblem(ParamStore.Level.WARNING, "b 500.0 is outside [0.0, 10.0]"));
        assertTrue(hasProblem(ParamStore.Level.WARNING, "i expected a whole number"));
    }

    @Test
    void unknownKeyIsInfoUntilDeclared() throws IOException {
        file(deployFile(), "{\"a\": 1, \"typo\": 2}");
        configure();
        cams.number("a", 0, "");
        assertTrue(hasProblem(ParamStore.Level.INFO, "has keys no code declares: typo"));
        cams.number("typo", 0, "");
        assertFalse(hasProblem(ParamStore.Level.INFO, "no code declares"));
    }

    @Test
    void localValuesAreCounted() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configure();
        cams.number("a", 0, "");
        assertTrue(hasProblem(ParamStore.Level.INFO, "1 value from this robot's local overrides"));
    }

    @Test
    void badDeclarationsThrow() {
        cams.number("a", 0, "");
        assertThrows(IllegalArgumentException.class, () -> cams.number("a", 0, ""));
        assertThrows(IllegalArgumentException.class, () -> cams.number("b", 11, "", 0, 10));
        assertThrows(IllegalArgumentException.class, () -> cams.number("c", Double.NaN, ""));
        assertThrows(IllegalArgumentException.class, () -> cams.number("has space", 0, ""));
    }

    @Test
    void checkFileReportsWithoutApplying() throws IOException {
        DoubleParam a = cams.number("a", 0, "", 0, 10);
        file(deployFile(), "{\"a\": 20, \"typo\": 1}");
        var problems = store.checkFile(deployFile());
        assertEquals(2, problems.size());
        assertEquals(0.0, a.get());
    }

    // ── Live ────────────────────────────────────────────────────────────────

    /** A transport whose dashboard values the test sets. */
    private static final class FakeTransport implements ParamTransport {
        final Map<String, Object> values = new HashMap<>();

        @Override
        public LiveValue publish(Param<?> param) {
            String key = param.path();
            values.put(key, param.get());
            return LiveValue.of(() -> values.get(key), v -> values.put(key, v));
        }

        @Override
        public LiveValue button(String name) {
            values.put(name, false);
            return LiveValue.of(() -> values.get(name), v -> values.put(name, v));
        }
    }

    @Test
    void liveChangeAppliesOnlyWhileDisabled() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(3.0, t.values.get("cams/a"), "published with the resolved value");

        t.values.put("cams/a", 4.0);
        store.applyLive(false);
        assertEquals(3.0, a.get(), "not while enabled");
        assertTrue(store.pendingLive().contains("cams/a"));

        int version = a.version();
        store.applyLive(true);
        assertEquals(4.0, a.get());
        assertEquals(ParamSource.LIVE, a.source(), "live beats the local file");
        assertTrue(a.version() > version);
        assertTrue(store.pendingLive().isEmpty());
    }

    @Test
    void liveChangeBackBeforeDisableIsDropped() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleParam a = cams.number("a", 1, "");
        t.values.put("cams/a", 4.0);
        store.applyLive(false);
        t.values.put("cams/a", 1.0);
        store.applyLive(false);
        assertTrue(store.pendingLive().isEmpty());
        store.applyLive(true);
        assertEquals(ParamSource.DEFAULT, a.source());
    }

    @Test
    void badLiveValueIsRefusedAndUndone() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleParam a = cams.number("a", 1, "", 0, 10);
        IntParam i = cams.integer("i", 1, "");
        t.values.put("cams/a", 99.0);
        t.values.put("cams/i", 2.5);
        store.applyLive(true);
        assertEquals(1.0, a.get());
        assertEquals(1, i.get());
        assertEquals(1.0, t.values.get("cams/a"), "dashboard shows the kept value");
        assertTrue(hasProblem(ParamStore.Level.WARNING, "cams/a refused"));

        t.values.put("cams/a", 5.0);
        store.applyLive(true);
        assertEquals(5.0, a.get());
        assertFalse(hasProblem(ParamStore.Level.WARNING, "cams/a refused"));
    }

    @Test
    void integerFromNtDouble() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        IntParam i = cams.integer("i", 1, "");
        t.values.put("cams/i", 3.0);
        store.applyLive(true);
        assertEquals(3, i.get());
    }

    // ── Save ────────────────────────────────────────────────────────────────

    @Test
    void saveWritesOnlyChangedValuesAndReloadsAsLocal() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleParam a = cams.number("a", 0, "");
        cams.number("b", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);

        ParamStore.SaveJob job = store.prepareSave("2026-10-23T12:00:00Z");
        assertEquals(java.util.Map.of("cams/a", 2.0), job.values());
        ParamStore.write(job);

        ObjectNode saved = ParamStore.parseObject(localFile());
        assertEquals(2.0, saved.get("a").get("value").doubleValue());
        assertEquals(1.0, saved.get("a").get("replaced").doubleValue());
        assertEquals("2026-10-23T12:00:00Z", saved.get("a").get("savedAt").textValue());
        assertFalse(saved.has("b"));
        assertFalse(Files.exists(robotLocal().resolve("cams.json.tmp")));

        var next = new ParamStore();
        next.configure(new ParamStore.Settings(deploy, "FM2026", local));
        var again = new ParamGroup("cams", next::register);
        DoubleParam a2 = again.number("a", 0, "");
        assertEquals(a.get(), a2.get());
        assertEquals(ParamSource.LOCAL, a2.source());
    }

    @Test
    void savedLiveValuesStopCountingAsUnsaved() throws IOException {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        assertTrue(hasProblem(ParamStore.Level.INFO, "1 value tuned live and not saved"));
        ParamStore.SaveJob job = store.prepareSave("now");
        ParamStore.write(job);
        store.markSaved(job);
        assertFalse(hasProblem(ParamStore.Level.INFO, "not saved"));
        assertTrue(hasProblem(ParamStore.Level.INFO, "1 value from this robot's local overrides"));
        t.values.put("cams/a", 3.0);
        store.applyLive(true);
        assertTrue(hasProblem(ParamStore.Level.INFO, "1 value tuned live and not saved"));
    }

    @Test
    void saveKeepsLocalEntriesAndUndeclaredKeys() throws IOException {
        file(localFile(), "{\"a\": {\"value\": 3, \"savedAt\": \"old\"}, \"gone\": 5}");
        configure();
        cams.number("a", 0, "");
        ParamStore.write(store.prepareSave("new"));
        ObjectNode saved = ParamStore.parseObject(localFile());
        assertEquals("old", saved.get("a").get("savedAt").textValue());
        assertEquals(5, saved.get("gone").intValue());
    }

    @Test
    void valueSetBackToTheGitValueLeavesTheLocalFile() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(localFile(), "{\"a\": 3}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 1.0);
        store.applyLive(true);
        ParamStore.write(store.prepareSave("now"));
        assertTrue(ParamStore.parseObject(localFile()).isEmpty());
    }

    @Test
    void saveMovesAnUnreadableLocalFileAside() throws IOException {
        file(localFile(), "{broken");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        ParamStore.write(store.prepareSave("now"));
        assertEquals("{broken", Files.readString(robotLocal().resolve("cams.json.bad")));
        assertEquals(2.0, ParamStore.parseObject(localFile()).get("a").get("value").doubleValue());
    }

    @Test
    void noLocalFolderNoSave() {
        store.configure(new ParamStore.Settings(deploy, "FM2026", null));
        cams.number("a", 0, "");
        assertEquals(null, store.prepareSave("now"));
    }

    // ── Robot folder and hardware stamps ────────────────────────────────────

    private static final double[] THIS_ROBOT = {0.1, 0.2, 0.3, 0.4};
    private static final double[] SAVING = {0.15, 0.2, 0.3, 0.4};
    private static final double[] OTHER_ROBOT = {-0.3, 0.05, 0.45, -0.1};

    /** A chassis whose devices read {@code atBoot}, and which saves {@link #SAVING} next. */
    private static HardwareStamp chassis(double[] atBoot) {
        return new HardwareStamp(
                "Chassis", "test offsets", atBoot, 0.001, () -> List.of(THIS_ROBOT, SAVING));
    }

    private void configureStamped(double[] atBoot) {
        store.configure(
                new ParamStore.Settings(deploy, "FM2026", local, Map.of("cams", chassis(atBoot))));
    }

    private static String stampJson(double[]... accept) {
        StringBuilder b = new StringBuilder("{\"what\": \"test offsets\", \"accept\": [");
        for (int i = 0; i < accept.length; i++) {
            b.append(i == 0 ? "" : ", ").append(java.util.Arrays.toString(accept[i]));
        }
        return b.append("]}").toString();
    }

    @Test
    void anotherRobotConfigsLocalFolderIsIgnored() throws IOException {
        file(local.resolve("PM2026").resolve("cams.json"), "{\"a\": 9}");
        configure();
        assertEquals(0.0, cams.number("a", 0, "").get());
    }

    @Test
    void stampedFileFromThisRobotIsUsed() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(localFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(new double[] {0.1004, 0.2, 0.3, 0.4});
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(3.0, a.get());
        assertEquals(ParamSource.LOCAL, a.source());
        assertFalse(hasProblem(ParamStore.Level.INFO, "no code declares"), "the stamp is no param");
        assertTrue(store.problems().keySet().stream().noneMatch(k -> k.startsWith("file:")));
    }

    @Test
    void stampedFileFromAnotherRobotIsRefusedAndGitIsUsed() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(localFile(), "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"a\": 3}");
        configureStamped(THIS_ROBOT);
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(ParamSource.ROBOT, a.source());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "was saved on another robot"));
        assertTrue(hasProblem(ParamStore.Level.ERROR, "its cams values are not used"));
    }

    @Test
    void unstampedFileOfAHardStopGroupIsRefused() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configureStamped(THIS_ROBOT);
        assertEquals(0.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "has no hardware stamp"));
    }

    @Test
    void unreadableHardwareRefusesTheFile() throws IOException {
        file(localFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(null);
        assertEquals(0.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "could not be read at boot"));
    }

    @Test
    void groupWithoutAStampIgnoresOne() throws IOException {
        file(
                robotLocal().resolve("other.json"),
                "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"b\": 2}");
        configureStamped(THIS_ROBOT);
        DoubleParam b = new ParamGroup("other", store::register).number("b", 0, "");
        assertEquals(2.0, b.get());
        assertEquals(ParamSource.LOCAL, b.source());
        assertFalse(hasProblem(ParamStore.Level.INFO, "no code declares"));
    }

    /** Save on this robot, boot it again, then boot another robot with the same controller. */
    @Test
    void savedStampMatchesThisRobotAfterTheRebootAndNotAnother() throws IOException {
        configureStamped(THIS_ROBOT);
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        ParamStore.write(store.prepareSave("now"));
        ObjectNode saved = ParamStore.parseObject(localFile());
        assertEquals(HardwareStamp.KEY, saved.fieldNames().next(), "stamp first");
        assertEquals(2, saved.get(HardwareStamp.KEY).get("accept").size());

        for (double[] reading : List.of(THIS_ROBOT, SAVING)) {
            var next = new ParamStore();
            next.configure(
                    new ParamStore.Settings(
                            deploy, "FM2026", local, Map.of("cams", chassis(reading))));
            DoubleParam a = new ParamGroup("cams", next::register).number("a", 0, "");
            assertEquals(2.0, a.get());
            assertEquals(ParamSource.LOCAL, a.source());
        }

        var other = new ParamStore();
        other.configure(
                new ParamStore.Settings(
                        deploy, "FM2026", local, Map.of("cams", chassis(OTHER_ROBOT))));
        DoubleParam a = new ParamGroup("cams", other::register).number("a", 0, "");
        assertEquals(0.0, a.get());
        assertTrue(
                other.problems().values().stream()
                        .anyMatch(p -> p.level() == ParamStore.Level.ERROR));
    }

    @Test
    void saveMovesARefusedFileAsideAndDropsItsValues() throws IOException {
        file(localFile(), "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"a\": 3, \"x\": 1}");
        configureStamped(THIS_ROBOT);
        cams.number("a", 0, "");
        ParamStore.write(store.prepareSave("now"));
        assertTrue(
                Files.readString(robotLocal().resolve("cams.json.refused")).contains("\"a\": 3"));
        ObjectNode saved = ParamStore.parseObject(localFile());
        assertFalse(saved.has("a"));
        assertFalse(saved.has("x"), "the other robot's undeclared keys are not carried over");
        assertTrue(saved.has(HardwareStamp.KEY));
    }

    // ── Files saved before the folder was per robot ─────────────────────────

    private Path legacyFile() {
        return local.resolve("cams.json");
    }

    @Test
    void oldFolderFileIsReadUntilTheNextSaveMovesIt() throws IOException {
        file(legacyFile(), "{\"a\": 3}");
        configure();
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(3.0, a.get());
        assertEquals(ParamSource.LOCAL, a.source());
        assertTrue(hasProblem(ParamStore.Level.INFO, "the next save moves it to"));

        ParamStore.write(store.prepareSave("now"));
        assertFalse(Files.exists(legacyFile()));
        assertTrue(Files.exists(local.resolve("cams.json.moved")));
        assertEquals(3, ParamStore.parseObject(localFile()).get("a").intValue());

        var next = new ParamStore();
        next.configure(new ParamStore.Settings(deploy, "FM2026", local));
        DoubleParam a2 = new ParamGroup("cams", next::register).number("a", 0, "");
        assertEquals(3.0, a2.get());
        assertTrue(next.problems().keySet().stream().noneMatch(k -> k.startsWith("file:")));
    }

    @Test
    void oldFolderFileIsIgnoredOnceThisRobotsExists() throws IOException {
        file(legacyFile(), "{\"a\": 9}");
        file(localFile(), "{\"a\": 3}");
        configure();
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertFalse(hasProblem(ParamStore.Level.INFO, "the next save moves it"));
    }

    @Test
    void oldFolderFileOfAHardStopGroupIsRefusedAndRetiredBySave() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(legacyFile(), "{\"a\": 3}");
        configureStamped(THIS_ROBOT);
        DoubleParam a = cams.number("a", 0, "");
        assertEquals(1.0, a.get(), "an old file has no stamp, so it may be another robot's");
        assertEquals(ParamSource.ROBOT, a.source());
        assertTrue(hasProblem(ParamStore.Level.ERROR, "saved before files were per robot"));
        assertTrue(hasProblem(ParamStore.Level.ERROR, "has no hardware stamp"));

        ParamStore.write(store.prepareSave("now"));
        assertFalse(Files.exists(legacyFile()));
        assertEquals("{\"a\": 3}", Files.readString(local.resolve("cams.json.refused")));
        ObjectNode saved = ParamStore.parseObject(localFile());
        assertFalse(saved.has("a"));
        assertTrue(saved.has(HardwareStamp.KEY));
    }

    @Test
    void stampedOldFolderFileFromThisRobotIsMigrated() throws IOException {
        file(legacyFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(THIS_ROBOT);
        assertEquals(3.0, cams.number("a", 0, "").get());
    }

    @Test
    void noRobotNameKeepsTheFlatFolder() throws IOException {
        file(legacyFile(), "{\"a\": 3}");
        store.configure(new ParamStore.Settings(deploy, null, local));
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertFalse(hasProblem(ParamStore.Level.INFO, "the next save moves it"));
    }
}
