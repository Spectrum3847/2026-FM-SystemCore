package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LogStorageTest {
    @TempDir Path dir;

    private File log(String name, int bytes, long modified) throws IOException {
        File f = dir.resolve(name).toFile();
        Files.write(f.toPath(), new byte[bytes]);
        assertTrue(f.setLastModified(modified));
        return f;
    }

    @Test
    void deletesOldestUntilUnderTheCap() throws IOException {
        File oldest = log("a.wpilog", 400, 1_000_000L);
        File middle = log("b.wpilog", 400, 2_000_000L);
        File newest = log("c.wpilog", 400, 3_000_000L);
        long[] r = LogStorage.cleanUp(dir.toFile(), 900, 0, 1);
        assertArrayEquals(new long[] {1, 400}, r);
        assertFalse(oldest.exists());
        assertTrue(middle.exists());
        assertTrue(newest.exists());
    }

    @Test
    void keepsTheNewestEvenOverTheCap() throws IOException {
        log("a.wpilog", 400, 1_000_000L);
        File newest = log("b.wpilog", 400, 2_000_000L);
        LogStorage.cleanUp(dir.toFile(), 0, 0, 1);
        assertTrue(newest.exists());
    }

    private static final long NOW = 1_800_000_000_000L; // 2027
    private static final long HOUR = 3_600_000L;

    private static LogStorage.LogFile f(String name, long bytes, long modified) {
        return new LogStorage.LogFile(name, bytes, modified);
    }

    /**
     * SystemCore booted with its clock behind the old logs: this boot's log is stamped earliest, so
     * it sorts oldest. Protected by name, it is skipped and the real oldest old log goes.
     */
    @Test
    void neverDeletesThisBootsLogWhenTheClockIsBehind() {
        var logs =
                List.of(
                        f("current.wpilog", 400, NOW - 10 * HOUR), // written, but clock was behind
                        f("old1.wpilog", 400, NOW + 5 * HOUR),
                        f("old2.wpilog", 400, NOW + 6 * HOUR),
                        f("old3.wpilog", 400, NOW + 7 * HOUR));
        assertEquals(
                List.of("old1.wpilog"),
                LogStorage.selectForDeletion(
                        logs, 1200, Long.MAX_VALUE, 0, 1, Set.of("current.wpilog"), NOW));
    }

    /** Anything modified in the last few minutes is being written, protected or not. */
    @Test
    void neverDeletesARecentlyModifiedFile() {
        var logs =
                List.of(
                        f("hoot.hoot", 400, NOW - 1000),
                        f("old1.wpilog", 400, NOW - 2 * HOUR),
                        f("old2.wpilog", 400, NOW - HOUR));
        assertEquals(
                List.of("old1.wpilog", "old2.wpilog"),
                LogStorage.selectForDeletion(logs, 0, Long.MAX_VALUE, 0, 0, Set.of(), NOW));
    }

    /** Low disk: deletes oldest until the free floor is met, counting each deletion as freed. */
    @Test
    void deletesUntilTheFreeFloorIsMet() {
        var logs =
                List.of(
                        f("c.wpilog", 300, NOW - HOUR),
                        f("a.wpilog", 300, NOW - 3 * HOUR),
                        f("b.wpilog", 300, NOW - 2 * HOUR));
        assertEquals(
                List.of("a.wpilog", "b.wpilog"),
                LogStorage.selectForDeletion(logs, Long.MAX_VALUE, 500, 1000, 0, Set.of(), NOW));
    }

    /** Equal times sort by name, so the order is total even then. */
    @Test
    void equalTimesAreOrderedByName() {
        var logs = List.of(f("b.wpilog", 400, NOW - HOUR), f("a.wpilog", 400, NOW - HOUR));
        assertEquals(
                List.of("a.wpilog"),
                LogStorage.selectForDeletion(logs, 400, Long.MAX_VALUE, 0, 0, Set.of(), NOW));
    }

    @Test
    void neverTouchesOtherFiles() throws IOException {
        File other = log("notes.txt", 5000, 1_000L);
        log("a.wpilog", 400, 2_000L);
        log("b.hoot", 400, 3_000L);
        long[] r = LogStorage.cleanUp(dir.toFile(), 0, 0, 0);
        assertEquals(2, r[0]);
        assertTrue(other.exists());
    }
}
