package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
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
