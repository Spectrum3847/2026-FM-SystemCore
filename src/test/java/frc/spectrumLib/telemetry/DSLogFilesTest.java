package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.EOFException;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** File-name rules, write-then-rename and the folder cap of the DS log receiver. */
class DSLogFilesTest {
    @TempDir Path dir;

    private static final String LOG = "2026_10_02 14_03_27 Fri.dslog";
    private static final String EVENTS = "2026_10_02 14_03_27 Fri.dsevents";

    private static InputStream bytes(int n, int fill) {
        byte[] b = new byte[n];
        java.util.Arrays.fill(b, (byte) fill);
        return new ByteArrayInputStream(b);
    }

    @Test
    void acceptsOnlyDriverStationNames() {
        assertTrue(DSLogFiles.isValidName(LOG));
        assertTrue(DSLogFiles.isValidName(EVENTS));
        assertFalse(DSLogFiles.isValidName(null));
        assertFalse(DSLogFiles.isValidName(""));
        assertFalse(DSLogFiles.isValidName("2026_10_02 14_03_27 Fri.wpilog"));
        assertFalse(DSLogFiles.isValidName("2026_10_02 14_03_27 Friday.dslog"));
        assertFalse(DSLogFiles.isValidName("2026_10_02_14_03_27 Fri.dslog"));
        assertFalse(DSLogFiles.isValidName("../2026_10_02 14_03_27 Fri.dslog"));
        assertFalse(DSLogFiles.isValidName("x/2026_10_02 14_03_27 Fri.dslog"));
        assertFalse(DSLogFiles.isValidName("2026_10_02 14_03_27 Fri.dslog\n"));
        assertFalse(DSLogFiles.isValidName("2026_10_02 14_03_27 Fri.dslog.part"));
        assertFalse(DSLogFiles.isValidName("2026_10_02 14_03_27 F1i.dslog"));
    }

    @Test
    void writesThenRenamesAndReplaces() throws IOException {
        File folder = dir.resolve("ds").toFile();
        assertEquals(10, DSLogFiles.receive(bytes(10, 1), 10, folder, LOG, () -> false));
        assertEquals(10, Files.size(folder.toPath().resolve(LOG)));
        // A later, larger snapshot replaces it.
        assertEquals(30, DSLogFiles.receive(bytes(30, 2), 30, folder, LOG, () -> false));
        byte[] stored = Files.readAllBytes(folder.toPath().resolve(LOG));
        assertEquals(30, stored.length);
        assertEquals(2, stored[0]);
        assertFalse(Files.exists(folder.toPath().resolve(LOG + DSLogFiles.PART_SUFFIX)));
    }

    @Test
    void shortBodyLeavesTheOldFileAndNoPart() throws IOException {
        File folder = dir.toFile();
        DSLogFiles.receive(bytes(5, 1), 5, folder, LOG, () -> false);
        assertThrows(
                EOFException.class,
                () -> DSLogFiles.receive(bytes(5, 2), 50, folder, LOG, () -> false));
        assertArrayEquals(new byte[] {1, 1, 1, 1, 1}, Files.readAllBytes(dir.resolve(LOG)));
        assertFalse(Files.exists(dir.resolve(LOG + DSLogFiles.PART_SUFFIX)));
    }

    @Test
    void abortStopsTheUpload() {
        assertThrows(
                DSLogFiles.AbortedException.class,
                () -> DSLogFiles.receive(bytes(5, 1), 5, dir.toFile(), LOG, () -> true));
        assertFalse(Files.exists(dir.resolve(LOG)));
        assertFalse(Files.exists(dir.resolve(LOG + DSLogFiles.PART_SUFFIX)));
    }

    @Test
    void rejectsBadNamesBeforeTouchingDisk() {
        assertThrows(
                IllegalArgumentException.class,
                () -> DSLogFiles.receive(bytes(1, 0), 1, dir.toFile(), "../evil", () -> false));
    }

    private void file(String name, int size, long modified) throws IOException {
        Path p = dir.resolve(name);
        Files.write(p, new byte[size]);
        assertTrue(p.toFile().setLastModified(modified));
    }

    @Test
    void trimDeletesOldestOwnFilesOnly() throws IOException {
        file("2026_10_01 10_00_00 Thu.dslog", 100, 1_000_000L);
        file("2026_10_01 11_00_00 Thu.dslog", 100, 2_000_000L);
        file("2026_10_01 12_00_00 Thu.dslog", 100, 3_000_000L);
        file("notes.txt", 1000, 500_000L);
        assertEquals(1, DSLogFiles.trim(dir.toFile(), 250, null));
        assertFalse(Files.exists(dir.resolve("2026_10_01 10_00_00 Thu.dslog")));
        assertTrue(Files.exists(dir.resolve("2026_10_01 11_00_00 Thu.dslog")));
        assertTrue(Files.exists(dir.resolve("notes.txt")));
    }

    @Test
    void trimKeepsTheNameBeingWritten() throws IOException {
        file("2026_10_01 10_00_00 Thu.dslog", 100, 1_000_000L);
        file("2026_10_01 11_00_00 Thu.dslog", 100, 2_000_000L);
        DSLogFiles.trim(dir.toFile(), 0, "2026_10_01 10_00_00 Thu.dslog");
        assertTrue(Files.exists(dir.resolve("2026_10_01 10_00_00 Thu.dslog")));
        assertFalse(Files.exists(dir.resolve("2026_10_01 11_00_00 Thu.dslog")));
    }

    @Test
    void listsAndCleansParts() throws IOException {
        file(LOG, 7, 1_000_000L);
        file(EVENTS + DSLogFiles.PART_SUFFIX, 3, 1_000_000L);
        file("other.part", 3, 1_000_000L);
        List<DSLogFiles.Entry> entries = DSLogFiles.list(dir.toFile());
        assertEquals(1, entries.size());
        assertEquals(LOG, entries.get(0).name());
        assertEquals(7, entries.get(0).size());
        assertEquals(
                "[{\"name\":\"" + LOG + "\",\"size\":7,\"mtime\":1000000}]",
                DSLogFiles.toJson(entries));
        assertEquals(1, DSLogFiles.cleanParts(dir.toFile()));
        assertTrue(Files.exists(dir.resolve("other.part")));
    }

    @Test
    void choosesUsbOnlyWhenItIsThere() {
        File usb = dir.resolve("U").toFile();
        File local = dir.resolve("local").toFile();
        assertEquals(local, DSLogFiles.chooseLogDir(usb.getPath(), local.getPath()));
        assertTrue(usb.mkdirs());
        assertEquals(usb, DSLogFiles.chooseLogDir(usb.getPath(), local.getPath()));
    }
}
