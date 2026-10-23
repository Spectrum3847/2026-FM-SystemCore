package frc.spectrumLib.telemetry;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Writes {@link MatchSummary}'s JSON next to the log being written, off the main thread.
 *
 * <p>The file is {@code <log name>.summary.json} ({@code akit_..._q34.wpilog} gets {@code
 * akit_..._q34.summary.json}), where the log is the newest {@code .wpilog} in the folder: the one
 * AdvantageKit is writing, under the name it gave it once the match was known. It is written to a
 * temporary file, synced, then renamed over the old one, so a reader (or a power cut) sees either
 * the whole previous file or the whole new one.
 */
final class SummaryFileWriter {
    static final String SUFFIX = ".summary.json";

    private final File folder;
    private ExecutorService executor;
    private volatile String lastResult = "";

    SummaryFileWriter(File folder) {
        this.folder = folder;
    }

    /** Queues a write. Main thread; returns at once. */
    void submit(String json) {
        if (executor == null) {
            executor =
                    Executors.newSingleThreadExecutor(
                            runnable -> {
                                Thread t = new Thread(runnable, "MatchSummary");
                                t.setDaemon(true);
                                return t;
                            });
        }
        executor.execute(
                () -> {
                    try {
                        File log = newestLog(folder);
                        if (log == null) {
                            lastResult = "no .wpilog in " + folder;
                            return;
                        }
                        File target = summaryFor(log);
                        writeAtomically(target.toPath(), json);
                        lastResult = target.getName();
                    } catch (Throwable t) {
                        lastResult = "failed: " + t;
                    }
                });
    }

    /** The last write's file name, or why it failed; empty before the first. */
    String lastResult() {
        return lastResult;
    }

    /** The summary file for a log: {@code x.wpilog} becomes {@code x.summary.json}. */
    static File summaryFor(File log) {
        String name = log.getName();
        String stem = name.endsWith(".wpilog") ? name.substring(0, name.length() - 7) : name;
        return new File(log.getParentFile(), stem + SUFFIX);
    }

    /** The most recently modified {@code .wpilog} in a folder, or null. Replay outputs excluded. */
    static File newestLog(File folder) {
        File[] logs =
                folder.listFiles(
                        f ->
                                f.isFile()
                                        && f.getName().endsWith(".wpilog")
                                        && !f.getName().endsWith("_sim.wpilog"));
        File newest = null;
        if (logs != null) {
            for (File f : logs) {
                if (newest == null || f.lastModified() > newest.lastModified()) {
                    newest = f;
                }
            }
        }
        return newest;
    }

    /**
     * Writes a temporary file beside the target, syncs it to disk, and renames it over the target.
     */
    static void writeAtomically(Path target, String content) throws IOException {
        Path tmp = target.resolveSibling(target.getFileName() + ".tmp");
        try (FileChannel ch =
                FileChannel.open(
                        tmp,
                        StandardOpenOption.CREATE,
                        StandardOpenOption.TRUNCATE_EXISTING,
                        StandardOpenOption.WRITE)) {
            ByteBuffer buf = ByteBuffer.wrap(content.getBytes(StandardCharsets.UTF_8));
            while (buf.hasRemaining()) {
                ch.write(buf);
            }
            ch.force(true);
        }
        try {
            Files.move(
                    tmp,
                    target,
                    StandardCopyOption.ATOMIC_MOVE,
                    StandardCopyOption.REPLACE_EXISTING);
        } catch (AtomicMoveNotSupportedException e) {
            // A USB stick's FAT filesystem: still a rename, just not guaranteed atomic.
            Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING);
        }
    }
}
