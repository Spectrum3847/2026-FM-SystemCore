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
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Writes {@link MatchSummary}'s JSON next to the log being written, off the main thread.
 *
 * <p>The file is {@code <log name>.summary.json} ({@code akit_..._q34.wpilog} gets {@code
 * akit_..._q34.summary.json}), where the log is this boot's: a {@code .wpilog} in the folder that
 * was not there when {@link LogStorage#cleanUpAtBoot()} listed it, under the name AdvantageKit gave
 * it once the match was known. Not the newest by modification time: SystemCore can boot with its
 * clock behind older logs. Where nothing was listed at boot (the simulator) it is the newest.
 *
 * <p>It is written to a temporary file, synced, renamed over the old one, and the folder synced, so
 * a reader (or a power cut) sees either the whole previous file or the whole new one.
 */
final class SummaryFileWriter {
    static final String SUFFIX = ".summary.json";

    private static final boolean WINDOWS =
            System.getProperty("os.name", "")
                    .toLowerCase(java.util.Locale.ROOT)
                    .startsWith("windows");

    private final File folder;
    private final ThreadPoolExecutor executor;
    private volatile String lastResult = "";

    /**
     * Starts the writer thread now. Call during robot init: a thread started later from the main
     * thread inherits its real-time priority.
     */
    SummaryFileWriter(File folder) {
        this.folder = folder;
        executor =
                new ThreadPoolExecutor(
                        1,
                        1,
                        0,
                        TimeUnit.SECONDS,
                        new LinkedBlockingQueue<>(),
                        runnable -> {
                            Thread t = new Thread(runnable, "MatchSummary");
                            t.setDaemon(true);
                            return t;
                        });
        executor.prestartAllCoreThreads();
    }

    /** Queues a write. Main thread; returns at once. */
    void submit(String json) {
        executor.execute(
                () -> {
                    try {
                        File log = thisBootsLog(folder, LogStorage.logsAtBoot());
                        if (log == null) {
                            lastResult = "no .wpilog from this boot in " + folder;
                            return;
                        }
                        File target = summaryFor(log);
                        boolean synced = writeAtomically(target.toPath(), json);
                        lastResult = target.getName() + (synced ? "" : " (folder sync failed)");
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

    /**
     * This boot's log in a folder, or null. Replay outputs excluded.
     *
     * @param folder the log folder
     * @param logsAtBoot the log names in it before this boot's log was opened, or null if unknown
     * @return the newest {@code .wpilog} not in {@code logsAtBoot}
     */
    static File thisBootsLog(File folder, Set<String> logsAtBoot) {
        File[] logs =
                folder.listFiles(
                        f ->
                                f.isFile()
                                        && f.getName().endsWith(".wpilog")
                                        && !f.getName().endsWith("_sim.wpilog")
                                        && (logsAtBoot == null
                                                || !logsAtBoot.contains(f.getName())));
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
     * Writes a temporary file beside the target, syncs it to disk, renames it over the target, then
     * syncs the folder so the rename itself survives a power cut. A failed write leaves no
     * temporary file behind.
     *
     * @return false if syncing the folder was tried and failed (it is not tried on Windows, which
     *     cannot open a folder to sync it)
     */
    static boolean writeAtomically(Path target, String content) throws IOException {
        Path tmp = target.resolveSibling(target.getFileName() + ".tmp");
        try {
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
        } catch (IOException | RuntimeException e) {
            try {
                Files.deleteIfExists(tmp);
            } catch (IOException | RuntimeException cleanup) {
                e.addSuppressed(cleanup);
            }
            throw e;
        }
        return syncFolder(target.toAbsolutePath().getParent());
    }

    /** Best effort: the file is already in place, so a failure here is only reported. */
    private static boolean syncFolder(Path folder) {
        if (WINDOWS || folder == null) {
            return true;
        }
        try (FileChannel dir = FileChannel.open(folder, StandardOpenOption.READ)) {
            dir.force(true);
            return true;
        } catch (IOException | RuntimeException e) {
            return false;
        }
    }
}
