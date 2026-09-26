package frc.spectrumLib.telemetry;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.wpilib.driverstation.Alert.Level;

/**
 * Where the robot's logs go on SystemCore, and keeping them from filling its storage.
 *
 * <p>A full internal disk is the SystemCore failure other teams have hit at events: a robot that
 * rebooted mid-match with an empty log, another that lost control with 50 MB left, deploys that
 * deleted the robot program (wpilibsuite/SystemcoreTesting #210, #211, #156). Every log goes to
 * internal storage when no USB stick is in, and nothing else ever deletes them; the bench unit had
 * 408 MB of them after two days. So:
 *
 * <ul>
 *   <li>{@link #chooseFolder()} waits briefly for a USB stick to mount before settling on internal
 *       storage: the robot program can start before {@code /U} is mounted (SystemcoreTesting #341).
 *   <li>{@link #cleanUp} deletes the oldest logs until the folder is under {@link #LOCAL_CAP_BYTES}
 *       and the disk has {@link #FREE_FLOOR_BYTES} free. Run at boot, before the new log opens.
 *   <li>{@link #start} checks free space every {@link #CHECK_PERIOD_SECONDS} on its own
 *       low-priority thread (disk calls stay off the real-time main thread) and {@link #periodic()}
 *       turns the result into alerts and log values on the main thread.
 * </ul>
 */
public final class LogStorage {
    private LogStorage() {}

    public static final String USB_MOUNT = "/U";
    public static final String USB_LOGS = "/U/logs";
    public static final String LOCAL_LOGS = "/home/systemcore/logs";

    /** How long boot waits for a USB stick that is present but not mounted yet. */
    static final double USB_WAIT_SECONDS = 3.0;

    /** Most internal storage this program's logs may use. A match log is ~50-150 MB. */
    public static final long LOCAL_CAP_BYTES = 2_000_000_000L;

    /** Free space cleanup keeps on the disk holding the logs. */
    public static final long FREE_FLOOR_BYTES = 1_000_000_000L;

    /** Below this much free space the storage alert goes red. */
    public static final long FREE_ALERT_BYTES = 500_000_000L;

    static final double CHECK_PERIOD_SECONDS = 10.0;

    /** Log files this program (and Phoenix, REV) write; nothing else in the folder is touched. */
    private static final String[] LOG_SUFFIXES = {".wpilog", ".hoot", ".revlog"};

    private static final Alert internalAlert =
            new Alert("Logging to internal storage (no USB stick)", Level.LOW);
    private static final Alert lowSpaceAlert = new Alert("", Level.HIGH);

    private static volatile String folder = LOCAL_LOGS;
    private static volatile long freeBytes = -1;
    private static volatile long logBytes = -1;
    private static ScheduledExecutorService checker;

    /**
     * The folder to log to: the USB stick if one is (or within {@link #USB_WAIT_SECONDS} becomes)
     * mounted, else internal storage. Blocks at most that long, and only when {@code /U} exists.
     *
     * @return the log folder, created if needed
     */
    public static String chooseFolder() {
        long deadline = System.nanoTime() + (long) (USB_WAIT_SECONDS * 1e9);
        boolean usb = usbMounted();
        while (!usb && new File(USB_MOUNT).isDirectory() && System.nanoTime() < deadline) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            usb = usbMounted();
        }
        File dir = new File(usb ? USB_LOGS : LOCAL_LOGS);
        dir.mkdirs();
        folder = dir.getPath();
        internalAlert.set(!usb);
        return folder;
    }

    /**
     * Whether a filesystem is mounted at {@code /U} and writable (not just the empty mountpoint).
     */
    private static boolean usbMounted() {
        try {
            for (String line : Files.readAllLines(new File("/proc/mounts").toPath())) {
                String[] f = line.split(" ");
                if (f.length > 1 && f[1].equals(USB_MOUNT)) {
                    return new File(USB_MOUNT).canWrite();
                }
            }
        } catch (IOException | RuntimeException e) {
            // not Linux, or /proc unreadable: fall back to the plain check
            return new File(USB_MOUNT).isDirectory() && new File(USB_MOUNT).canWrite();
        }
        return false;
    }

    /**
     * Deletes the oldest log files in {@code dir} until they total at most {@code capBytes} and the
     * disk has at least {@code freeFloorBytes} free, keeping the newest {@code keepNewest}.
     *
     * @param dir the log folder
     * @param capBytes most the logs may total
     * @param freeFloorBytes free space to leave on the disk
     * @param keepNewest how many of the newest logs are never deleted
     * @return {count deleted, bytes deleted}
     */
    public static long[] cleanUp(File dir, long capBytes, long freeFloorBytes, int keepNewest) {
        File[] listed = dir.listFiles(f -> f.isFile() && isLogFile(f.getName()));
        if (listed == null || listed.length == 0) {
            return new long[] {0, 0};
        }
        List<File> logs = new ArrayList<>(Arrays.asList(listed));
        logs.sort(Comparator.comparingLong(File::lastModified)); // oldest first
        long total = 0;
        for (File f : logs) {
            total += f.length();
        }
        long deleted = 0;
        long deletedBytes = 0;
        while (logs.size() > keepNewest
                && (total > capBytes || dir.getUsableSpace() < freeFloorBytes)) {
            File oldest = logs.remove(0);
            long size = oldest.length();
            if (oldest.delete()) {
                total -= size;
                deleted++;
                deletedBytes += size;
            }
        }
        return new long[] {deleted, deletedBytes};
    }

    static boolean isLogFile(String name) {
        for (String suffix : LOG_SUFFIXES) {
            if (name.endsWith(suffix)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Cleans the internal log folder (always: logs land there whenever the stick is out) and starts
     * the background free-space check. Call once at boot on the real robot, before the log opens.
     */
    public static void start() {
        long[] r = cleanUp(new File(LOCAL_LOGS), LOCAL_CAP_BYTES, FREE_FLOOR_BYTES, 2);
        if (r[0] > 0) {
            Telemetry.print(
                    String.format(
                            "LogStorage: deleted %d old logs (%.0f MB) from %s",
                            r[0], r[1] / 1e6, LOCAL_LOGS));
        }
        checker =
                Executors.newSingleThreadScheduledExecutor(
                        runnable -> {
                            Thread t = new Thread(runnable, "LogStorage");
                            t.setDaemon(true);
                            t.setPriority(Thread.MIN_PRIORITY);
                            return t;
                        });
        checker.scheduleWithFixedDelay(
                LogStorage::check, 0, (long) (CHECK_PERIOD_SECONDS * 1000), TimeUnit.MILLISECONDS);
    }

    /** On the LogStorage thread: measure, and trim the internal folder if the disk runs short. */
    private static void check() {
        File dir = new File(folder);
        long free = dir.getUsableSpace();
        if (free < FREE_FLOOR_BYTES && folder.equals(LOCAL_LOGS)) {
            // Keeps the newest 2: the log being written is always among them.
            cleanUp(dir, LOCAL_CAP_BYTES, FREE_FLOOR_BYTES, 2);
            free = dir.getUsableSpace();
        }
        File[] listed = dir.listFiles(f -> f.isFile() && isLogFile(f.getName()));
        long total = 0;
        if (listed != null) {
            for (File f : listed) {
                total += f.length();
            }
        }
        logBytes = total;
        freeBytes = free;
    }

    /** Alerts and log values from the last background check. Call once per loop; cheap. */
    public static void periodic() {
        long free = freeBytes;
        if (free < 0) {
            return; // no check yet (or not started: sim, replay)
        }
        lowSpaceAlert.setText(
                String.format("Robot storage low: %.0f MB free (%s)", free / 1e6, folder));
        lowSpaceAlert.set(free < FREE_ALERT_BYTES);
        if (Telemetry.slowLogThisLoop()) {
            Telemetry.logDash("System/Storage/FreeMB", free / 1e6);
            Telemetry.log("System/Storage/LogsMB", logBytes / 1e6);
        }
    }

    /** The folder chosen at boot. */
    public static String folder() {
        return folder;
    }
}
