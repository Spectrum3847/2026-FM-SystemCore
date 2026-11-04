package frc.spectrumLib.telemetry;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.wpilib.driverstation.Alert.Level;

/**
 * Where the robot's logs go on SystemCore, and keeping them from filling its storage.
 *
 * <p>A full internal disk is a SystemCore failure other teams have hit at events: a robot that
 * rebooted mid-match with an empty log, another that lost control with 50 MB left, deploys that
 * deleted the robot program (wpilibsuite/SystemcoreTesting #210, #211, #156). Every log goes to
 * internal storage when no USB stick is in, and nothing else ever deletes them; the bench unit had
 * 408 MB of them after two days. So:
 *
 * <ul>
 *   <li>{@link #chooseFolder()} waits briefly for a USB stick to mount before settling on internal
 *       storage: the robot program can start before {@code /U} is mounted (SystemcoreTesting #341).
 *   <li>{@link #cleanUpAtBoot()} deletes the oldest logs before this boot's log exists.
 *   <li>{@link #start} checks free space off the main thread and trims the internal folder if the
 *       disk runs short; {@link #periodic()} turns the result into alerts and log values on the
 *       main thread.
 * </ul>
 *
 * <p>SystemCore can boot with its clock behind older logs, so this boot's log can sort oldest. A
 * file is never deleted if it appeared after {@link #cleanUpAtBoot()} or was modified within {@link
 * #ACTIVE_WINDOW_MILLIS}.
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

    /** A log modified within this long of now, either side, is still being written. */
    static final long ACTIVE_WINDOW_MILLIS = 5 * 60 * 1000L;

    /** Log files this program (and Phoenix, REV) write; nothing else in the folder is touched. */
    private static final String[] LOG_SUFFIXES = {".wpilog", ".hoot", ".revlog"};

    private static final Alert internalAlert =
            new Alert("Logging to internal storage (no USB stick)", Level.LOW);
    private static final Alert lowSpaceAlert = new Alert("", Level.HIGH);
    private static final Alert checkFailedAlert = new Alert("", Level.MEDIUM);

    private static volatile String folder = LOCAL_LOGS;
    private static volatile long freeBytes = -1;
    private static volatile long logBytes = -1;
    private static ScheduledExecutorService checker;

    /** Log names present at the boot cleanup; null until it has run. */
    private static volatile Set<String> bootLogs;

    /** {count, bytes} the boot cleanup deleted. */
    private static long[] bootDeleted = {0, 0};

    /** Reported on the main thread. */
    private static volatile String checkError;

    private static boolean checkErrorReported;

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
     * A log file with its times read once, since the active log's time changing mid-sort breaks the
     * sort's contract.
     */
    record LogFile(String name, long bytes, long modifiedMillis) {}

    /**
     * Which logs to delete, oldest first, to meet {@code capBytes} and {@code freeFloorBytes}.
     * Never a protected or recently modified file. Pure, for tests.
     *
     * @param keepNewest how many of the newest deletable logs are kept anyway
     * @param nowMillis on the clock that stamps the files
     */
    static List<String> selectForDeletion(
            List<LogFile> logs,
            long capBytes,
            long freeBytes,
            long freeFloorBytes,
            int keepNewest,
            Set<String> protectedNames,
            long nowMillis) {
        long total = 0;
        List<LogFile> deletable = new ArrayList<>();
        for (LogFile f : logs) {
            total += f.bytes();
            boolean active = Math.abs(nowMillis - f.modifiedMillis()) < ACTIVE_WINDOW_MILLIS;
            if (!active && !protectedNames.contains(f.name())) {
                deletable.add(f);
            }
        }
        deletable.sort(
                Comparator.comparingLong(LogFile::modifiedMillis).thenComparing(LogFile::name));
        List<String> out = new ArrayList<>();
        long free = freeBytes;
        int i = 0;
        while (deletable.size() - i > keepNewest && (total > capBytes || free < freeFloorBytes)) {
            LogFile oldest = deletable.get(i++);
            out.add(oldest.name());
            total -= oldest.bytes();
            free = free > Long.MAX_VALUE - oldest.bytes() ? Long.MAX_VALUE : free + oldest.bytes();
        }
        return out;
    }

    private static List<LogFile> list(File dir) {
        File[] listed = dir.listFiles(f -> f.isFile() && isLogFile(f.getName()));
        List<LogFile> out = new ArrayList<>();
        if (listed != null) {
            for (File f : listed) {
                out.add(new LogFile(f.getName(), f.length(), f.lastModified()));
            }
        }
        return out;
    }

    /**
     * Deletes the oldest log files in {@code dir} until they total at most {@code capBytes} and the
     * disk has at least {@code freeFloorBytes} free.
     *
     * @param dir the log folder
     * @param capBytes most the logs may total
     * @param freeFloorBytes free space to leave on the disk
     * @param keepNewest how many of the newest logs are kept
     * @param protectedNames files never deleted
     * @return {count deleted, bytes deleted}
     */
    public static long[] cleanUp(
            File dir,
            long capBytes,
            long freeFloorBytes,
            int keepNewest,
            Set<String> protectedNames) {
        List<LogFile> logs = list(dir);
        if (logs.isEmpty()) {
            return new long[] {0, 0};
        }
        List<String> doomed =
                selectForDeletion(
                        logs,
                        capBytes,
                        dir.getUsableSpace(),
                        freeFloorBytes,
                        keepNewest,
                        protectedNames,
                        System.currentTimeMillis());
        long deleted = 0;
        long deletedBytes = 0;
        for (String name : doomed) {
            File f = new File(dir, name);
            long size = f.length();
            if (f.delete()) {
                deleted++;
                deletedBytes += size;
            }
        }
        return new long[] {deleted, deletedBytes};
    }

    public static long[] cleanUp(File dir, long capBytes, long freeFloorBytes, int keepNewest) {
        return cleanUp(dir, capBytes, freeFloorBytes, keepNewest, Collections.emptySet());
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
     * Cleans the internal log folder and remembers which logs exist, so later ones are protected.
     * Call before {@code Logger.start()}.
     */
    public static void cleanUpAtBoot() {
        bootDeleted = cleanUp(new File(LOCAL_LOGS), LOCAL_CAP_BYTES, FREE_FLOOR_BYTES, 2);
        Set<String> names = new HashSet<>();
        for (LogFile f : list(new File(folder))) {
            names.add(f.name());
        }
        bootLogs = Collections.unmodifiableSet(names);
    }

    /**
     * Reports the boot cleanup and starts the background free-space check. Call after {@code
     * Logger.start()}.
     */
    public static void start() {
        if (bootDeleted[0] > 0) {
            Telemetry.print(
                    String.format(
                            "LogStorage: deleted %d old logs (%.0f MB) from %s",
                            bootDeleted[0], bootDeleted[1] / 1e6, LOCAL_LOGS));
        }
        checker =
                Executors.newSingleThreadScheduledExecutor(
                        runnable -> {
                            Thread t = new Thread(runnable, "LogStorage");
                            t.setDaemon(true);
                            t.setPriority(Thread.MIN_PRIORITY);
                            return t;
                        });
        var _ =
                checker.scheduleWithFixedDelay(
                        LogStorage::checkSafely,
                        0,
                        (long) (CHECK_PERIOD_SECONDS * 1000),
                        TimeUnit.MILLISECONDS);
    }

    /** Never throws, since a throwing scheduled task silently cancels every later run. */
    private static void checkSafely() {
        try {
            check();
            checkError = null;
        } catch (Throwable t) {
            checkError = t.toString();
        }
    }

    /** On the LogStorage thread: measure, and trim the internal folder if the disk runs short. */
    private static void check() {
        File dir = new File(folder);
        long free = dir.getUsableSpace();
        if (free < FREE_FLOOR_BYTES && folder.equals(LOCAL_LOGS)) {
            Set<String> before = bootLogs;
            Set<String> thisBoot = new HashSet<>();
            for (LogFile f : list(dir)) {
                if (before == null || !before.contains(f.name())) {
                    thisBoot.add(f.name());
                }
            }
            cleanUp(dir, LOCAL_CAP_BYTES, FREE_FLOOR_BYTES, 2, thisBoot);
            free = dir.getUsableSpace();
        }
        long total = 0;
        for (LogFile f : list(dir)) {
            total += f.bytes();
        }
        logBytes = total;
        freeBytes = free;
    }

    /** Alerts and log values from the last background check. Call once per loop; cheap. */
    public static void periodic() {
        String error = checkError;
        if ((error != null) != checkErrorReported) {
            checkErrorReported = error != null;
            if (error != null) {
                checkFailedAlert.setText("Log storage check failed: " + error);
                Telemetry.print("LogStorage check failed: " + error, Telemetry.PrintPriority.HIGH);
            }
            checkFailedAlert.set(error != null);
        }
        long free = freeBytes;
        if (free < 0) {
            return; // no check yet (or not started: sim, replay)
        }
        lowSpaceAlert.setText(
                String.format("Robot storage low: %.0f MB free (%s)", free / 1e6, folder));
        lowSpaceAlert.set(free < FREE_ALERT_BYTES);
        if (Telemetry.slowLogThisLoop()) {
            // Free space where logs are written (USB stick or the controller).
            Telemetry.logDash("System/Storage/FreeMB", free / 1e6);
            // Space the logs in the log folder take.
            Telemetry.log("System/Storage/LogsMB", logBytes / 1e6);
        }
    }

    /** The folder chosen at boot. */
    public static String folder() {
        return folder;
    }

    /**
     * The log names in {@link #folder()} before this boot's log was opened, so a log not among them
     * is this boot's whatever its time; null where {@link #cleanUpAtBoot()} has not run (the sim).
     */
    static Set<String> logsAtBoot() {
        return bootLogs;
    }
}
