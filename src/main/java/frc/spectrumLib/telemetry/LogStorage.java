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
 * <p>A full internal disk is the SystemCore failure other teams have hit at events: a robot that
 * rebooted mid-match with an empty log, another that lost control with 50 MB left, deploys that
 * deleted the robot program (wpilibsuite/SystemcoreTesting #210, #211, #156). Every log goes to
 * internal storage when no USB stick is in, and nothing else ever deletes them; the bench unit had
 * 408 MB of them after two days. So:
 *
 * <ul>
 *   <li>{@link #chooseFolder()} waits briefly for a USB stick to mount before settling on internal
 *       storage: the robot program can start before {@code /U} is mounted (SystemcoreTesting #341).
 *       It only waits when a USB disk is actually attached, so a boot with no stick does not pay
 *       the wait.
 *   <li>{@link #cleanUpAtBoot()} deletes the oldest logs until the folder is under {@link
 *       #LOCAL_CAP_BYTES} and the disk has {@link #FREE_FLOOR_BYTES} free. Call it between {@link
 *       #chooseFolder()} and {@code Logger.start()}, so it runs before this boot's log exists and
 *       cannot delete it. It also remembers which logs were already there.
 *   <li>{@link #start} (after {@code Logger.start()}) checks free space every {@link
 *       #CHECK_PERIOD_SECONDS} on its own low-priority thread (disk calls stay off the real-time
 *       main thread) and trims the internal folder if the disk runs short; {@link #periodic()}
 *       turns the result into alerts and log values on the main thread.
 * </ul>
 *
 * <p>The log being written is never deleted, whatever the clock says. "Oldest" is by modification
 * time, and SystemCore can boot with its clock behind the previous logs' times, so this boot's log
 * could sort oldest; keeping "the newest two" did not protect it. Instead a file is never deleted
 * if it appeared after {@link #cleanUpAtBoot()} (this boot's AdvantageKit log, including after
 * AdvantageKit renames it once the match is known) or if it was modified within {@link
 * #ACTIVE_WINDOW_MILLIS} of now on the same clock that stamps it (anything still being written).
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

    /**
     * A log modified within this long of now (either side, for a clock that has jumped) is taken to
     * be still being written and is never deleted. Every open log is written far more often.
     */
    static final long ACTIVE_WINDOW_MILLIS = 5 * 60 * 1000L;

    /** Where USB disks show up; see {@link #usbDiskAttached()}. */
    private static final String SYS_BLOCK = "/sys/block";

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

    /** Log file names in {@link #folder} after the boot cleanup; null until it has run. */
    private static volatile Set<String> bootLogs;

    /** {count, bytes} the boot cleanup deleted, printed by {@link #start}. */
    private static long[] bootDeleted = {0, 0};

    /** Set by the background check when it throws; reported on the main thread. */
    private static volatile String checkError;

    private static boolean checkErrorReported;

    /**
     * The folder to log to: the USB stick if one is (or within {@link #USB_WAIT_SECONDS} becomes)
     * mounted, else internal storage. Blocks at most that long, and only when {@code /U} exists and
     * a USB disk is attached (one that could still mount).
     *
     * @return the log folder, created if needed
     */
    public static String chooseFolder() {
        long deadline = System.nanoTime() + (long) (USB_WAIT_SECONDS * 1e9);
        boolean usb = usbMounted();
        boolean couldMount = !usb && new File(USB_MOUNT).isDirectory() && usbDiskAttached();
        while (!usb && couldMount && System.nanoTime() < deadline) {
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
     * Whether a USB disk is attached, mounted or not: a SCSI-style disk ({@code sd*}) in {@code
     * /sys/block}, which is how Linux names USB mass storage (SystemCore's own storage is not one).
     * Enumeration comes well before the mount, so with no such disk there is nothing to wait for.
     * If {@code /sys/block} cannot be read, says yes, keeping the old always-wait behavior.
     */
    private static boolean usbDiskAttached() {
        String[] blocks = new File(SYS_BLOCK).list();
        if (blocks == null) {
            return true;
        }
        for (String b : blocks) {
            if (b.startsWith("sd")) {
                return true;
            }
        }
        return false;
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
     * One log file, with its size and modification time read once. Sorting {@link File}s by {@code
     * File::lastModified} re-read the times during the sort, and the log being written changes its
     * time, which can break the sort's contract ("Comparison method violates its general
     * contract").
     *
     * @param name the file name
     * @param bytes its size
     * @param modifiedMillis its modification time
     */
    record LogFile(String name, long bytes, long modifiedMillis) {}

    /**
     * Which logs to delete, oldest first: until they total at most {@code capBytes} and the disk
     * has at least {@code freeFloorBytes} free (each deletion counted as freeing its size), never a
     * protected or recently modified file, and keeping the newest {@code keepNewest} of the rest.
     * Pure, for tests.
     *
     * @param logs the log files in the folder
     * @param capBytes most the logs may total
     * @param freeBytes free space on the disk now
     * @param freeFloorBytes free space to leave on the disk
     * @param keepNewest how many of the newest deletable logs are kept anyway
     * @param protectedNames files never deleted (this boot's logs)
     * @param nowMillis the current time, on the clock that stamps the files
     * @return the names to delete, oldest first
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
     * disk has at least {@code freeFloorBytes} free; see {@link #selectForDeletion}.
     *
     * @param dir the log folder
     * @param capBytes most the logs may total
     * @param freeFloorBytes free space to leave on the disk
     * @param keepNewest how many of the newest deletable logs are never deleted
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

    /**
     * {@link #cleanUp(File, long, long, int, Set)} with nothing protected but recently modified
     * files.
     */
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
     * Cleans the internal log folder (always: logs land there whenever the stick is out) and
     * remembers what logs {@link #folder} holds, so anything that appears later is this boot's and
     * is never deleted. Call once at boot on the real robot, after {@link #chooseFolder()} and
     * before {@code Logger.start()}, so this boot's log does not exist yet.
     */
    public static void cleanUpAtBoot() {
        bootDeleted = cleanUp(new File(LOCAL_LOGS), LOCAL_CAP_BYTES, FREE_FLOOR_BYTES, 2);
        Set<String> names = new HashSet<>();
        for (LogFile f : list(new File(folder))) {
            names.add(f.name());
        }
        bootLogs = names;
    }

    /**
     * Reports the boot cleanup and starts the background free-space check. Call once at boot on the
     * real robot, after {@code Logger.start()} (and after {@link #cleanUpAtBoot()}).
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
        checker.scheduleWithFixedDelay(
                LogStorage::checkSafely,
                0,
                (long) (CHECK_PERIOD_SECONDS * 1000),
                TimeUnit.MILLISECONDS);
    }

    /**
     * {@link #check()}, never throwing: an exception out of a {@code scheduleWithFixedDelay} task
     * silently cancels every later run, which would freeze the free-space value and alerts. Not
     * logged here (Logger is main-thread only): {@link #periodic()} reports it.
     */
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
            // Never this boot's logs (anything not there at the boot cleanup) or anything still
            // being written, whatever order the clock puts them in.
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
            Telemetry.logDash("System/Storage/FreeMB", free / 1e6);
            Telemetry.log("System/Storage/LogsMB", logBytes / 1e6);
        }
    }

    /** The folder chosen at boot. */
    public static String folder() {
        return folder;
    }
}
