package frc.spectrumLib.telemetry;

import java.io.File;
import java.net.InetSocketAddress;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * A drop box for the Driver Station's own logs: the DS laptop's MatchReview helper PUTs a match's
 * {@code .dslog} and {@code .dsevents} to the robot right after the match, while the field network
 * is still up, and they wait in {@code <logdir>/ds/} for the pit laptop to pull them over SFTP with
 * the robot's wpilogs. See {@code docs/tools/ds-log-receiver.md}.
 *
 * <ul>
 *   <li>{@code PUT /dslog/<name>}: store one file (name, token, disabled, size checks; 201).
 *   <li>{@code GET /dslog/}: list the stored files.
 *   <li>{@code GET /health}: whether it is up, the robot is enabled, and where files go.
 * </ul>
 *
 * <p>Robot safety: the server runs on its own low-priority daemon threads, started once from {@code
 * robotInit()} on the real robot only. It never calls WPILib or HAL; the only link to the main loop
 * is a volatile {@code enabled} flag that {@link #periodic(boolean)} writes each loop and the
 * server reads. Uploads are refused while enabled (and one in progress is abandoned if the robot
 * enables). Results are queued and logged by {@link #periodic(boolean)} on the main thread. Every
 * failure, including a JVM without {@code jdk.httpserver}, is caught and logged; the robot program
 * carries on without the receiver.
 */
public final class DSLogReceiver {
    private DSLogReceiver() {}

    /** Settings. Must match the MatchReview helper on the DS laptop. */
    public static final class Constants {
        private Constants() {}

        /** Master switch. false: {@link #start()} does nothing and the port stays closed. */
        public static final boolean ENABLED = true;

        /**
         * Port, all interfaces. The field firewall passes only 5800-5810 to the robot. 5800 is
         * WPILib's deploy-folder WebServer; Limelight USB port forwarding (unused here) would take
         * 5800-5809.
         */
        public static final int PORT = 5806;

        /**
         * Shared token MatchReview sends in {@link #TOKEN_HEADER}. This is only a guard against
         * stray traffic on the closed field network (another tool or a port scan writing files): it
         * is in a public repo and travels over plain HTTP, so it is not security.
         */
        public static final String TOKEN = "spectrum-ds-3847";

        /** Request header that carries {@link #TOKEN}. */
        public static final String TOKEN_HEADER = "X-MatchReview-Token";

        /** Largest file accepted (a match's .dslog is a few hundred KB). */
        public static final long MAX_FILE_BYTES = 25L * 1024 * 1024;

        /** Most the {@code ds/} folder may hold; the oldest files are deleted to stay under it. */
        public static final long MAX_FOLDER_BYTES = 500L * 1024 * 1024;

        /** Free space an upload must leave on the disk, so DS logs never starve the wpilog. */
        public static final long MIN_FREE_BYTES = 200L * 1024 * 1024;

        /** Log folder when a USB stick is in (DogLog's choice too). */
        public static final String USB_LOGS = "/U/logs";

        /** Log folder otherwise. */
        public static final String LOCAL_LOGS = "/home/systemcore/logs";

        /** Folder under the log folder that holds the DS files. */
        public static final String SUBFOLDER = "ds";

        /**
         * Seconds a request (headers and body) may take before the JDK server drops it. The field
         * can cut the network mid-upload; without this the dead connection would hold the one
         * request thread until reboot. 25 MB at the field's 4 Mbit/s is about 50 s.
         */
        static final int MAX_REQUEST_SECONDS = 120;

        /** Seconds the client may take to read a response. */
        static final int MAX_RESPONSE_SECONDS = 30;

        /** Results waiting for the main loop; more than this (a flood) are dropped. */
        static final int EVENT_QUEUE = 64;

        /** Results logged per loop, so a backlog never costs one loop much. */
        static final int EVENTS_PER_LOOP = 4;
    }

    private enum Kind {
        STARTED,
        RECEIVED,
        REJECTED,
        FAILED
    }

    private record Event(Kind kind, String name, long size, String message) {}

    /** Written by the main loop, read by the server. Starts true: refuse until a loop has run. */
    private static volatile boolean robotEnabled = true;

    private static final ArrayBlockingQueue<Event> events =
            new ArrayBlockingQueue<>(Constants.EVENT_QUEUE);
    private static final AtomicInteger dropped = new AtomicInteger();

    // Main thread only.
    private static boolean started;
    private static int received;
    private static int rejected;

    /**
     * Starts the receiver on its own thread, once. Call from {@code robotInit()} on the real robot.
     * Returns at once; binding the port and any failure happen on that thread and are reported
     * through {@link #periodic(boolean)}.
     */
    public static void start() {
        if (!Constants.ENABLED || started) {
            return;
        }
        started = true;
        try {
            Thread t = lowPriorityDaemon("DSLogReceiver-start").newThread(DSLogReceiver::run);
            t.start();
        } catch (Throwable t) {
            offer(new Event(Kind.FAILED, "", 0, "could not start: " + t));
        }
    }

    /**
     * Updates the enabled flag the server reads and logs what the server did since the last call.
     * Call once per loop from {@code robotPeriodic()}; it never throws.
     *
     * @param enabled {@code DriverStation.isEnabled()}
     */
    public static void periodic(boolean enabled) {
        robotEnabled = enabled;
        if (!started) {
            return;
        }
        try {
            for (int i = 0; i < Constants.EVENTS_PER_LOOP; i++) {
                Event e = events.poll();
                if (e == null) {
                    break;
                }
                log(e);
            }
        } catch (Throwable t) {
            // logging must never take the loop down
        }
    }

    /** The folder files go to now: the USB stick if it is in and writable, else internal. */
    public static File logDir() {
        return DSLogFiles.chooseLogDir(Constants.USB_LOGS, Constants.LOCAL_LOGS);
    }

    private static void log(Event e) {
        switch (e.kind()) {
            case STARTED -> {
                Telemetry.log("DSLogReceiver/Running", true);
                Telemetry.print("DSLogReceiver: " + e.message());
            }
            case RECEIVED -> {
                received++;
                Telemetry.log("DSLogReceiver/LastFile", e.name());
                Telemetry.log("DSLogReceiver/LastSize", e.size());
                Telemetry.log("DSLogReceiver/Count", received);
                Telemetry.print("DSLogReceiver: received " + e.name() + " (" + e.size() + " B)");
            }
            case REJECTED -> {
                rejected++;
                Telemetry.log("DSLogReceiver/LastError", e.message());
                Telemetry.log("DSLogReceiver/Rejected", rejected);
                int lost = dropped.getAndSet(0);
                if (lost > 0) {
                    Telemetry.log("DSLogReceiver/Dropped", lost);
                }
                Telemetry.print("DSLogReceiver: refused " + e.size() + ": " + e.message());
            }
            case FAILED -> {
                Telemetry.log("DSLogReceiver/Running", false);
                Telemetry.log("DSLogReceiver/LastError", e.message());
                Telemetry.print("DSLogReceiver: " + e.message(), Telemetry.PrintPriority.HIGH);
            }
        }
    }

    private static void offer(Event e) {
        if (!events.offer(e)) {
            dropped.incrementAndGet();
        }
    }

    /** On the start thread: bind and serve. The server's own threads inherit its daemon status. */
    private static void run() {
        try {
            // JDK server timeouts; read once when its classes load, so set them first.
            System.setProperty(
                    "sun.net.httpserver.maxReqTime",
                    System.getProperty(
                            "sun.net.httpserver.maxReqTime",
                            Integer.toString(Constants.MAX_REQUEST_SECONDS)));
            System.setProperty(
                    "sun.net.httpserver.maxRspTime",
                    System.getProperty(
                            "sun.net.httpserver.maxRspTime",
                            Integer.toString(Constants.MAX_RESPONSE_SECONDS)));
            File dsDir = new File(logDir(), Constants.SUBFOLDER);
            DSLogFiles.cleanParts(dsDir);
            DSLogHttpServer server =
                    new DSLogHttpServer(
                            new InetSocketAddress(Constants.PORT),
                            DSLogReceiver::logDir,
                            Constants.SUBFOLDER,
                            Constants.TOKEN,
                            Constants.TOKEN_HEADER,
                            Constants.MAX_FILE_BYTES,
                            Constants.MAX_FOLDER_BYTES,
                            Constants.MIN_FREE_BYTES,
                            () -> robotEnabled,
                            new DSLogHttpServer.Listener() {
                                @Override
                                public void received(String name, long size) {
                                    offer(new Event(Kind.RECEIVED, name, size, ""));
                                }

                                @Override
                                public void rejected(int status, String reason) {
                                    offer(new Event(Kind.REJECTED, "", status, reason));
                                }
                            },
                            lowPriorityDaemon("DSLogReceiver"));
            server.start();
            offer(
                    new Event(
                            Kind.STARTED,
                            "",
                            0,
                            "listening on port " + Constants.PORT + ", files to " + dsDir));
        } catch (Throwable t) {
            offer(new Event(Kind.FAILED, "", 0, "could not start: " + t));
        }
    }

    static ThreadFactory lowPriorityDaemon(String name) {
        return runnable -> {
            Thread t = new Thread(runnable, name);
            t.setDaemon(true);
            t.setPriority(Thread.MIN_PRIORITY);
            return t;
        };
    }
}
