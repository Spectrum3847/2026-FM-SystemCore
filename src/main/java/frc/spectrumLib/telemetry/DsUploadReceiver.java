package frc.spectrumLib.telemetry;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.RandomAccessFile;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;
import java.util.regex.Pattern;
import org.wpilib.driverstation.Alert.Level;

/**
 * Receives files the Driver Station laptop pushes after a match: its DS log, and audio and video
 * recorded at the driver station. They land in {@code <log folder>/ds/}, and the pit laptop pulls
 * them with the match log.
 *
 * <p>Protocol v1 (the MatchReview helper's DS-laptop mode is the client), HTTP/1.1 on port {@value
 * #PORT}, header {@code X-MatchReview-Token} on every request:
 *
 * <ul>
 *   <li>{@code PUT /upload/<kind>/<name>?offset=<n>&total=<N>} appends one chunk.
 *   <li>{@code HEAD /upload/<kind>/<name>} returns {@code X-Received} (bytes so far, 0 for none)
 *       and {@code X-Complete}, so a push cut off by the robot powering down resumes at the next
 *       pit tether. A 404 here means robot code without this protocol.
 *   <li>{@code PUT /dslog/<name>} takes a whole NI DS log in one body (the helper's first version).
 * </ul>
 *
 * <p>The field gives each team about 7 Mbps, so a push only happens after the match, while the
 * robot is disabled; uploads are refused while it is enabled. The thread only writes files and
 * never touches robot state, so replay is unaffected. Counters reach the log through {@link
 * #periodic(boolean)} on the main thread.
 *
 * <p>A 2027 DS log is stored as {@code <name>.dswpilog}: pit helpers from before this protocol take
 * any {@code .wpilog} they find for a robot log and upload it to the public log site.
 */
public final class DsUploadReceiver {
    public static final int PORT = 5806;
    public static final String TOKEN_FILE = "/home/systemcore/ds-upload-token";
    public static final String TOKEN_HEADER = "x-matchreview-token";

    /** Pushed files, finished or partial, are trimmed above this. */
    static final long DIR_CAP_BYTES = 1_500_000_000L;

    private static final int MAX_HEADER_BYTES = 8192;
    private static final int SOCKET_TIMEOUT_MS = 15_000;
    // Each read restarts the socket timeout, so a client trickling bytes would hold the only
    // connection slot forever without a total limit. A 20 MB whole NI log at 7 Mbps takes ~25 s.
    static final long CONNECTION_LIMIT_MS = 120_000;
    // A refused PUT's body is read and dropped up to this, so the client sees the 409 or 401
    // instead of a reset connection. The helper sends 1 MiB chunks.
    private static final long DRAIN_LIMIT_BYTES = 32L << 20;
    private static final String NAME_CHARS = "[A-Za-z0-9 _.()-]{1,120}";

    /** What may be pushed, by name and size. */
    enum Kind {
        DSLOG(
                "^\\d{4}_\\d{2}_\\d{2} \\d{2}_\\d{2}_\\d{2} [A-Za-z]{3}\\.(dslog|dsevents)$",
                20_000_000L),
        DSWPILOG("^" + NAME_CHARS + "\\.wpilog$", 50_000_000L),
        AUDIO("^" + NAME_CHARS + "\\.(m4a|opus)$", 20_000_000L),
        VIDEO("^" + NAME_CHARS + "\\.mp4$", 200_000_000L);

        final Pattern name;
        final long maxBytes;

        Kind(String regex, long maxBytes) {
            this.name = Pattern.compile(regex);
            this.maxBytes = maxBytes;
        }

        static Kind parse(String s) {
            for (Kind k : values()) {
                if (k.name().toLowerCase(Locale.ROOT).equals(s)) {
                    return k;
                }
            }
            return null;
        }

        /** The file name on the robot for an uploaded name. */
        String diskName(String name) {
            return this == DSWPILOG
                    ? name.substring(0, name.length() - ".wpilog".length()) + ".dswpilog"
                    : name;
        }
    }

    /** An HTTP reply: status, extra headers and a small JSON body. */
    record Reply(int status, Map<String, String> headers, String body) {
        static Reply of(int status, String body) {
            return new Reply(status, Map.of(), body);
        }
    }

    private final File dir;
    private final byte[] token;
    private final long freeFloorBytes;
    private final long capBytes;
    private volatile boolean robotEnabled = true; // refuse until the main loop says otherwise
    private final AtomicLong filesCompleted = new AtomicLong();
    private final AtomicLong bytesReceived = new AtomicLong();
    private volatile String lastFile = "";
    private volatile String lastError = "";
    // In write order. The SystemCore can boot with its clock behind, so lastModified cannot tell
    // this boot's files (not pulled yet) from older ones.
    private final Set<String> writtenThisBoot = new LinkedHashSet<>();

    private static DsUploadReceiver instance;

    /** Created on first use, so {@link #main} runs without WPILib on the classpath. */
    private static final class Alerts {
        static final Alert OFF = new Alert("", Level.LOW);
    }

    DsUploadReceiver(File dir, String token, long freeFloorBytes) {
        this(dir, token, freeFloorBytes, DIR_CAP_BYTES);
    }

    DsUploadReceiver(File dir, String token, long freeFloorBytes, long capBytes) {
        this.dir = dir;
        this.token = token.trim().getBytes(StandardCharsets.UTF_8);
        this.freeFloorBytes = freeFloorBytes;
        this.capBytes = capBytes;
    }

    /**
     * Runs a receiver on a laptop, for bench tests and the MatchReview client's tests: {@code
     * <folder> <token> <port>} (port 0 picks one). Prints {@code listening <port>}. Lines on stdin:
     * {@code enabled} / {@code disabled} set the robot state (it starts disabled).
     *
     * @param args folder, token, port
     * @throws IOException if the port cannot be opened
     */
    public static void main(String[] args) throws IOException {
        File dir = new File(args[0]);
        if (!dir.isDirectory() && !dir.mkdirs()) {
            throw new IOException("cannot create " + dir);
        }
        DsUploadReceiver r = new DsUploadReceiver(dir, args[1], 0);
        r.robotEnabled = false;
        ServerSocket server = new ServerSocket();
        server.setReuseAddress(true);
        server.bind(new InetSocketAddress(Integer.parseInt(args[2])));
        System.out.println("listening " + server.getLocalPort());
        System.out.flush();
        Thread t = new Thread(() -> r.serve(server), "DsUploadReceiver");
        t.setDaemon(true);
        t.start();
        BufferedReader stdin =
                new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
        for (String line; (line = stdin.readLine()) != null; ) {
            r.robotEnabled = line.trim().equals("enabled");
            System.out.println("robot " + (r.robotEnabled ? "enabled" : "disabled"));
            System.out.flush();
        }
    }

    /**
     * Starts the receiver on a daemon thread, writing to {@code <logFolder>/ds}. Does nothing (and
     * raises an info alert) when the token file is missing. Call during robot init, before the main
     * thread goes real-time, so the thread stays at normal priority.
     *
     * @param logFolder the folder the robot logs to
     */
    public static synchronized void start(String logFolder) {
        if (instance != null) {
            return;
        }
        String token;
        try {
            token = Files.readString(new File(TOKEN_FILE).toPath()).trim();
        } catch (IOException e) {
            token = "";
        }
        if (token.isEmpty()) {
            Alerts.OFF.setText("DS upload receiver off: no " + TOKEN_FILE);
            Alerts.OFF.set(true);
            return;
        }
        File dir = new File(logFolder, "ds");
        if (!dir.isDirectory() && !dir.mkdirs()) {
            Alerts.OFF.setText("DS upload receiver off: cannot create " + dir);
            Alerts.OFF.set(true);
            return;
        }
        DsUploadReceiver r = new DsUploadReceiver(dir, token, LogStorage.FREE_FLOOR_BYTES);
        ServerSocket server;
        try {
            server = new ServerSocket();
            server.setReuseAddress(true);
            server.bind(new InetSocketAddress(PORT));
        } catch (IOException e) {
            Alerts.OFF.setText("DS upload receiver off: port " + PORT + ": " + e.getMessage());
            Alerts.OFF.set(true);
            return;
        }
        instance = r;
        Thread t = new Thread(() -> r.serve(server), "DsUploadReceiver");
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY);
        t.start();
    }

    /**
     * Passes the robot's enabled state to the receiver and logs its counters. Call every loop.
     *
     * @param enabled whether the robot is enabled this loop
     */
    public static void periodic(boolean enabled) {
        DsUploadReceiver r = instance;
        if (r == null) {
            return;
        }
        r.robotEnabled = enabled;
        if (Telemetry.slowLogThisLoop()) {
            Telemetry.log("DSUpload/FilesReceived", r.filesCompleted.get());
            Telemetry.log("DSUpload/BytesReceived", r.bytesReceived.get());
            Telemetry.log("DSUpload/LastFile", r.lastFile);
            Telemetry.log("DSUpload/LastError", r.lastError);
        }
    }

    void setRobotEnabled(boolean enabled) {
        robotEnabled = enabled;
    }

    long filesCompleted() {
        return filesCompleted.get();
    }

    // ── socket ──────────────────────────────────────────────────────────────────────────────────

    private void serve(ServerSocket server) {
        try (server) {
            while (!Thread.currentThread().isInterrupted()) {
                try (Socket s = server.accept()) {
                    s.setSoTimeout(SOCKET_TIMEOUT_MS);
                    handleConnection(s.getInputStream(), s.getOutputStream());
                } catch (IOException e) {
                    lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
                }
            }
        } catch (IOException e) {
            lastError = "listen failed: " + e.getMessage();
        }
    }

    /** One request per connection: read the head, hand the body stream to {@link #handle}. */
    void handleConnection(InputStream rawIn, OutputStream out) throws IOException {
        handleConnection(rawIn, out, CONNECTION_LIMIT_MS);
    }

    void handleConnection(InputStream rawIn, OutputStream out, long limitMs) throws IOException {
        LimitedInput in =
                new LimitedInput(
                        new BufferedInputStream(rawIn), System.nanoTime() + limitMs * 1_000_000L);
        Reply reply;
        boolean isHead = false;
        try {
            String head = readHead(in);
            if (head == null) {
                reply = Reply.of(400, "{\"error\":\"bad request\"}");
            } else {
                String[] lines = head.split("\r\n");
                String[] start = lines[0].split(" ");
                Map<String, String> headers = new HashMap<>();
                for (int i = 1; i < lines.length; i++) {
                    int c = lines[i].indexOf(':');
                    if (c > 0) {
                        headers.put(
                                lines[i].substring(0, c).trim().toLowerCase(Locale.ROOT),
                                lines[i].substring(c + 1).trim());
                    }
                }
                if (start.length < 2) {
                    reply = Reply.of(400, "{\"error\":\"bad request line\"}");
                } else {
                    isHead = start[0].equals("HEAD");
                    in.bodyStarts();
                    reply = handle(start[0], start[1], headers, in);
                    long unread = parseLong(headers.get("content-length")) - in.bodyRead();
                    if (unread > 0 && unread <= DRAIN_LIMIT_BYTES) {
                        drain(in, unread);
                    }
                }
            }
        } catch (SocketTimeoutException e) {
            lastError = "timed out mid-upload";
            return; // the client resumes from X-Received
        }
        StringBuilder sb = new StringBuilder();
        sb.append("HTTP/1.1 ").append(reply.status()).append(' ').append(reason(reply.status()));
        sb.append("\r\nContent-Type: application/json\r\nConnection: close\r\n");
        for (Map.Entry<String, String> h : reply.headers().entrySet()) {
            sb.append(h.getKey()).append(": ").append(h.getValue()).append("\r\n");
        }
        byte[] body = reply.body().getBytes(StandardCharsets.UTF_8);
        sb.append("Content-Length: ").append(body.length).append("\r\n\r\n");
        out.write(sb.toString().getBytes(StandardCharsets.UTF_8));
        if (!isHead) { // a HEAD reply is headers only
            out.write(body);
        }
        out.flush();
    }

    private static String readHead(InputStream in) throws IOException {
        ByteArrayOutputStream buf = new ByteArrayOutputStream();
        int matched = 0;
        while (buf.size() < MAX_HEADER_BYTES) {
            int b = in.read();
            if (b < 0) {
                return null;
            }
            buf.write(b);
            matched = (b == "\r\n\r\n".charAt(matched)) ? matched + 1 : (b == '\r' ? 1 : 0);
            if (matched == 4) {
                String s = buf.toString(StandardCharsets.ISO_8859_1);
                return s.substring(0, s.length() - 4);
            }
        }
        return null;
    }

    private static String reason(int status) {
        return switch (status) {
            case 200 -> "OK";
            case 400 -> "Bad Request";
            case 401 -> "Unauthorized";
            case 404 -> "Not Found";
            case 409 -> "Conflict";
            case 413 -> "Payload Too Large";
            case 507 -> "Insufficient Storage";
            default -> "Error";
        };
    }

    // ── request handling ───────────────────────────────────────────────────────────────────────

    /**
     * Handles one request. The body is read from {@code body}, up to {@code Content-Length}.
     *
     * @param method HTTP method
     * @param target request target, path plus query
     * @param headers request headers, names lower-case
     * @param body the request body stream
     * @return the reply to send
     * @throws IOException on a read or write failure
     */
    Reply handle(String method, String target, Map<String, String> headers, InputStream body)
            throws IOException {
        String given = headers.getOrDefault(TOKEN_HEADER, "");
        if (!MessageDigest.isEqual(token, given.getBytes(StandardCharsets.UTF_8))) {
            return Reply.of(401, "{\"error\":\"bad token\"}");
        }
        if (robotEnabled) {
            return Reply.of(409, "{\"error\":\"robot is enabled\"}");
        }
        int q = target.indexOf('?');
        String path = q < 0 ? target : target.substring(0, q);
        Map<String, String> query = parseQuery(q < 0 ? "" : target.substring(q + 1));
        String[] parts = path.split("/");
        Kind kind;
        String rawName;
        boolean whole = false;
        if (parts.length == 3 && parts[1].equals("dslog")) {
            kind = Kind.DSLOG;
            rawName = parts[2];
            whole = true;
        } else if (parts.length == 4 && parts[1].equals("upload")) {
            kind = Kind.parse(parts[2]);
            rawName = parts[3];
        } else {
            return Reply.of(404, "{\"error\":\"unknown path\"}");
        }
        String name = decode(rawName);
        if (kind == null || name == null || !safeName(name) || !kind.name.matcher(name).matches()) {
            return Reply.of(400, "{\"error\":\"bad kind or file name\"}");
        }
        String stored = kind.diskName(name);
        File done = new File(dir, stored);
        File part = new File(dir, stored + ".part");
        if (method.equals("HEAD")) {
            // A partial copy wins: it is a newer, larger version being sent after a finished one.
            if (part.isFile()) {
                return received(200, part.length(), false);
            }
            return received(200, done.isFile() ? done.length() : 0, done.isFile());
        }
        if (!method.equals("PUT")) {
            return Reply.of(400, "{\"error\":\"PUT or HEAD only\"}");
        }
        long length = parseLong(headers.get("content-length"));
        if (length < 0) {
            return Reply.of(400, "{\"error\":\"Content-Length required\"}");
        }
        long offset = whole ? 0 : parseLong(query.get("offset"));
        long total = whole ? length : parseLong(query.get("total"));
        if (offset < 0 || total < 0 || offset + length > total) {
            return Reply.of(400, "{\"error\":\"bad offset or total\"}");
        }
        if (total > kind.maxBytes) {
            return Reply.of(413, "{\"error\":\"over the " + kind.maxBytes + " byte cap\"}");
        }
        if (done.isFile() && done.length() == total && !whole) {
            drain(body, length);
            return received(200, total, true); // already have it
        }
        long have = part.isFile() ? part.length() : 0;
        if (offset != have && offset != 0) {
            drain(body, length);
            return received(400, have, false);
        }
        trim(total - offset, part.getName());
        if (dir.getUsableSpace() - (total - offset) < freeFloorBytes) {
            drain(body, length);
            return Reply.of(507, "{\"error\":\"robot disk nearly full\"}");
        }
        writtenThisBoot.remove(part.getName());
        writtenThisBoot.add(part.getName());
        try (RandomAccessFile f = new RandomAccessFile(part, "rw")) {
            f.setLength(offset); // offset 0 restarts a push
            f.seek(offset);
            byte[] buf = new byte[64 * 1024];
            long left = length;
            while (left > 0) {
                int n = body.read(buf, 0, (int) Math.min(buf.length, left));
                if (n < 0) {
                    break; // client gone: keep what arrived, it resumes from X-Received
                }
                f.write(buf, 0, n);
                left -= n;
                bytesReceived.addAndGet(n);
            }
        }
        long now = part.length();
        if (now == total) {
            try {
                Files.move(
                        part.toPath(),
                        done.toPath(),
                        StandardCopyOption.REPLACE_EXISTING,
                        StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException e) {
                Files.move(part.toPath(), done.toPath(), StandardCopyOption.REPLACE_EXISTING);
            }
            writtenThisBoot.remove(part.getName());
            writtenThisBoot.add(done.getName());
            filesCompleted.incrementAndGet();
            lastFile = stored;
            return received(200, total, true);
        }
        return received(200, now, false);
    }

    private static Reply received(int status, long bytes, boolean complete) {
        return new Reply(
                status,
                Map.of(
                        "X-Received",
                        Long.toString(bytes),
                        "X-Complete",
                        Boolean.toString(complete)),
                "{\"received\":" + bytes + ",\"complete\":" + complete + "}");
    }

    /**
     * Deletes files until {@code incoming} more bytes fit under the cap: first those from earlier
     * boots (oldest first, partial ones included), then this boot's in the order they were written.
     * The file being written ({@code keep}) is never deleted.
     */
    void trim(long incoming, String keep) {
        File[] files = dir.listFiles(File::isFile);
        if (files == null) {
            return;
        }
        List<String> order = new ArrayList<>(writtenThisBoot);
        List<File> sorted = new ArrayList<>(List.of(files));
        sorted.sort(
                Comparator.comparingInt((File f) -> order.indexOf(f.getName()))
                        .thenComparingLong(File::lastModified));
        long total = sorted.stream().mapToLong(File::length).sum();
        for (File f : sorted) {
            if (total + incoming <= capBytes) {
                break;
            }
            if (f.getName().equals(keep)) {
                continue;
            }
            long size = f.length();
            if (f.delete()) {
                total -= size;
                writtenThisBoot.remove(f.getName());
            }
        }
    }

    static boolean safeName(String name) {
        return !name.contains("/")
                && !name.contains("\\")
                && !name.contains("..")
                && name.chars().noneMatch(c -> c < 0x20);
    }

    private static String decode(String s) {
        try {
            return URLDecoder.decode(s.replace("+", "%2B"), StandardCharsets.UTF_8);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    private static Map<String, String> parseQuery(String q) {
        Map<String, String> out = new HashMap<>();
        for (String kv : q.split("&")) {
            int e = kv.indexOf('=');
            if (e > 0) {
                out.put(kv.substring(0, e), kv.substring(e + 1));
            }
        }
        return out;
    }

    private static long parseLong(String s) {
        if (s == null) {
            return -1;
        }
        try {
            return Long.parseLong(s.trim());
        } catch (NumberFormatException e) {
            return -1;
        }
    }

    private static void drain(InputStream in, long n) throws IOException {
        byte[] buf = new byte[8192];
        while (n > 0) {
            int r = in.read(buf, 0, (int) Math.min(buf.length, n));
            if (r < 0) {
                return;
            }
            n -= r;
        }
    }

    /** Counts the body bytes read, and fails a read once the connection's time is up. */
    static final class LimitedInput extends FilterInputStream {
        private final long deadlineNanos;
        private long read;
        private long bodyStart;

        LimitedInput(InputStream in, long deadlineNanos) {
            super(in);
            this.deadlineNanos = deadlineNanos;
        }

        void bodyStarts() {
            bodyStart = read;
        }

        long bodyRead() {
            return read - bodyStart;
        }

        private void check() throws SocketTimeoutException {
            if (System.nanoTime() > deadlineNanos) {
                throw new SocketTimeoutException("connection over its time limit");
            }
        }

        @Override
        public int read() throws IOException {
            check();
            int b = super.read();
            if (b >= 0) {
                read++;
            }
            return b;
        }

        @Override
        public int read(byte[] b, int off, int len) throws IOException {
            check();
            int n = super.read(b, off, len);
            if (n > 0) {
                read += n;
            }
            return n;
        }
    }
}
