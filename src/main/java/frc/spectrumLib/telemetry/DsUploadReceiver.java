package frc.spectrumLib.telemetry;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
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
import java.util.List;
import java.util.Locale;
import java.util.Map;
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
 */
public final class DsUploadReceiver {
    public static final int PORT = 5806;
    public static final String TOKEN_FILE = "/home/systemcore/ds-upload-token";
    public static final String TOKEN_HEADER = "x-matchreview-token";

    /** The folder's pushed files are trimmed oldest first above this. */
    static final long DIR_CAP_BYTES = 1_500_000_000L;

    private static final int MAX_HEADER_BYTES = 8192;
    private static final int SOCKET_TIMEOUT_MS = 15_000;
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
    private volatile boolean robotEnabled = true; // refuse until the main loop says otherwise
    private final AtomicLong filesCompleted = new AtomicLong();
    private final AtomicLong bytesReceived = new AtomicLong();
    private volatile String lastFile = "";
    private volatile String lastError = "";

    private static DsUploadReceiver instance;
    private static final Alert offAlert = new Alert("", Level.LOW);

    DsUploadReceiver(File dir, String token, long freeFloorBytes) {
        this.dir = dir;
        this.token = token.trim().getBytes(StandardCharsets.UTF_8);
        this.freeFloorBytes = freeFloorBytes;
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
            offAlert.setText("DS upload receiver off: no " + TOKEN_FILE);
            offAlert.set(true);
            return;
        }
        File dir = new File(logFolder, "ds");
        if (!dir.isDirectory() && !dir.mkdirs()) {
            offAlert.setText("DS upload receiver off: cannot create " + dir);
            offAlert.set(true);
            return;
        }
        instance = new DsUploadReceiver(dir, token, LogStorage.FREE_FLOOR_BYTES);
        Thread t = new Thread(instance::serve, "DsUploadReceiver");
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

    private void serve() {
        try (ServerSocket server = new ServerSocket()) {
            server.setReuseAddress(true);
            server.bind(new InetSocketAddress(PORT));
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
        InputStream in = new BufferedInputStream(rawIn);
        String head = readHead(in);
        Reply reply;
        boolean isHead = false;
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
                try {
                    reply = handle(start[0], start[1], headers, in);
                } catch (SocketTimeoutException e) {
                    lastError = "timed out mid-upload";
                    return; // the client resumes from X-Received
                }
            }
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
        File done = new File(dir, name);
        File part = new File(dir, name + ".part");
        if (method.equals("HEAD")) {
            if (done.isFile()) {
                return received(200, done.length(), true);
            }
            return received(200, part.isFile() ? part.length() : 0, false);
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
        trim(total - offset);
        if (dir.getUsableSpace() - (total - offset) < freeFloorBytes) {
            drain(body, length);
            return Reply.of(507, "{\"error\":\"robot disk nearly full\"}");
        }
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
            filesCompleted.incrementAndGet();
            lastFile = name;
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

    /** Deletes the oldest finished files until {@code incoming} more bytes fit under the cap. */
    private void trim(long incoming) {
        File[] files = dir.listFiles(f -> f.isFile() && !f.getName().endsWith(".part"));
        if (files == null) {
            return;
        }
        List<File> sorted = new ArrayList<>(List.of(files));
        sorted.sort(Comparator.comparingLong(File::lastModified));
        long total = sorted.stream().mapToLong(File::length).sum();
        for (File f : sorted) {
            if (total + incoming <= DIR_CAP_BYTES) {
                break;
            }
            long size = f.length();
            if (f.delete()) {
                total -= size;
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
}
