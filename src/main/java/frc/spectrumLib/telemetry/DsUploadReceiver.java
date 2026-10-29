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
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.ReentrantLock;
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
 * <p>Extend ({@code dslog} and {@code dswpilog} only). The DS keeps appending to its session log,
 * so after each match the file the robot finished last time is a prefix of the one to send. HEAD on
 * a finished file of those kinds adds {@code X-Sha256-Prefix-Supported: 1}. The client may then
 * send {@code PUT /upload/<kind>/<name>?offset=<n>&total=<N>&extend=1} with {@code X-Prefix-Sha256:
 * <hex SHA-256 of its first n bytes>}, where n is the finished file's length and N &gt; n. If the
 * robot's file hashes the same, it becomes the partial copy and the chunk is appended; later chunks
 * are plain offset PUTs. Otherwise the reply is 409 with {@code X-Extend: mismatch}, and the client
 * sends the whole file from offset 0.
 *
 * <p>The field gives each team about 7 Mbps, so a push only happens after the match, while the
 * robot is disabled. Uploads are refused while it is enabled, and one under way when it enables
 * stops at the next 64 KiB, keeps what arrived and closes, so the client resumes later. A small
 * pool of threads serves connections, so one idle or slow client cannot hold up the rest. The
 * threads only write files and never touch robot state, so replay is unaffected. Counters reach the
 * log through {@link #periodic(boolean)} on the main thread.
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
    // The client sends its request head in one write. Anything slower is not the client, and must
    // not hold a pool thread for the full socket timeout.
    static final int HEAD_TIMEOUT_MS = 3_000;

    /** {@link #HEAD_TIMEOUT_MS}; tests shorten it so they don't wait out 3 s. */
    int headTimeoutMs = HEAD_TIMEOUT_MS;

    // Each read restarts the socket timeout, so a client trickling bytes would hold its thread
    // forever without a total limit. A 20 MB whole NI log at 7 Mbps takes ~25 s.
    static final long CONNECTION_LIMIT_MS = 120_000;
    // A refused PUT's body is read and dropped up to this, so the client sees the 409 instead of
    // a reset connection. The helper sends 1 MiB chunks. Without the token, only a little is.
    private static final long DRAIN_LIMIT_BYTES = 32L << 20;
    private static final long UNAUTHORIZED_DRAIN_LIMIT_BYTES = 64L << 10;
    static final int POOL_THREADS = 3;
    static final String PREFIX_SUPPORTED_HEADER = "X-Sha256-Prefix-Supported";
    static final String PREFIX_HASH_HEADER = "x-prefix-sha256";
    static final String EXTEND_HEADER = "X-Extend";
    // Connections waiting for a thread; past this they are closed at once.
    private static final int POOL_QUEUE = 8;
    private static final String NAME_CHARS = "[A-Za-z0-9 _.()-]{1,120}";

    /** What may be pushed, by name and size. */
    enum Kind {
        DSLOG(
                "^\\d{4}_\\d{2}_\\d{2} \\d{2}_\\d{2}_\\d{2} [A-Za-z]{3}\\.(dslog|dsevents)$",
                20_000_000L),
        // One file per DS session, extended after every match: a full event day runs past 50 MB.
        DSWPILOG("^" + NAME_CHARS + "\\.wpilog$", 400_000_000L),
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

        /** The DS session logs, which grow between pushes and so may be extended. */
        boolean extendable() {
            return this == DSLOG || this == DSWPILOG;
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
    // this boot's files (not pulled yet) from older ones. Guarded by this, as is active: the files
    // being written, which trim never deletes.
    private final Set<String> writtenThisBoot = new LinkedHashSet<>();
    private final Set<String> active = new LinkedHashSet<>();
    // One per stored name, so two connections never write the same file at once.
    private final Map<String, ReentrantLock> fileLocks = new ConcurrentHashMap<>();

    /** A SHA-256 state over a file's first {@code length} bytes, as of mtime {@code modified}. */
    private record Hashed(long length, long modified, MessageDigest state) {
        String hex() {
            return HexFormat.of().formatHex(copy(state).digest());
        }
    }

    // Keyed by stored name, used only under that name's lock, for extendable kinds. finished: a
    // finished file's hash, good while its length and mtime match, so a repeated extend doesn't
    // rehash. running: a partial copy's hash, updated as chunks arrive, so the file it becomes
    // needs
    // no hashing at its next extend.
    private final Map<String, Hashed> finished = new ConcurrentHashMap<>();
    private final Map<String, Hashed> running = new ConcurrentHashMap<>();
    private final AtomicLong filesHashed = new AtomicLong();

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
        r.startServing(server);
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
     * thread goes real-time: the accept thread and the connection pool are created here, at minimum
     * priority, and never later.
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
        r.startServing(server);
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
            // Driver Station logs and match media uploaded to the robot since boot.
            Telemetry.log("DSUpload/FilesReceived", r.filesCompleted.get());
            // Bytes uploaded since boot.
            Telemetry.log("DSUpload/BytesReceived", r.bytesReceived.get());
            // The last file uploaded.
            Telemetry.log("DSUpload/LastFile", r.lastFile);
            // The last upload error, or empty.
            Telemetry.log("DSUpload/LastError", r.lastError);
        }
    }

    void setRobotEnabled(boolean enabled) {
        robotEnabled = enabled;
    }

    long filesCompleted() {
        return filesCompleted.get();
    }

    long filesHashed() {
        return filesHashed.get();
    }

    // ── socket ──────────────────────────────────────────────────────────────────────────────────

    /** Starts the accept thread and the connection pool, all daemon threads at minimum priority. */
    void startServing(ServerSocket server) {
        ThreadPoolExecutor pool = newPool();
        Thread t = new Thread(() -> serve(server, pool), "DsUploadReceiver");
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY);
        t.start();
    }

    private static ThreadPoolExecutor newPool() {
        AtomicInteger n = new AtomicInteger();
        ThreadPoolExecutor pool =
                new ThreadPoolExecutor(
                        POOL_THREADS,
                        POOL_THREADS,
                        0,
                        TimeUnit.MILLISECONDS,
                        new ArrayBlockingQueue<>(POOL_QUEUE),
                        r -> {
                            Thread t = new Thread(r, "DsUploadReceiver-" + n.incrementAndGet());
                            t.setDaemon(true);
                            t.setPriority(Thread.MIN_PRIORITY);
                            return t;
                        });
        pool.prestartAllCoreThreads();
        return pool;
    }

    private void serve(ServerSocket server, ThreadPoolExecutor pool) {
        try (server) {
            while (!Thread.currentThread().isInterrupted() && !server.isClosed()) {
                Socket s;
                try {
                    s = server.accept();
                } catch (IOException e) {
                    lastError = "accept: " + e.getMessage();
                    continue;
                }
                try {
                    pool.execute(() -> serveOne(s));
                } catch (RejectedExecutionException e) {
                    lastError = "too many connections";
                    closeQuietly(s);
                }
            }
        } catch (IOException e) {
            lastError = "listen failed: " + e.getMessage();
        }
    }

    private void serveOne(Socket socket) {
        try (Socket s = socket) {
            s.setSoTimeout(headTimeoutMs);
            handleConnection(
                    s.getInputStream(),
                    s.getOutputStream(),
                    CONNECTION_LIMIT_MS,
                    () -> s.setSoTimeout(SOCKET_TIMEOUT_MS));
        } catch (IOException e) {
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private static void closeQuietly(Socket s) {
        try {
            s.close();
        } catch (IOException e) {
            // already gone
        }
    }

    /** Runs once the request head has arrived, to lift the socket timeout for the body. */
    interface HeadRead {
        void run() throws IOException;
    }

    /** One request per connection: read the head, hand the body stream to {@link #handle}. */
    void handleConnection(InputStream rawIn, OutputStream out) throws IOException {
        handleConnection(rawIn, out, CONNECTION_LIMIT_MS);
    }

    void handleConnection(InputStream rawIn, OutputStream out, long limitMs) throws IOException {
        handleConnection(rawIn, out, limitMs, () -> {});
    }

    void handleConnection(InputStream rawIn, OutputStream out, long limitMs, HeadRead headRead)
            throws IOException {
        long now = System.nanoTime();
        LimitedInput in =
                new LimitedInput(
                        new BufferedInputStream(rawIn),
                        now + limitMs * 1_000_000L,
                        now + headTimeoutMs * 1_000_000L);
        Reply reply;
        boolean isHead = false;
        boolean headDone = false;
        try {
            String head = readHead(in);
            headDone = true;
            headRead.run();
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
                    long limit =
                            reply.status() == 401
                                    ? UNAUTHORIZED_DRAIN_LIMIT_BYTES
                                    : DRAIN_LIMIT_BYTES;
                    if (unread > 0 && unread <= limit) {
                        drain(in, unread);
                    }
                }
            }
        } catch (SocketTimeoutException e) {
            lastError = headDone ? "timed out mid-upload" : "no request in time";
            return; // the client resumes from X-Received
        } catch (EnabledMidUpload e) {
            lastError = "robot enabled mid-upload";
            return; // no reply and no drain frees the field link; it resumes from X-Received
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
        if (method.equals("PUT")) {
            return lockedPut(kind, done, part, stored, whole, headers, query, body);
        }
        if (method.equals("HEAD")) {
            // A partial copy wins: it is a newer, larger version being sent after a finished one.
            if (part.isFile()) {
                return received(200, part.length(), false);
            }
            if (done.isFile() && kind.extendable() && !whole) {
                Reply r = received(200, done.length(), true);
                Map<String, String> h = new HashMap<>(r.headers());
                h.put(PREFIX_SUPPORTED_HEADER, "1");
                return new Reply(200, h, r.body());
            }
            return received(200, done.isFile() ? done.length() : 0, done.isFile());
        }
        return Reply.of(400, "{\"error\":\"PUT or HEAD only\"}");
    }

    /**
     * Runs {@link #put} holding the lock for {@code stored}, waiting for another connection's write
     * of the same file to finish, up to this connection's time limit.
     */
    private Reply lockedPut(
            Kind kind,
            File done,
            File part,
            String stored,
            boolean whole,
            Map<String, String> headers,
            Map<String, String> query,
            InputStream body)
            throws IOException {
        ReentrantLock lock = fileLocks.computeIfAbsent(stored, k -> new ReentrantLock());
        long waitNanos =
                body instanceof LimitedInput li
                        ? li.remainingNanos()
                        : CONNECTION_LIMIT_MS * 1_000_000L;
        boolean locked;
        try {
            locked = lock.tryLock(Math.max(0, waitNanos), TimeUnit.NANOSECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new SocketTimeoutException("interrupted waiting for " + stored);
        }
        if (!locked) {
            throw new SocketTimeoutException("another upload of " + stored + " held it");
        }
        synchronized (this) {
            active.add(done.getName());
            active.add(part.getName());
        }
        try {
            return put(kind, done, part, stored, whole, headers, query, body);
        } finally {
            synchronized (this) {
                active.remove(done.getName());
                active.remove(part.getName());
            }
            lock.unlock();
        }
    }

    /** Writes one PUT's chunk. The caller holds the lock for {@code stored}. */
    private Reply put(
            Kind kind,
            File done,
            File part,
            String stored,
            boolean whole,
            Map<String, String> headers,
            Map<String, String> query,
            InputStream body)
            throws IOException {
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
        Hashed extendFrom = null;
        if ("1".equals(query.get("extend"))) {
            if (whole || !kind.extendable()) {
                return Reply.of(400, "{\"error\":\"extend is for dslog and dswpilog only\"}");
            }
            String want = headers.getOrDefault(PREFIX_HASH_HEADER, "").toLowerCase(Locale.ROOT);
            if (!want.matches("[0-9a-f]{64}") || total <= offset) {
                return Reply.of(400, "{\"error\":\"bad extend\"}");
            }
            // Only all of a finished file can be extended; anything else is sent again whole.
            Hashed prefix =
                    done.isFile() && !part.isFile() && done.length() == offset
                            ? prefixHash(stored, done)
                            : null;
            if (prefix == null || !prefix.hex().equals(want)) {
                drain(body, length);
                return new Reply(
                        409, Map.of(EXTEND_HEADER, "mismatch"), "{\"error\":\"prefix mismatch\"}");
            }
            extendFrom = prefix;
        }
        if (extendFrom == null && done.isFile() && done.length() == total && !whole) {
            drain(body, length);
            return received(200, total, true); // already have it
        }
        long have = extendFrom != null ? offset : part.isFile() ? part.length() : 0;
        if (offset != have && offset != 0) {
            drain(body, length);
            return received(400, have, false);
        }
        trim(total - offset, part.getName());
        if (dir.getUsableSpace() - (total - offset) < freeFloorBytes) {
            drain(body, length);
            return Reply.of(507, "{\"error\":\"robot disk nearly full\"}");
        }
        Hashed resume = running.remove(stored);
        MessageDigest digest = null;
        if (extendFrom != null) {
            move(done, part);
            finished.remove(stored);
            noteWritten(done.getName(), part.getName());
            digest = copy(extendFrom.state());
        } else {
            noteWritten(part.getName(), part.getName());
            if (kind.extendable() && offset == 0) {
                digest = sha256();
            } else if (kind.extendable() && resume != null && resume.length() == offset) {
                digest = resume.state();
            }
        }
        long hashed = offset;
        try (RandomAccessFile f = new RandomAccessFile(part, "rw")) {
            f.setLength(offset); // offset 0 restarts a push
            f.seek(offset);
            byte[] buf = new byte[64 * 1024];
            long left = length;
            while (left > 0) {
                if (robotEnabled) {
                    throw new EnabledMidUpload(); // keep what arrived
                }
                int n = body.read(buf, 0, (int) Math.min(buf.length, left));
                if (n < 0) {
                    break; // client gone: keep what arrived, it resumes from X-Received
                }
                f.write(buf, 0, n);
                if (digest != null) {
                    digest.update(buf, 0, n);
                    hashed += n;
                }
                left -= n;
                bytesReceived.addAndGet(n);
            }
        } finally {
            if (digest != null) {
                running.put(stored, new Hashed(hashed, 0, digest));
            }
        }
        long now = part.length();
        if (now == total) {
            move(part, done);
            Hashed all = running.remove(stored);
            if (all != null && all.length() == total) {
                finished.put(stored, new Hashed(total, done.lastModified(), all.state()));
            } else {
                finished.remove(stored);
            }
            noteWritten(part.getName(), done.getName());
            filesCompleted.incrementAndGet();
            lastFile = stored;
            return received(200, total, true);
        }
        return received(200, now, false);
    }

    private static void move(File from, File to) throws IOException {
        try {
            Files.move(
                    from.toPath(),
                    to.toPath(),
                    StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE);
        } catch (AtomicMoveNotSupportedException e) {
            Files.move(from.toPath(), to.toPath(), StandardCopyOption.REPLACE_EXISTING);
        }
    }

    /**
     * The hash of all of a finished file, from the cache if the file is unchanged since. Reads it
     * on this (receiver) thread, and stops if the robot enables.
     */
    private Hashed prefixHash(String stored, File done) throws IOException {
        long modified = done.lastModified();
        Hashed known = finished.get(stored);
        if (known != null && known.length() == done.length() && known.modified() == modified) {
            return known;
        }
        MessageDigest digest = sha256();
        long length = 0;
        try (InputStream in = Files.newInputStream(done.toPath())) {
            byte[] buf = new byte[64 * 1024];
            for (int n; (n = in.read(buf)) > 0; ) {
                if (robotEnabled) {
                    throw new EnabledMidUpload();
                }
                digest.update(buf, 0, n);
                length += n;
            }
        }
        filesHashed.incrementAndGet();
        Hashed h = new Hashed(length, modified, digest);
        finished.put(stored, h);
        return h;
    }

    private static MessageDigest sha256() {
        try {
            return MessageDigest.getInstance("SHA-256");
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("every JVM has SHA-256", e);
        }
    }

    private static MessageDigest copy(MessageDigest d) {
        try {
            return (MessageDigest) d.clone();
        } catch (CloneNotSupportedException e) {
            throw new IllegalStateException("the JDK's SHA-256 can be cloned", e);
        }
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

    private synchronized void noteWritten(String was, String now) {
        writtenThisBoot.remove(was);
        writtenThisBoot.add(now);
    }

    /**
     * Deletes files until {@code incoming} more bytes fit under the cap: first those from earlier
     * boots (oldest first, partial ones included), then this boot's in the order they were written.
     * Files being written ({@code keep}, and any another connection holds) are never deleted.
     */
    synchronized void trim(long incoming, String keep) {
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
            if (f.getName().equals(keep) || active.contains(f.getName())) {
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

    /** Thrown from the copy loop when the robot enables: stop and close without a reply. */
    static final class EnabledMidUpload extends IOException {
        private static final long serialVersionUID = 1L;

        EnabledMidUpload() {
            super("robot enabled mid-upload");
        }
    }

    /**
     * Counts the body bytes read, and fails a read once the connection's time is up, or the head's
     * time before the body starts.
     */
    static final class LimitedInput extends FilterInputStream {
        private final long deadlineNanos;
        private final long headDeadlineNanos;
        private boolean inBody;
        private long read;
        private long bodyStart;

        LimitedInput(InputStream in, long deadlineNanos, long headDeadlineNanos) {
            super(in);
            this.deadlineNanos = deadlineNanos;
            this.headDeadlineNanos = headDeadlineNanos;
        }

        void bodyStarts() {
            bodyStart = read;
            inBody = true;
        }

        long bodyRead() {
            return read - bodyStart;
        }

        long remainingNanos() {
            return deadlineNanos - System.nanoTime();
        }

        private void check() throws SocketTimeoutException {
            long now = System.nanoTime();
            if (now > deadlineNanos) {
                throw new SocketTimeoutException("connection over its time limit");
            }
            if (!inBody && now > headDeadlineNanos) {
                throw new SocketTimeoutException("request head too slow");
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
