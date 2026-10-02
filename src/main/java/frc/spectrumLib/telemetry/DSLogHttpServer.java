package frc.spectrumLib.telemetry;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.security.MessageDigest;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * The HTTP half of {@link DSLogReceiver}. Kept in its own class so that a JVM without the {@code
 * jdk.httpserver} module only fails to load this class (caught by the receiver's start thread),
 * never {@link DSLogReceiver} itself, which the main loop calls.
 *
 * <p>Everything here runs on the server's threads. It must not touch WPILib, HAL, NetworkTables or
 * {@link Telemetry}: it reads the robot's enabled state through the supplied {@link
 * BooleanSupplier} (a volatile the main loop writes) and reports through the {@link Listener},
 * which only queues.
 *
 * <p>Requests are handled one at a time on a single executor thread, so two uploads never touch the
 * folder at once.
 */
final class DSLogHttpServer {

    /** Where results go. Called on the server thread; implementations must only queue. */
    interface Listener {
        void received(String name, long size);

        void rejected(int status, String reason);
    }

    private static final String DSLOG = "/dslog";

    private final HttpServer server;
    private final ExecutorService executor;
    private final Supplier<File> logDir;
    private final String subfolder;
    private final byte[] token;
    private final String tokenHeader;
    private final long maxFileBytes;
    private final long maxFolderBytes;
    private final long minFreeBytes;
    private final BooleanSupplier robotEnabled;
    private final Listener listener;

    /**
     * Binds the port (does not start serving; call {@link #start()}).
     *
     * @param address where to listen; port 0 picks a free one (tests)
     * @param logDir the log folder, asked on every request (a USB stick can appear late)
     * @param subfolder the folder under {@code logDir} that holds the DS files
     * @param token the value {@code tokenHeader} must carry on a PUT
     * @param tokenHeader the token's request header
     * @param maxFileBytes largest Content-Length accepted
     * @param maxFolderBytes most the stored files may total
     * @param minFreeBytes free space an upload must leave on the disk
     * @param robotEnabled true refuses uploads, and aborts one in progress
     * @param listener receives every accepted file and rejected upload
     * @param threads makes the one request thread
     * @throws IOException if the port cannot be bound
     */
    DSLogHttpServer(
            InetSocketAddress address,
            Supplier<File> logDir,
            String subfolder,
            String token,
            String tokenHeader,
            long maxFileBytes,
            long maxFolderBytes,
            long minFreeBytes,
            BooleanSupplier robotEnabled,
            Listener listener,
            ThreadFactory threads)
            throws IOException {
        this.logDir = logDir;
        this.subfolder = subfolder;
        this.token = token.getBytes(StandardCharsets.UTF_8);
        this.tokenHeader = tokenHeader;
        this.maxFileBytes = maxFileBytes;
        this.maxFolderBytes = maxFolderBytes;
        this.minFreeBytes = minFreeBytes;
        this.robotEnabled = robotEnabled;
        this.listener = listener;
        server = HttpServer.create(address, 4);
        server.createContext(DSLOG, ex -> handle(ex, this::dslog));
        server.createContext("/health", ex -> handle(ex, this::health));
        executor = Executors.newSingleThreadExecutor(threads);
        server.setExecutor(executor);
    }

    void start() {
        server.start();
    }

    void stop() {
        server.stop(0);
        executor.shutdownNow();
    }

    int port() {
        return server.getAddress().getPort();
    }

    File dsDir() {
        return new File(logDir.get(), subfolder);
    }

    private interface Handler {
        void handle(HttpExchange ex) throws IOException;
    }

    /** Runs a handler with nothing allowed to escape: a bad request must never hurt the robot. */
    private void handle(HttpExchange ex, Handler h) {
        try {
            h.handle(ex);
        } catch (Throwable t) {
            listener.rejected(500, t.getClass().getSimpleName() + ": " + t.getMessage());
            try {
                send(ex, 500, error("internal error"));
            } catch (Throwable ignored) {
                // response already started, or the client is gone
            }
        } finally {
            try {
                ex.close();
            } catch (Throwable ignored) {
                // nothing left to do
            }
        }
    }

    private void health(HttpExchange ex) throws IOException {
        if (!"GET".equals(ex.getRequestMethod())) {
            send(ex, 405, error("use GET"));
            return;
        }
        send(
                ex,
                200,
                "{\"ok\":true,\"enabled\":"
                        + robotEnabled.getAsBoolean()
                        + ",\"logDir\":"
                        + DSLogFiles.jsonString(logDir.get().getPath())
                        + "}");
    }

    private void dslog(HttpExchange ex) throws IOException {
        String path = ex.getRequestURI().getRawPath();
        String rest = path == null ? "" : path.substring(Math.min(DSLOG.length(), path.length()));
        if (rest.isEmpty() || rest.equals("/")) {
            if (!"GET".equals(ex.getRequestMethod())) {
                send(ex, 405, error("use GET /dslog/ to list, PUT /dslog/<name> to upload"));
                return;
            }
            send(ex, 200, DSLogFiles.toJson(DSLogFiles.list(dsDir())));
            return;
        }
        if (!rest.startsWith("/")) {
            send(ex, 404, error("not found"));
            return;
        }
        if (!"PUT".equals(ex.getRequestMethod())) {
            send(ex, 405, error("use PUT"));
            return;
        }
        put(ex, rest.substring(1));
    }

    private void put(HttpExchange ex, String rawName) throws IOException {
        String name;
        try {
            name = URLDecoder.decode(rawName, StandardCharsets.UTF_8);
        } catch (IllegalArgumentException e) {
            reject(ex, 400, "bad URL encoding");
            return;
        }
        if (!DSLogFiles.isValidName(name)) {
            reject(ex, 400, "bad file name: " + name);
            return;
        }
        String given = ex.getRequestHeaders().getFirst(tokenHeader);
        if (given == null
                || !MessageDigest.isEqual(token, given.getBytes(StandardCharsets.UTF_8))) {
            reject(ex, 401, "missing or wrong " + tokenHeader);
            return;
        }
        if (robotEnabled.getAsBoolean()) {
            reject(ex, 409, "robot is enabled");
            return;
        }
        String lengthHeader = ex.getRequestHeaders().getFirst("Content-Length");
        if (lengthHeader == null) {
            reject(ex, 411, "Content-Length required");
            return;
        }
        long length;
        try {
            length = Long.parseLong(lengthHeader.trim());
        } catch (NumberFormatException e) {
            reject(ex, 400, "bad Content-Length");
            return;
        }
        if (length < 0) {
            reject(ex, 400, "bad Content-Length");
            return;
        }
        if (length > maxFileBytes) {
            reject(ex, 413, "over " + maxFileBytes + " bytes: " + length);
            return;
        }

        File dir = dsDir();
        Files.createDirectories(dir.toPath());
        // Make room first, so the folder stays under its cap once this file lands.
        DSLogFiles.trim(dir, maxFolderBytes - length, name);
        if (dir.getUsableSpace() - length < minFreeBytes) {
            reject(ex, 507, "not enough free space in " + dir.getPath());
            return;
        }

        long size;
        try {
            size = DSLogFiles.receive(ex.getRequestBody(), length, dir, name, robotEnabled);
        } catch (DSLogFiles.AbortedException e) {
            reject(ex, 409, "robot enabled during upload of " + name);
            return;
        } catch (IOException e) {
            reject(ex, 400, "upload of " + name + " failed: " + e.getMessage());
            return;
        }
        DSLogFiles.trim(dir, maxFolderBytes, name);
        listener.received(name, size);
        send(ex, 201, "{\"name\":" + DSLogFiles.jsonString(name) + ",\"size\":" + size + "}");
    }

    /** Reports a refusal and answers it; a client that has already gone is not a second error. */
    private void reject(HttpExchange ex, int status, String reason) {
        listener.rejected(status, reason);
        try {
            send(ex, status, error(reason));
        } catch (IOException ignored) {
            // the client hung up (often why the upload failed in the first place)
        }
    }

    private static String error(String message) {
        return "{\"error\":" + DSLogFiles.jsonString(message) + "}";
    }

    private static void send(HttpExchange ex, int status, String json) throws IOException {
        byte[] body = json.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", "application/json");
        ex.getResponseHeaders().set("Connection", "close");
        ex.sendResponseHeaders(status, body.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(body);
        }
    }
}
