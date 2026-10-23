package frc.robot.subsystems.vision;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Pushes a Limelight field map with Limelight's {@code POST http://<camera>.local:5807/
 * upload-fieldmap}, on a background thread. The main thread only starts a push and copies the last
 * result into the logged inputs, so a camera that never answers costs it nothing.
 */
public class LimelightFieldMapPushIO implements FieldMapPushIO {

    /** Limelight's REST API port. */
    static final int API_PORT = 5807;

    /** Between failed pushes to one camera. */
    static final double RETRY_SECONDS = 5.0;

    private static final Duration CONNECT_TIMEOUT = Duration.ofSeconds(1);
    private static final Duration REQUEST_TIMEOUT = Duration.ofSeconds(3);

    /** Sends one request; returns the HTTP status. */
    @FunctionalInterface
    interface Sender {
        int post(URI uri, byte[] body) throws Exception;
    }

    /** What a finished push reported. */
    private record Result(int status, String error) {}

    private final URI uri;
    private final byte[] payload;
    private final String hash;
    private final boolean enabled;
    private final Executor executor;
    private final Sender sender;

    private final AtomicBoolean inFlight = new AtomicBoolean(false);
    private final AtomicReference<Result> last = new AtomicReference<>();
    private long attempts = 0;
    private long nextAttemptNanos = 0;
    private String confirmedHash = "";

    /** One thread for every camera's pushes: they are rare and small. */
    private static ExecutorService sharedExecutor;

    private static HttpClient client;

    /**
     * Creates the push to one Limelight, and when enabled starts the boot push.
     *
     * @param camera the Limelight's hostname, e.g. {@code limelight-back}
     * @param fmap the .fmap JSON to send
     * @param hash the canonical hash of the map in it
     * @param enabled whether to push at all; off, nothing is ever sent
     * @return the IO; when enabled, its first push has already started
     */
    public static LimelightFieldMapPushIO forLimelight(
            String camera, String fmap, String hash, boolean enabled) {
        LimelightFieldMapPushIO io =
                new LimelightFieldMapPushIO(
                        uploadUri(camera),
                        fmap,
                        hash,
                        enabled,
                        executor(),
                        LimelightFieldMapPushIO::httpPost);
        // At boot, during robot init: the worker thread is created now, before the main thread
        // goes real-time, so it runs at normal priority.
        io.updateInputs(new FieldMapPushInputs(), true);
        return io;
    }

    LimelightFieldMapPushIO(
            URI uri, String fmap, String hash, boolean enabled, Executor executor, Sender sender) {
        this.uri = uri;
        this.payload = fmap.getBytes(StandardCharsets.UTF_8);
        this.hash = hash;
        this.enabled = enabled;
        this.executor = executor;
        this.sender = sender;
    }

    /** Where a camera's field map goes. */
    static URI uploadUri(String camera) {
        return URI.create("http://" + camera + ".local:" + API_PORT + "/upload-fieldmap");
    }

    @Override
    public void updateInputs(FieldMapPushInputs inputs, boolean mayPush) {
        long now = System.nanoTime();
        if (enabled
                && mayPush
                && now - nextAttemptNanos >= 0
                && inFlight.compareAndSet(false, true)) {
            attempts++;
            nextAttemptNanos = now + (long) (RETRY_SECONDS * 1e9);
            executor.execute(this::push);
        }
        Result r = last.get();
        inputs.enabled = enabled;
        inputs.attempts = attempts;
        inputs.inFlight = inFlight.get();
        inputs.attemptedHash = attempts > 0 ? hash : "";
        inputs.httpStatus = r == null ? -1 : r.status();
        inputs.error = r == null ? "" : r.error();
        if (r != null && r.status() >= 200 && r.status() < 300) {
            confirmedHash = hash;
        }
        inputs.confirmedHash = confirmedHash;
    }

    private void push() {
        Result result;
        try {
            int status = sender.post(uri, payload);
            result = new Result(status, status >= 200 && status < 300 ? "" : "HTTP " + status);
        } catch (Exception e) {
            result = new Result(0, e.getClass().getSimpleName() + ": " + e.getMessage());
        }
        last.set(result);
        inFlight.set(false);
    }

    private static synchronized Executor executor() {
        if (sharedExecutor == null) {
            sharedExecutor =
                    Executors.newSingleThreadExecutor(
                            r -> {
                                Thread t = new Thread(r, "LimelightFieldMapPush");
                                t.setDaemon(true);
                                return t;
                            });
        }
        return sharedExecutor;
    }

    static int httpPost(URI uri, byte[] body) throws Exception {
        synchronized (LimelightFieldMapPushIO.class) {
            if (client == null) {
                client = HttpClient.newBuilder().connectTimeout(CONNECT_TIMEOUT).build();
            }
        }
        HttpRequest request =
                HttpRequest.newBuilder(uri)
                        .timeout(REQUEST_TIMEOUT)
                        .header("Content-Type", "application/json")
                        .POST(HttpRequest.BodyPublishers.ofByteArray(body))
                        .build();
        return client.send(request, HttpResponse.BodyHandlers.discarding()).statusCode();
    }
}
