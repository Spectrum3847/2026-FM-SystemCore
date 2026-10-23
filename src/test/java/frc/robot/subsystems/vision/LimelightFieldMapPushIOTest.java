package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.sun.net.httpserver.HttpServer;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class LimelightFieldMapPushIOTest {
    private static final String HASH = "2792cd4e868967b8";
    private static final URI URI_ = URI.create("http://limelight-back.local:5807/upload-fieldmap");

    /** Runs submitted pushes only when told, like a slow background thread. */
    private static final class ManualExecutor implements java.util.concurrent.Executor {
        final List<Runnable> queued = new ArrayList<>();

        @Override
        public void execute(Runnable r) {
            queued.add(r);
        }

        void runAll() {
            List<Runnable> now = new ArrayList<>(queued);
            queued.clear();
            now.forEach(Runnable::run);
        }
    }

    @Test
    void theUrlIsLimelightsUploadEndpoint() {
        assertEquals(URI_, LimelightFieldMapPushIO.uploadUri("limelight-back"));
    }

    @Test
    void offNeverSends() {
        ManualExecutor ex = new ManualExecutor();
        var io = new LimelightFieldMapPushIO(URI_, "{}", HASH, false, ex, (u, b) -> 200);
        FieldMapPushInputs in = new FieldMapPushInputs();
        io.updateInputs(in, true);
        assertTrue(ex.queued.isEmpty());
        assertFalse(in.enabled);
        assertEquals(0, in.attempts);
        assertEquals("", in.attemptedHash);
    }

    @Test
    void aPushRunsOffTheCallerAndIsConfirmedOn2xx() {
        ManualExecutor ex = new ManualExecutor();
        var io = new LimelightFieldMapPushIO(URI_, "{}", HASH, true, ex, (u, b) -> 200);
        FieldMapPushInputs in = new FieldMapPushInputs();
        io.updateInputs(in, true);
        assertEquals(1, ex.queued.size(), "queued, not run on the caller");
        assertTrue(in.inFlight);
        assertEquals(HASH, in.attemptedHash);
        assertEquals("", in.confirmedHash);
        io.updateInputs(in, true);
        assertEquals(1, ex.queued.size(), "one push at a time");

        ex.runAll();
        io.updateInputs(in, false);
        assertFalse(in.inFlight);
        assertEquals(200, in.httpStatus);
        assertEquals(HASH, in.confirmedHash);
        assertEquals("", in.error);
    }

    @Test
    void aFailureIsReportedAndRetriedOnlyAfterTheBackoff() {
        ManualExecutor ex = new ManualExecutor();
        var io =
                new LimelightFieldMapPushIO(
                        URI_,
                        "{}",
                        HASH,
                        true,
                        ex,
                        (u, b) -> {
                            throw new java.net.ConnectException("refused");
                        });
        FieldMapPushInputs in = new FieldMapPushInputs();
        io.updateInputs(in, true);
        ex.runAll();
        io.updateInputs(in, true);
        assertEquals(0, in.httpStatus);
        assertEquals("ConnectException: refused", in.error);
        assertEquals("", in.confirmedHash);
        assertTrue(ex.queued.isEmpty(), "no retry inside the backoff");
        assertEquals(1, in.attempts);
    }

    @Test
    void aRejectionIsNotConfirmed() {
        ManualExecutor ex = new ManualExecutor();
        var io = new LimelightFieldMapPushIO(URI_, "{}", HASH, true, ex, (u, b) -> 500);
        FieldMapPushInputs in = new FieldMapPushInputs();
        io.updateInputs(in, true);
        ex.runAll();
        io.updateInputs(in, false);
        assertEquals(500, in.httpStatus);
        assertEquals("HTTP 500", in.error);
        assertEquals("", in.confirmedHash);
    }

    /** The real HTTP client against a local server standing in for a Limelight. */
    @Test
    void postsTheFmapAsJson() throws Exception {
        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        AtomicReference<String> body = new AtomicReference<>();
        AtomicReference<String> method = new AtomicReference<>();
        AtomicReference<String> type = new AtomicReference<>();
        server.createContext(
                "/upload-fieldmap",
                ex -> {
                    method.set(ex.getRequestMethod());
                    type.set(ex.getRequestHeaders().getFirst("Content-Type"));
                    body.set(
                            new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8));
                    ex.sendResponseHeaders(200, -1);
                    ex.close();
                });
        server.start();
        try {
            URI uri =
                    URI.create(
                            "http://127.0.0.1:"
                                    + server.getAddress().getPort()
                                    + "/upload-fieldmap");
            String fmap = "{\"type\":\"frc\",\"fiducials\":[]}";
            int status =
                    LimelightFieldMapPushIO.httpPost(uri, fmap.getBytes(StandardCharsets.UTF_8));
            assertEquals(200, status);
            assertEquals("POST", method.get());
            assertEquals("application/json", type.get());
            assertEquals(fmap, body.get());
        } finally {
            server.stop(0);
        }
    }
}
