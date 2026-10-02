package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpRequest.BodyPublisher;
import java.net.http.HttpRequest.BodyPublishers;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The DS log receiver's HTTP contract, against a real server on a loopback port. */
class DSLogHttpServerTest {
    @TempDir Path logDir;

    private static final String TOKEN = "test-token";
    private static final String HEADER = DSLogReceiver.Constants.TOKEN_HEADER;
    private static final String NAME_URL = "2026_10_02%2014_03_27%20Fri.dslog";
    private static final String NAME = "2026_10_02 14_03_27 Fri.dslog";

    private final AtomicBoolean enabled = new AtomicBoolean(false);
    private final List<String> received = new CopyOnWriteArrayList<>();
    private final List<Integer> rejected = new CopyOnWriteArrayList<>();
    private final HttpClient client =
            HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(5)).build();
    private DSLogHttpServer server;

    @BeforeEach
    void start() throws IOException {
        server =
                new DSLogHttpServer(
                        new InetSocketAddress(InetAddress.getLoopbackAddress(), 0),
                        logDir::toFile,
                        "ds",
                        TOKEN,
                        HEADER,
                        1000,
                        10_000,
                        0,
                        enabled::get,
                        new DSLogHttpServer.Listener() {
                            @Override
                            public void received(String name, long size) {
                                received.add(name + ":" + size);
                            }

                            @Override
                            public void rejected(int status, String reason) {
                                rejected.add(status);
                            }
                        },
                        DSLogReceiver.lowPriorityDaemon("DSLogHttpServerTest"));
        server.start();
    }

    @AfterEach
    void stop() {
        server.stop();
    }

    private URI uri(String path) {
        return URI.create("http://127.0.0.1:" + server.port() + path);
    }

    private HttpResponse<String> put(String name, String token, BodyPublisher body)
            throws IOException, InterruptedException {
        HttpRequest.Builder b =
                HttpRequest.newBuilder(uri("/dslog/" + name))
                        .timeout(Duration.ofSeconds(10))
                        .PUT(body);
        if (token != null) {
            b.header(HEADER, token);
        }
        return client.send(b.build(), BodyHandlers.ofString());
    }

    private HttpResponse<String> get(String path) throws IOException, InterruptedException {
        return client.send(
                HttpRequest.newBuilder(uri(path)).timeout(Duration.ofSeconds(10)).GET().build(),
                BodyHandlers.ofString());
    }

    @Test
    void storesAFileAndListsIt() throws Exception {
        HttpResponse<String> r = put(NAME_URL, TOKEN, BodyPublishers.ofByteArray(new byte[42]));
        assertEquals(201, r.statusCode());
        assertEquals("{\"name\":\"" + NAME + "\",\"size\":42}", r.body());
        assertEquals(42, Files.size(logDir.resolve("ds").resolve(NAME)));
        assertEquals(List.of(NAME + ":42"), received);

        HttpResponse<String> list = get("/dslog/");
        assertEquals(200, list.statusCode());
        assertTrue(list.body().startsWith("[{\"name\":\"" + NAME + "\",\"size\":42,\"mtime\":"));
    }

    @Test
    void refusesBadNameTokenEnabledAndSize() throws Exception {
        byte[] ten = new byte[10];
        assertEquals(400, put("evil.txt", TOKEN, BodyPublishers.ofByteArray(ten)).statusCode());
        assertEquals(
                400, put("..%2F" + NAME_URL, TOKEN, BodyPublishers.ofByteArray(ten)).statusCode());
        assertEquals(401, put(NAME_URL, null, BodyPublishers.ofByteArray(ten)).statusCode());
        assertEquals(401, put(NAME_URL, "nope", BodyPublishers.ofByteArray(ten)).statusCode());
        enabled.set(true);
        assertEquals(409, put(NAME_URL, TOKEN, BodyPublishers.ofByteArray(ten)).statusCode());
        enabled.set(false);
        assertEquals(
                413, put(NAME_URL, TOKEN, BodyPublishers.ofByteArray(new byte[1001])).statusCode());
        // No Content-Length: an unknown-length body goes chunked.
        assertEquals(
                411,
                put(
                                NAME_URL,
                                TOKEN,
                                BodyPublishers.ofInputStream(() -> new ByteArrayInputStream(ten)))
                        .statusCode());
        assertFalse(Files.exists(logDir.resolve("ds").resolve(NAME)));
        assertEquals(List.of(400, 400, 401, 401, 409, 413, 411), rejected);
        assertTrue(received.isEmpty());
    }

    @Test
    void health() throws Exception {
        HttpResponse<String> r = get("/health");
        assertEquals(200, r.statusCode());
        assertTrue(r.body().startsWith("{\"ok\":true,\"enabled\":false,\"logDir\":"));
        assertEquals(405, put("", TOKEN, BodyPublishers.noBody()).statusCode());
    }
}
