package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.telemetry.DsUploadReceiver.Reply;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class DsUploadReceiverTest {
    private static final String TOKEN = "s3cret";
    private static final String DSLOG = "2026_10_16 20_11_57 Fri.dslog";

    @TempDir Path dir;
    private DsUploadReceiver rx;

    @BeforeEach
    void setUp() {
        rx = new DsUploadReceiver(dir.toFile(), TOKEN, 0);
        rx.setRobotEnabled(false);
    }

    private Reply put(String target, byte[] body) throws IOException {
        Map<String, String> h = new HashMap<>();
        h.put(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        h.put("content-length", Integer.toString(body.length));
        return rx.handle("PUT", target, h, new ByteArrayInputStream(body));
    }

    private Reply head(String target) throws IOException {
        Map<String, String> h = Map.of(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        return rx.handle("HEAD", target, h, new ByteArrayInputStream(new byte[0]));
    }

    private static byte[] bytes(int n, int seed) {
        byte[] b = new byte[n];
        for (int i = 0; i < n; i++) {
            b[i] = (byte) (i * 31 + seed);
        }
        return b;
    }

    @Test
    void chunksAppendAndTheFileAppearsWhenComplete() throws IOException {
        byte[] all = bytes(3000, 7);
        String base = "/upload/video/TXMAN1_Q34.mp4";
        assertEquals(200, head(base).status()); // nothing yet; 404 is only for older robot code
        assertEquals("0", head(base).headers().get("X-Received"));
        assertEquals(
                200, put(base + "?offset=0&total=3000", Arrays.copyOfRange(all, 0, 1000)).status());
        assertFalse(new File(dir.toFile(), "TXMAN1_Q34.mp4").exists());
        assertEquals("1000", head(base).headers().get("X-Received"));
        assertEquals("false", head(base).headers().get("X-Complete"));
        assertEquals(
                200,
                put(base + "?offset=1000&total=3000", Arrays.copyOfRange(all, 1000, 3000))
                        .status());
        File done = new File(dir.toFile(), "TXMAN1_Q34.mp4");
        assertArrayEquals(all, Files.readAllBytes(done.toPath()));
        assertFalse(new File(dir.toFile(), "TXMAN1_Q34.mp4.part").exists());
        assertEquals("true", head(base).headers().get("X-Complete"));
        assertEquals(1, rx.filesCompleted());
    }

    @Test
    void aWrongOffsetIsRefusedWithWhatTheRobotHas() throws IOException {
        String base = "/upload/audio/TXMAN1_Q34.m4a";
        put(base + "?offset=0&total=100", bytes(40, 1));
        Reply r = put(base + "?offset=60&total=100", bytes(40, 2));
        assertEquals(400, r.status());
        assertEquals("40", r.headers().get("X-Received"));
    }

    @Test
    void offsetZeroRestartsAPush() throws IOException {
        String base = "/upload/audio/a.m4a";
        put(base + "?offset=0&total=10", bytes(5, 1));
        byte[] fresh = bytes(10, 9);
        assertEquals(200, put(base + "?offset=0&total=10", fresh).status());
        assertArrayEquals(fresh, Files.readAllBytes(dir.resolve("a.m4a")));
    }

    @Test
    void aFinishedFileIsNotRewritten() throws IOException {
        String base = "/upload/audio/a.m4a?offset=0&total=10";
        put(base, bytes(10, 1));
        Reply again = put(base, bytes(10, 2));
        assertEquals(200, again.status());
        assertTrue(again.body().contains("\"complete\":true"));
        assertArrayEquals(bytes(10, 1), Files.readAllBytes(dir.resolve("a.m4a")));
    }

    @Test
    void refusedWhileEnabled() throws IOException {
        rx.setRobotEnabled(true);
        assertEquals(409, put("/upload/audio/a.m4a?offset=0&total=1", bytes(1, 0)).status());
    }

    @Test
    void refusesABadToken() throws IOException {
        Map<String, String> h =
                Map.of(DsUploadReceiver.TOKEN_HEADER, "nope", "content-length", "1");
        Reply r =
                rx.handle(
                        "PUT",
                        "/upload/audio/a.m4a?offset=0&total=1",
                        h,
                        new ByteArrayInputStream(new byte[1]));
        assertEquals(401, r.status());
    }

    @Test
    void refusesBadNamesAndPathTraversal() throws IOException {
        assertEquals(
                400, put("/upload/video/..%2F..%2Fetc.mp4?offset=0&total=1", bytes(1, 0)).status());
        assertEquals(400, put("/upload/video/run.sh?offset=0&total=1", bytes(1, 0)).status());
        assertEquals(400, put("/upload/movie/a.mp4?offset=0&total=1", bytes(1, 0)).status());
        assertEquals(
                400, put("/upload/dslog/notadslog.dslog?offset=0&total=1", bytes(1, 0)).status());
        assertFalse(DsUploadReceiver.safeName("a/b.mp4"));
        assertFalse(DsUploadReceiver.safeName("..mp4"));
    }

    @Test
    void refusesOverTheSizeCap() throws IOException {
        Reply r = put("/upload/audio/a.m4a?offset=0&total=999999999", bytes(1, 0));
        assertEquals(413, r.status());
    }

    @Test
    void takesADayLongDsSessionLog() throws IOException {
        String base = "/upload/dswpilog/FIRST_DS_day.wpilog";
        assertEquals(200, put(base + "?offset=0&total=300000000", bytes(10, 1)).status());
        assertEquals(413, put(base + "?offset=0&total=400000001", bytes(10, 1)).status());
    }

    @Test
    void refusesWhenTheDiskWouldFillUp() throws IOException {
        DsUploadReceiver full = new DsUploadReceiver(dir.toFile(), TOKEN, Long.MAX_VALUE);
        full.setRobotEnabled(false);
        Map<String, String> h = Map.of(DsUploadReceiver.TOKEN_HEADER, TOKEN, "content-length", "1");
        Reply r =
                full.handle(
                        "PUT",
                        "/upload/audio/a.m4a?offset=0&total=1",
                        h,
                        new ByteArrayInputStream(new byte[1]));
        assertEquals(507, r.status());
    }

    @Test
    void wholeNiDsLogRouteStillWorks() throws IOException {
        byte[] log = bytes(500, 3);
        assertEquals(200, put("/dslog/" + DSLOG.replace(" ", "%20"), log).status());
        assertArrayEquals(log, Files.readAllBytes(dir.resolve(DSLOG)));
        byte[] bigger = bytes(800, 4); // the full file, sent again at the pit tether
        assertEquals(200, put("/dslog/" + DSLOG.replace(" ", "%20"), bigger).status());
        assertArrayEquals(bigger, Files.readAllBytes(dir.resolve(DSLOG)));
    }

    @Test
    void aDsWpilogIsStoredUnderANameOldPitHelpersIgnore() throws IOException {
        String base = "/upload/dswpilog/FIRST_DS_1.wpilog";
        assertEquals(200, put(base + "?offset=0&total=10", bytes(10, 1)).status());
        assertTrue(Files.isRegularFile(dir.resolve("FIRST_DS_1.dswpilog")));
        assertFalse(Files.exists(dir.resolve("FIRST_DS_1.wpilog")));
        assertEquals("true", head(base).headers().get("X-Complete"));
    }

    @Test
    void headReportsANewerPartialCopyOverAFinishedOne() throws IOException {
        String base = "/upload/dslog/" + DSLOG.replace(" ", "%20");
        put(base + "?offset=0&total=10", bytes(10, 1));
        put(base + "?offset=0&total=30", bytes(20, 1)); // the grown log, cut off part way
        Reply h = head(base);
        assertEquals("20", h.headers().get("X-Received"));
        assertEquals("false", h.headers().get("X-Complete"));
        assertEquals(200, put(base + "?offset=20&total=30", bytes(10, 2)).status());
        assertEquals(30, Files.size(dir.resolve(DSLOG)));
    }

    @Test
    void aRefusedPutIsReadSoTheClientGetsTheReply() throws IOException {
        rx.setRobotEnabled(true);
        byte[] body = bytes(200_000, 3);
        String req =
                "PUT /upload/video/a.mp4?offset=0&total=200000 HTTP/1.1\r\n"
                        + "X-MatchReview-Token: "
                        + TOKEN
                        + "\r\nContent-Length: 200000\r\n\r\n";
        ByteArrayOutputStream raw = new ByteArrayOutputStream();
        raw.write(req.getBytes(StandardCharsets.ISO_8859_1));
        raw.write(body);
        ByteArrayInputStream in = new ByteArrayInputStream(raw.toByteArray());
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        rx.handleConnection(in, out);
        assertTrue(out.toString(StandardCharsets.ISO_8859_1).startsWith("HTTP/1.1 409"));
        assertEquals(0, in.available(), "the body was read before replying");
    }

    @Test
    void aConnectionOverItsTimeLimitIsDropped() throws IOException {
        String req = "HEAD /upload/audio/a.m4a HTTP/1.1\r\nX-MatchReview-Token: " + TOKEN;
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        rx.handleConnection(
                new ByteArrayInputStream(req.getBytes(StandardCharsets.ISO_8859_1)), out, -1);
        assertEquals(0, out.size(), "no reply: the slot is freed");
    }

    @Test
    void trimDeletesEarlierBootsFirstAndCountsPartialFiles() throws IOException {
        Files.write(dir.resolve("old_Q1.mp4"), bytes(40, 1));
        Files.write(dir.resolve("old_Q2.mp4.part"), bytes(40, 1));
        dir.resolve("old_Q1.mp4").toFile().setLastModified(System.currentTimeMillis());
        // this boot's file looks oldest, as when the robot boots with its clock behind
        DsUploadReceiver small = new DsUploadReceiver(dir.toFile(), TOKEN, 0, 90);
        small.setRobotEnabled(false);
        Map<String, String> h = new HashMap<>();
        h.put(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        h.put("content-length", "30");
        small.handle(
                "PUT",
                "/upload/video/new_Q3.mp4?offset=0&total=30",
                h,
                new ByteArrayInputStream(bytes(30, 2)));
        dir.resolve("new_Q3.mp4").toFile().setLastModified(0);
        small.handle(
                "PUT",
                "/upload/video/new_Q4.mp4?offset=0&total=30",
                h,
                new ByteArrayInputStream(bytes(30, 3)));
        assertFalse(Files.exists(dir.resolve("old_Q1.mp4")));
        assertFalse(Files.exists(dir.resolve("old_Q2.mp4.part")));
        assertTrue(Files.exists(dir.resolve("new_Q3.mp4")), "this boot's files go last");
        assertTrue(Files.exists(dir.resolve("new_Q4.mp4")));
    }

    @Test
    void aHeadReplyIsHeadersOnly() throws IOException {
        String req =
                "HEAD /upload/audio/a.m4a HTTP/1.1\r\nX-MatchReview-Token: " + TOKEN + "\r\n\r\n";
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        rx.handleConnection(
                new ByteArrayInputStream(req.getBytes(StandardCharsets.ISO_8859_1)), out);
        String reply = out.toString(StandardCharsets.ISO_8859_1);
        assertTrue(reply.startsWith("HTTP/1.1 200 OK"), reply);
        assertTrue(reply.contains("X-Received: 0"), reply);
        assertTrue(reply.endsWith("\r\n\r\n"), reply);
    }

    @Test
    void aWholeHttpRequestGetsAnHttpReply() throws IOException {
        byte[] body = bytes(64, 5);
        String req =
                "PUT /upload/dswpilog/FRC_DS_Q34.wpilog?offset=0&total=64 HTTP/1.1\r\n"
                        + "Host: 10.85.15.2:5809\r\n"
                        + "X-MatchReview-Token: "
                        + TOKEN
                        + "\r\n"
                        + "Content-Length: 64\r\n\r\n";
        ByteArrayOutputStream in = new ByteArrayOutputStream();
        in.write(req.getBytes(StandardCharsets.ISO_8859_1));
        in.write(body);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        rx.handleConnection(new ByteArrayInputStream(in.toByteArray()), out);
        String reply = out.toString(StandardCharsets.ISO_8859_1);
        assertTrue(reply.startsWith("HTTP/1.1 200 OK"), reply);
        assertTrue(reply.contains("X-Received: 64"), reply);
        assertArrayEquals(body, Files.readAllBytes(dir.resolve("FRC_DS_Q34.dswpilog")));
    }

    /** A body that runs {@code after} as the read that reaches byte {@code first} returns. */
    private static InputStream bodyThen(byte[] body, int first, Runnable after) {
        return new InputStream() {
            private int pos;
            private boolean ran;

            @Override
            public int read() {
                byte[] one = new byte[1];
                return read(one, 0, 1) < 0 ? -1 : one[0] & 0xff;
            }

            @Override
            public int read(byte[] b, int off, int len) {
                if (pos >= body.length) {
                    return -1;
                }
                int n = Math.min(len, (pos < first ? first : body.length) - pos);
                System.arraycopy(body, pos, b, off, n);
                pos += n;
                if (pos >= first && !ran) {
                    ran = true;
                    after.run();
                }
                return n;
            }
        };
    }

    @Test
    void enablingMidUploadStopsAndKeepsWhatArrived() throws IOException {
        byte[] body = bytes(300_000, 4);
        Map<String, String> h = new HashMap<>();
        h.put(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        h.put("content-length", "300000");
        InputStream in = bodyThen(body, 65536, () -> rx.setRobotEnabled(true));
        assertThrows(
                DsUploadReceiver.EnabledMidUpload.class,
                () -> rx.handle("PUT", "/upload/video/a.mp4?offset=0&total=300000", h, in));
        assertEquals(65536, Files.size(dir.resolve("a.mp4.part")), "stopped at the next chunk");
        assertFalse(Files.exists(dir.resolve("a.mp4")));
        rx.setRobotEnabled(false);
        assertEquals("65536", head("/upload/video/a.mp4").headers().get("X-Received"));
        Reply rest =
                put(
                        "/upload/video/a.mp4?offset=65536&total=300000",
                        Arrays.copyOfRange(body, 65536, 300_000));
        assertEquals("true", rest.headers().get("X-Complete"));
        assertArrayEquals(body, Files.readAllBytes(dir.resolve("a.mp4")));
    }

    @Test
    void enablingMidUploadClosesWithoutAReplyOrDrain() throws IOException {
        byte[] body = bytes(300_000, 4);
        String req =
                "PUT /upload/video/a.mp4?offset=0&total=300000 HTTP/1.1\r\n"
                        + "X-MatchReview-Token: "
                        + TOKEN
                        + "\r\nContent-Length: 300000\r\n\r\n";
        byte[] headBytes = req.getBytes(StandardCharsets.ISO_8859_1);
        byte[] all = new byte[headBytes.length + body.length];
        System.arraycopy(headBytes, 0, all, 0, headBytes.length);
        System.arraycopy(body, 0, all, headBytes.length, body.length);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        rx.handleConnection(
                bodyThen(all, headBytes.length + 100_000, () -> rx.setRobotEnabled(true)), out);
        assertEquals(0, out.size(), "closed without a reply");
        assertTrue(Files.size(dir.resolve("a.mp4.part")) < 300_000);
    }

    @Test
    void writesToOneFileAreSerialized() throws Exception {
        String base = "/upload/dslog/" + DSLOG.replace(" ", "%20");
        CountDownLatch halfway = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        Map<String, String> h = new HashMap<>();
        h.put(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        h.put("content-length", "10");
        InputStream slow =
                bodyThen(
                        bytes(10, 1),
                        5,
                        () -> {
                            halfway.countDown();
                            try {
                                release.await();
                            } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                            }
                        });
        AtomicReference<Reply> first = new AtomicReference<>();
        AtomicReference<Reply> second = new AtomicReference<>();
        Thread a =
                new Thread(
                        () -> {
                            try {
                                first.set(rx.handle("PUT", base + "?offset=0&total=20", h, slow));
                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            }
                        });
        a.start();
        assertTrue(halfway.await(5, TimeUnit.SECONDS));
        // Unserialized, this would see 5 bytes on disk and refuse offset 10.
        Thread b =
                new Thread(
                        () -> {
                            try {
                                second.set(
                                        rx.handle(
                                                "PUT",
                                                base + "?offset=10&total=20",
                                                h,
                                                new ByteArrayInputStream(bytes(10, 2))));
                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            }
                        });
        b.start();
        b.join(300);
        assertTrue(b.isAlive(), "the second write waits for the first");
        release.countDown();
        a.join(5000);
        b.join(5000);
        assertEquals("10", first.get().headers().get("X-Received"));
        assertEquals("true", second.get().headers().get("X-Complete"));
        byte[] want = new byte[20];
        System.arraycopy(bytes(10, 1), 0, want, 0, 10);
        System.arraycopy(bytes(10, 2), 0, want, 10, 10);
        assertArrayEquals(want, Files.readAllBytes(dir.resolve(DSLOG)));
    }

    private static String send(int port, byte[] request) throws IOException {
        try (Socket s = new Socket()) {
            s.connect(new InetSocketAddress("127.0.0.1", port), 2000);
            s.setSoTimeout(10_000);
            OutputStream o = s.getOutputStream();
            o.write(request);
            o.flush();
            return new String(s.getInputStream().readAllBytes(), StandardCharsets.ISO_8859_1);
        }
    }

    @Test
    void anIdleConnectionDoesNotBlockAnUploadAndIsDroppedSoon() throws Exception {
        rx.headTimeoutMs = 500; // the upload must be served before the idle one is dropped
        try (ServerSocket server = new ServerSocket()) {
            server.bind(new InetSocketAddress("127.0.0.1", 0));
            rx.startServing(server);
            int port = server.getLocalPort();
            try (Socket idle = new Socket()) {
                idle.connect(new InetSocketAddress("127.0.0.1", port), 2000);
                idle.setSoTimeout(rx.headTimeoutMs + 5000);
                long t0 = System.nanoTime();
                String req =
                        "PUT /upload/audio/a.m4a?offset=0&total=4 HTTP/1.1\r\n"
                                + "X-MatchReview-Token: "
                                + TOKEN
                                + "\r\nContent-Length: 4\r\n\r\nabcd";
                String reply = send(port, req.getBytes(StandardCharsets.ISO_8859_1));
                assertTrue(reply.startsWith("HTTP/1.1 200"), reply);
                assertTrue(
                        System.nanoTime() - t0 < rx.headTimeoutMs * 1_000_000L,
                        "served while the idle one is open");
                assertEquals(-1, idle.getInputStream().read(), "idle one closed, no reply");
                long held = (System.nanoTime() - t0) / 1_000_000;
                assertTrue(held < rx.headTimeoutMs + 2000, "held " + held + " ms");
            }
        }
    }

    @Test
    void aTricklingHeadIsDroppedAtTheHeadDeadline() throws IOException {
        // Bytes keep arriving, so only the head deadline (not the socket timeout) ends it.
        rx.headTimeoutMs = 200;
        InputStream trickle =
                new InputStream() {
                    @Override
                    public int read() {
                        try {
                            Thread.sleep(50);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                        return 'x';
                    }

                    @Override
                    public int read(byte[] b, int off, int len) {
                        b[off] = (byte) read();
                        return 1; // a socket returns what has arrived
                    }
                };
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        long t0 = System.nanoTime();
        rx.handleConnection(trickle, out);
        long ms = (System.nanoTime() - t0) / 1_000_000;
        assertEquals(0, out.size());
        assertTrue(ms >= rx.headTimeoutMs, "dropped before its deadline: " + ms + " ms");
        assertTrue(ms < rx.headTimeoutMs + 1000, "took " + ms + " ms");
    }

    // ── extend ──────────────────────────────────────────────────────────────────────────────────

    private static final String DSLOG_URL = "/upload/dslog/" + DSLOG.replace(" ", "%20");

    private static String sha(byte[] b, int n) {
        try {
            MessageDigest d = MessageDigest.getInstance("SHA-256");
            d.update(b, 0, n);
            return HexFormat.of().formatHex(d.digest());
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    private static byte[] copy(byte[] b, int from, int to) {
        return Arrays.copyOfRange(b, from, to);
    }

    private Reply extend(String base, int n, int total, byte[] chunk, String hash)
            throws IOException {
        Map<String, String> h = new HashMap<>();
        h.put(DsUploadReceiver.TOKEN_HEADER, TOKEN);
        h.put("content-length", Integer.toString(chunk.length));
        h.put(DsUploadReceiver.PREFIX_HASH_HEADER, hash);
        return rx.handle(
                "PUT",
                base + "?offset=" + n + "&total=" + total + "&extend=1",
                h,
                new ByteArrayInputStream(chunk));
    }

    @Test
    void aGrownSessionLogIsExtendedNotResent() throws IOException {
        byte[] log = bytes(3000, 6); // the session log after three matches
        assertEquals(200, put(DSLOG_URL + "?offset=0&total=1000", copy(log, 0, 1000)).status());
        Reply h = head(DSLOG_URL);
        assertEquals("1", h.headers().get(DsUploadReceiver.PREFIX_SUPPORTED_HEADER));
        assertEquals("1000", h.headers().get("X-Received"));
        assertEquals("true", h.headers().get("X-Complete"));

        Reply r = extend(DSLOG_URL, 1000, 2000, copy(log, 1000, 1300), sha(log, 1000));
        assertEquals(200, r.status());
        assertEquals("1300", r.headers().get("X-Received"));
        assertEquals("false", r.headers().get("X-Complete"));
        assertFalse(Files.exists(dir.resolve(DSLOG)), "the finished copy became the partial one");
        assertEquals("1300", head(DSLOG_URL).headers().get("X-Received"));
        assertNull(head(DSLOG_URL).headers().get(DsUploadReceiver.PREFIX_SUPPORTED_HEADER));
        Reply rest = put(DSLOG_URL + "?offset=1300&total=2000", copy(log, 1300, 2000));
        assertEquals("true", rest.headers().get("X-Complete"));
        assertArrayEquals(copy(log, 0, 2000), Files.readAllBytes(dir.resolve(DSLOG)));

        // After the next match: hashed as it arrived, so neither extend read the file back.
        r = extend(DSLOG_URL, 2000, 3000, copy(log, 2000, 3000), sha(log, 2000));
        assertEquals("true", r.headers().get("X-Complete"));
        assertArrayEquals(log, Files.readAllBytes(dir.resolve(DSLOG)));
        assertEquals(0, rx.filesHashed());
    }

    @Test
    void aFileFromAnEarlierBootIsHashedOnceThenCached() throws IOException {
        byte[] log = bytes(2000, 8);
        Files.write(dir.resolve(DSLOG), copy(log, 0, 1500));
        String wrong = sha(log, 1499);
        assertEquals(409, extend(DSLOG_URL, 1500, 2000, copy(log, 1500, 2000), wrong).status());
        assertEquals(1, rx.filesHashed());
        Reply r = extend(DSLOG_URL, 1500, 2000, copy(log, 1500, 2000), sha(log, 1500));
        assertEquals(200, r.status());
        assertEquals(1, rx.filesHashed(), "the retry used the cached hash");
        assertArrayEquals(log, Files.readAllBytes(dir.resolve(DSLOG)));
    }

    @Test
    void aMismatchIsA409AndLeavesTheFileAlone() throws IOException {
        byte[] old = bytes(1000, 1);
        put(DSLOG_URL + "?offset=0&total=1000", old);
        byte[] other = bytes(1500, 2); // a different session under the same name
        Reply r = extend(DSLOG_URL, 1000, 1500, copy(other, 1000, 1500), sha(other, 1000));
        assertEquals(409, r.status());
        assertEquals("mismatch", r.headers().get(DsUploadReceiver.EXTEND_HEADER));
        assertArrayEquals(old, Files.readAllBytes(dir.resolve(DSLOG)));
        assertFalse(Files.exists(dir.resolve(DSLOG + ".part")));
        // the client's fallback: the whole file from offset 0
        Reply again = put(DSLOG_URL + "?offset=0&total=1500", other);
        assertEquals("true", again.headers().get("X-Complete"));
        assertArrayEquals(other, Files.readAllBytes(dir.resolve(DSLOG)));
    }

    @Test
    void anExtendFromAnythingButTheWholeFinishedFileIsAMismatch() throws IOException {
        byte[] log = bytes(2000, 3);
        put(DSLOG_URL + "?offset=0&total=1000", copy(log, 0, 1000));
        Reply r = extend(DSLOG_URL, 900, 2000, copy(log, 900, 2000), sha(log, 900));
        assertEquals(409, r.status());
        assertEquals("mismatch", r.headers().get(DsUploadReceiver.EXTEND_HEADER));
        assertEquals(0, rx.filesHashed(), "a wrong length is refused without reading the file");
        String fresh = "/upload/dslog/2026_10_17%2009_00_00%20Sat.dslog";
        r = extend(fresh, 100, 200, bytes(100, 1), sha(log, 100));
        assertEquals("mismatch", r.headers().get(DsUploadReceiver.EXTEND_HEADER));
    }

    @Test
    void extendIsOnlyForDsLogs() throws IOException {
        put("/upload/audio/a.m4a?offset=0&total=10", bytes(10, 1));
        Reply h = head("/upload/audio/a.m4a");
        assertNull(h.headers().get(DsUploadReceiver.PREFIX_SUPPORTED_HEADER));
        Reply r = extend("/upload/audio/a.m4a", 10, 20, bytes(10, 2), sha(bytes(10, 1), 10));
        assertEquals(400, r.status());
        assertArrayEquals(bytes(10, 1), Files.readAllBytes(dir.resolve("a.m4a")));
    }

    @Test
    void aDsWpilogCanBeExtended() throws IOException {
        byte[] log = bytes(600, 4);
        String base = "/upload/dswpilog/FRC_DS_1.wpilog";
        put(base + "?offset=0&total=400", copy(log, 0, 400));
        assertEquals("1", head(base).headers().get(DsUploadReceiver.PREFIX_SUPPORTED_HEADER));
        Reply r = extend(base, 400, 600, copy(log, 400, 600), sha(log, 400));
        assertEquals("true", r.headers().get("X-Complete"));
        assertArrayEquals(log, Files.readAllBytes(dir.resolve("FRC_DS_1.dswpilog")));
    }

    @Test
    void aBadExtendRequestIsA400() throws IOException {
        byte[] log = bytes(1000, 5);
        put(DSLOG_URL + "?offset=0&total=1000", log);
        assertEquals(400, extend(DSLOG_URL, 1000, 1000, new byte[0], sha(log, 1000)).status());
        assertEquals(400, extend(DSLOG_URL, 1000, 1100, bytes(100, 1), "abc").status());
        assertArrayEquals(log, Files.readAllBytes(dir.resolve(DSLOG)));
    }

    @Test
    void anExtendWhileEnabledIsRefusedButNotAMismatch() throws IOException {
        byte[] log = bytes(1000, 5);
        put(DSLOG_URL + "?offset=0&total=1000", log);
        rx.setRobotEnabled(true);
        Reply r = extend(DSLOG_URL, 1000, 1100, bytes(100, 1), sha(log, 1000));
        assertEquals(409, r.status());
        assertNull(r.headers().get(DsUploadReceiver.EXTEND_HEADER));
    }
}
