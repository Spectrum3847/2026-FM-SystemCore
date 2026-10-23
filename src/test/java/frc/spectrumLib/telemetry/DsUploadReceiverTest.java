package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.telemetry.DsUploadReceiver.Reply;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
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
                        + "Host: 10.85.15.2:5806\r\n"
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
}
