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
        assertEquals(
                200, put(base + "?offset=0&total=3000", Arrays.copyOfRange(all, 0, 1000)).status());
        assertFalse(new File(dir.toFile(), "TXMAN1_Q34.mp4").exists());
        assertEquals("1000", head(base).headers().get("X-Received"));
        assertEquals(
                200,
                put(base + "?offset=1000&total=3000", Arrays.copyOfRange(all, 1000, 3000))
                        .status());
        File done = new File(dir.toFile(), "TXMAN1_Q34.mp4");
        assertArrayEquals(all, Files.readAllBytes(done.toPath()));
        assertFalse(new File(dir.toFile(), "TXMAN1_Q34.mp4.part").exists());
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
        assertArrayEquals(body, Files.readAllBytes(dir.resolve("FRC_DS_Q34.wpilog")));
    }
}
