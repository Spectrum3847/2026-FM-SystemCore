package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;

class RadioStatusRedactorTest {
    // The shape of the VH-109's status, whitespace stripped as AdvantageKit logs it (made-up
    // values; the fields are those in the TXMAN1 logs).
    private static final String STATUS =
            "{\"mode\":\"TEAM_ROBOT_RADIO\",\"channel\":\"\",\"teamNumber\":8515,"
                    + "\"ssidSuffix\":\"\","
                    + "\"networkStatus24\":{\"ssid\":\"FRC-8515\",\"isEnabled\":false,"
                    + "\"hashedWpaKey\":\"0123456789abcdef\",\"wpaKeySalt\":\"fedcba98\","
                    + "\"isLinked\":false,\"macAddress\":\"\",\"dataAgeMs\":0,\"signalDbm\":0,"
                    + "\"noiseDbm\":0,\"signalNoiseRatio\":-999,\"rxRateMbps\":-999,"
                    + "\"rxPackets\":0,\"rxBytes\":-999,\"txRateMbps\":-999,\"txPackets\":0,"
                    + "\"txBytes\":-999,\"bandwidthUsedMbps\":-999,\"connectionQuality\":\"\"},"
                    + "\"networkStatus6\":{\"ssid\":\"8515\",\"isEnabled\":true,"
                    + "\"hashedWpaKey\":\"aaaabbbbccccdddd\",\"wpaKeySalt\":\"11112222\","
                    + "\"isLinked\":true,\"macAddress\":\"4A:00:00:00:00:01\",\"signalDbm\":-54,"
                    + "\"rxRateMbps\":1200.5},"
                    + "\"status\":\"ACTIVE\",\"version\":\"VH-109_2.0.1-02062026\","
                    + "\"sixGhzBypassed\":false,\"qosEnabled\":true,\"systemcoreEnabled\":true}";

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private static JsonNode redacted(String json) throws IOException {
        return MAPPER.readTree(RadioStatusRedactor.redact(json));
    }

    @Test
    void keepsTheAllowedFieldsAndBlanksKeysSaltsAndMacs() throws IOException {
        String out = RadioStatusRedactor.redact(STATUS);
        for (String secret :
                List.of("0123456789abcdef", "aaaabbbbccccdddd", "fedcba98", "11112222", "4A:00")) {
            assertFalse(out.contains(secret), secret);
        }
        JsonNode j = MAPPER.readTree(out);
        assertEquals("TEAM_ROBOT_RADIO", j.get("mode").asText());
        assertEquals(8515, j.get("teamNumber").asInt());
        assertEquals("ACTIVE", j.get("status").asText());
        assertEquals("VH-109_2.0.1-02062026", j.get("version").asText());
        assertTrue(j.get("qosEnabled").asBoolean());
        JsonNode six = j.get("networkStatus6");
        assertEquals("8515", six.get("ssid").asText());
        assertEquals(-54, six.get("signalDbm").asInt());
        assertEquals(1200.5, six.get("rxRateMbps").asDouble());
        assertTrue(six.get("isLinked").asBoolean());
        assertEquals("", six.get("hashedWpaKey").asText());
        assertEquals("", six.get("wpaKeySalt").asText());
        assertEquals("", six.get("macAddress").asText());
        assertEquals(-999, j.get("networkStatus24").get("txBytes").asInt());
        assertEquals(out, RadioStatusRedactor.redact(out), "redacting twice changes nothing");
    }

    @Test
    void anEscapedQuoteDoesNotEndTheSecret() throws IOException {
        String in = "{\"networkStatus6\":{\"hashedWpaKey\":\"abc\\\"def0123\",\"ssid\":\"x\"}}";
        String out = RadioStatusRedactor.redact(in);
        assertFalse(out.contains("def0123"), out);
        assertEquals("", redacted(in).get("networkStatus6").get("hashedWpaKey").asText());
    }

    @Test
    void unknownFieldsAreBlankedWhateverTheirTypeOrName() throws IOException {
        String in =
                "{\"psk\":\"hunter2\",\"bssid\":\"4A:00:00:00:00:02\",\"hash\":\"deadbeef\","
                        + "\"wpaKeyNumber\":123456789,\"keyed\":true,\"nothing\":null,"
                        + "\"secrets\":{\"key\":\"s3cret\",\"n\":42},"
                        + "\"keys\":[\"k1\",\"k2\",{\"k\":\"k3\"}]}";
        String out = RadioStatusRedactor.redact(in);
        for (String secret :
                List.of("hunter2", "4A:00", "deadbeef", "123456789", "s3cret", "42", "k1", "k3")) {
            assertFalse(out.contains(secret), secret + " in " + out);
        }
        JsonNode j = MAPPER.readTree(out);
        assertEquals("", j.get("psk").asText());
        assertEquals("", j.get("bssid").asText());
        assertEquals("", j.get("hash").asText());
        assertEquals(0, j.get("wpaKeyNumber").asLong());
        assertFalse(j.get("keyed").asBoolean());
        assertTrue(j.get("nothing").isNull());
        assertTrue(j.get("secrets").isObject() && j.get("secrets").isEmpty());
        assertTrue(j.get("keys").isArray() && j.get("keys").isEmpty());
    }

    @Test
    void anAllowedNameHoldingAnObjectOrArrayIsEmptied() throws IOException {
        String in =
                "{\"ssid\":{\"hashedWpaKey\":\"s3cret\"},\"status\":[\"s3cret\"],"
                        + "\"networkStatus6\":{\"version\":{\"psk\":\"s3cret\"}}}";
        assertFalse(RadioStatusRedactor.redact(in).contains("s3cret"));
        JsonNode j = redacted(in);
        assertTrue(j.get("ssid").isObject() && j.get("ssid").isEmpty());
        assertTrue(j.get("status").isArray() && j.get("status").isEmpty());
    }

    @Test
    void onlyTheTwoNetworkObjectsAreLookedInside() throws IOException {
        // A new band or a list of clients in a future firmware: emptied, not trusted.
        String in =
                "{\"networkStatus5\":{\"ssid\":\"FRC-8515\",\"psk\":\"s3cret\"},"
                        + "\"networkStatus24\":{\"clients\":[{\"macAddress\":\"4A:00\"}],"
                        + "\"ssid\":\"FRC-8515\"},\"networkStatus6\":\"s3cret\"}";
        String out = RadioStatusRedactor.redact(in);
        assertFalse(out.contains("s3cret"));
        assertFalse(out.contains("4A:00"));
        JsonNode j = MAPPER.readTree(out);
        assertTrue(j.get("networkStatus5").isEmpty());
        assertTrue(j.get("networkStatus24").get("clients").isEmpty());
        assertEquals("FRC-8515", j.get("networkStatus24").get("ssid").asText());
        assertEquals("", j.get("networkStatus6").asText());
    }

    @Test
    void anythingThatDoesNotParseAsAnObjectBecomesEmpty() {
        for (String in :
                List.of(
                        "",
                        "not json \"hashedWpaKey\":\"s3cret\"",
                        "{\"hashedWpaKey\":\"s3cret\"",
                        "{\"ssid\":\"x\"} {\"hashedWpaKey\":\"s3cret\"}",
                        "[{\"hashedWpaKey\":\"s3cret\"}]",
                        "\"s3cret\"",
                        "{\"ssid\":\"x\",\"psk\":\"s3cret\",}")) {
            assertEquals(RadioStatusRedactor.UNPARSABLE, RadioStatusRedactor.redact(in), in);
        }
    }

    @Test
    void rewritesTheTableBeforeTheOtherReceivers() {
        LogTable table = new LogTable(0);
        table.put(RadioStatusRedactor.KEY, new LogValue(STATUS, "json"));
        var redactor = new RadioStatusRedactor();
        redactor.putTable(table);
        LogValue v = table.get(RadioStatusRedactor.KEY);
        assertEquals("json", v.customTypeStr, "the log keeps the json type");
        assertFalse(v.getString().contains("aaaabbbbccccdddd"));

        // The next cycle repeats the status (cached), then the radio sends one that won't parse.
        LogTable next = new LogTable(0);
        next.put(RadioStatusRedactor.KEY, new LogValue(STATUS, "json"));
        redactor.putTable(next);
        assertEquals(v.getString(), next.get(RadioStatusRedactor.KEY).getString());
        LogTable broken = new LogTable(0);
        broken.put(RadioStatusRedactor.KEY, new LogValue("{\"psk\":\"s3cret\"", "json"));
        redactor.putTable(broken);
        assertEquals("{}", broken.get(RadioStatusRedactor.KEY).getString());
        assertEquals("json", broken.get(RadioStatusRedactor.KEY).customTypeStr);
    }

    @Test
    void leavesTablesWithoutTheStatusAlone() {
        LogTable table = new LogTable(0);
        table.put("RadioStatus/Connected", true);
        new RadioStatusRedactor().putTable(table);
        assertTrue(table.get("RadioStatus/Connected").getBoolean());
    }

    /**
     * Every data receiver is added through {@link RadioStatusRedactor#addFirst}, so none can get
     * the table before the redactor. The robot-boot check is in {@code BindingsSmokeTest}.
     */
    @Test
    void noCodeAddsADataReceiverAroundTheRedactor() throws IOException {
        List<String> offenders = new ArrayList<>();
        try (Stream<Path> files = Files.walk(Path.of("src", "main", "java"))) {
            for (Path p : files.filter(f -> f.toString().endsWith(".java")).toList()) {
                if (p.getFileName().toString().equals("RadioStatusRedactor.java")) {
                    continue;
                }
                if (Files.readString(p).contains("addDataReceiver(")) {
                    offenders.add(p.toString());
                }
            }
        }
        assertEquals(List.of(), offenders, "add receivers with RadioStatusRedactor.addFirst");
    }
}
