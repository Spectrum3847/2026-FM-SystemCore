package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;

class RadioStatusRedactorTest {
    // The shape of the VH-109's status, whitespace stripped as AdvantageKit logs it (made-up
    // values).
    private static final String STATUS =
            "{\"mode\":\"TEAM_ROBOT_RADIO\",\"teamNumber\":8515,"
                    + "\"networkStatus24\":{\"ssid\":\"FRC-8515\",\"hashedWpaKey\":\"0123456789abcdef\","
                    + "\"wpaKeySalt\":\"fedcba98\",\"isLinked\":false,\"macAddress\":\"\"},"
                    + "\"networkStatus6\":{\"ssid\":\"8515\",\"hashedWpaKey\":\"aaaabbbbccccdddd\","
                    + "\"wpaKeySalt\":\"11112222\",\"isLinked\":true,"
                    + "\"macAddress\":\"4A:00:00:00:00:01\",\"signalDbm\":-54},"
                    + "\"status\":\"ACTIVE\",\"version\":\"VH-109_2.0.1-02062026\"}";

    @Test
    void blanksKeysSaltsAndMacAndKeepsTheRest() {
        String out = RadioStatusRedactor.redact(STATUS);
        assertFalse(out.contains("0123456789abcdef"));
        assertFalse(out.contains("aaaabbbbccccdddd"));
        assertFalse(out.contains("fedcba98"));
        assertFalse(out.contains("11112222"));
        assertFalse(out.contains("4A:00:00:00:00:01"));
        assertTrue(out.contains("\"hashedWpaKey\":\"\""));
        assertTrue(out.contains("\"ssid\":\"FRC-8515\""));
        assertTrue(out.contains("\"signalDbm\":-54"));
        assertTrue(out.contains("\"version\":\"VH-109_2.0.1-02062026\""));
        assertEquals(RadioStatusRedactor.redact(out), out, "redacting twice changes nothing");
    }

    @Test
    void rewritesTheTableBeforeTheOtherReceivers() throws InterruptedException {
        LogTable table = new LogTable(0);
        table.put(RadioStatusRedactor.KEY, new LogValue(STATUS, "json"));
        new RadioStatusRedactor().putTable(table);
        LogValue v = table.get(RadioStatusRedactor.KEY);
        assertEquals("json", v.customTypeStr, "the log keeps the json type");
        assertFalse(v.getString().contains("aaaabbbbccccdddd"));
    }

    @Test
    void leavesTablesWithoutTheStatusAlone() throws InterruptedException {
        LogTable table = new LogTable(0);
        table.put("RadioStatus/Connected", true);
        new RadioStatusRedactor().putTable(table);
        assertTrue(table.get("RadioStatus/Connected").getBoolean());
    }
}
