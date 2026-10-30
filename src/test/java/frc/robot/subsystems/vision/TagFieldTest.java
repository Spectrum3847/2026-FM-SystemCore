package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.vision.FieldMapHash;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;

class TagFieldTest {
    private static final Fields ANDYMARK = Fields.FRC_2026_REBUILT_ANDY_MARK;
    private static final Fields WELDED = Fields.FRC_2026_REBUILT_WELDED;
    private static final String ANDYMARK_HASH = FieldMapHash.of(Field.loadField(ANDYMARK));

    @Test
    void replayUsesTheLoggedFieldNotTheEvents() {
        TagField.Choice c = TagField.forReplay(WELDED, ANDYMARK.name(), ANDYMARK_HASH);
        assertEquals(ANDYMARK, c.field());
        assertEquals(ANDYMARK_HASH, c.hash());
        assertEquals(Field.loadField(ANDYMARK), c.layout());
        assertNull(c.warning());
    }

    @Test
    void anOldLogFallsBackToTheEventAndWarns() {
        TagField.Choice c = TagField.forReplay(WELDED, TagField.ABSENT, TagField.ABSENT);
        assertEquals(WELDED, c.field());
        assertNotNull(c.warning());
        assertTrue(c.warning().contains("replay may differ"), c.warning());
    }

    @Test
    void anUnknownFieldFallsBackToTheEventAndWarns() {
        TagField.Choice c = TagField.forReplay(ANDYMARK, "custom", "0123456789abcdef");
        assertEquals(ANDYMARK, c.field());
        assertTrue(c.warning().contains("custom"), c.warning());
    }

    @Test
    void aHashThatNoLongerMatchesKeepsTheFieldButWarns() {
        TagField.Choice c = TagField.forReplay(WELDED, ANDYMARK.name(), "0123456789abcdef");
        assertEquals(ANDYMARK, c.field());
        assertTrue(c.warning().contains("0123456789abcdef"), c.warning());
    }

    @Test
    void aLogWithANameButNoHashIsTrusted() {
        assertNull(TagField.forReplay(WELDED, ANDYMARK.name(), TagField.ABSENT).warning());
    }
}
