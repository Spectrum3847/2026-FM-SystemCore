package frc.spectrumLib.gamepads;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.button.Trigger;

/**
 * Buttons shared between bare bindings and modifier chords (LB + A, RT + X, ...), resolved as one
 * answer from the buttons held. Press and release order cannot matter, and at most one of the
 * group's triggers is true at a time, so their edge bindings cannot race.
 *
 * <p>While any modifier in the group is held, the group's buttons belong to the chords: every bare
 * binding is off, and the first chord whose modifier and button are held (and that is enabled) is
 * on. Otherwise the first bare binding whose button is held is on. "First" is the order the
 * bindings were made, so make the one that should win first.
 *
 * <p>A bare binding waits {@code settleSeconds} before it counts, so a button pressed a loop before
 * the modifier, or let go a loop after it, does not blip the bare action. Chords act at once. Going
 * from a bare binding to a later one does not wait: that only happens by letting go of the first
 * button with the second still held.
 *
 * <p>Bind the rising edges of the chord and bare triggers, and the "nothing held" action to the
 * falling edge of {@link #any()}. An {@code onFalse} on each trigger would race the next one's
 * {@code onTrue}.
 *
 * <p>Only the face-button side is handled: the modifiers' own bindings (LT alone to intake, LT + LB
 * to eject) are untouched.
 *
 * <p>Reads the clock and the buttons, both AdvantageKit inputs, so it replays exactly. Its triggers
 * poll it several times a loop, so only the first call at a new timestamp advances it: the clock
 * must be the robot clock, which is pinned per loop.
 */
public final class ModifierChords {
    /** Long enough to cover a modifier and a button pressed a loop or two apart by hand. */
    public static final double DEFAULT_SETTLE_SECONDS = 0.12;

    private static final int NONE = -1;

    private record Binding(
            BooleanSupplier modifier, BooleanSupplier button, BooleanSupplier enabled) {
        boolean bare() {
            return modifier == null;
        }
    }

    private final DoubleSupplier clockSeconds;
    private final double settleSeconds;
    private final List<Binding> bindings = new ArrayList<>();

    private double lastTime = Double.NaN;
    private int output = NONE;

    /** The bare binding being waited on, and since when; NONE when not waiting. */
    private int pending = NONE;

    private double pendingSince = Double.NaN;

    /**
     * @param clockSeconds the robot clock, pinned per loop ({@code Timer::getTimestamp})
     * @param settleSeconds how long a bare button must be held, with no modifier, before it counts;
     *     0 to act at once
     */
    public ModifierChords(DoubleSupplier clockSeconds, double settleSeconds) {
        this.clockSeconds = clockSeconds;
        this.settleSeconds = settleSeconds;
    }

    /**
     * @param modifier a bumper, a trigger, or a combination such as {@code bothBumpers}
     * @param button the button pressed with it
     * @return true while this chord is the group's answer
     */
    public Trigger chord(BooleanSupplier modifier, BooleanSupplier button) {
        return chord(modifier, button, () -> true);
    }

    /**
     * A chord that only acts while {@code enabled} (teleop, say). While disabled its modifier still
     * keeps the bare bindings off.
     *
     * @param modifier a bumper, a trigger, or a combination such as {@code bothBumpers}
     * @param button the button pressed with it
     * @param enabled whether the chord may act
     * @return true while this chord is the group's answer
     */
    public Trigger chord(
            BooleanSupplier modifier, BooleanSupplier button, BooleanSupplier enabled) {
        return add(new Binding(modifier, button, enabled));
    }

    /**
     * @param button a button that also takes part in chords
     * @return true while {@code button} is the group's answer: held, with no modifier, past the
     *     settle time
     */
    public Trigger bare(BooleanSupplier button) {
        return add(new Binding(null, button, () -> true));
    }

    /**
     * @return true while any of the group's bindings is the answer; its falling edge is "nothing
     *     held"
     */
    public Trigger any() {
        return new Trigger(() -> active() != NONE);
    }

    private Trigger add(Binding binding) {
        bindings.add(binding);
        int index = bindings.size() - 1;
        return new Trigger(() -> active() == index);
    }

    private boolean isBare(int index) {
        return index != NONE && bindings.get(index).bare();
    }

    /** The binding held now, before the settle time. */
    private int resolve() {
        boolean modifierHeld = false;
        for (Binding b : bindings) {
            if (!b.bare() && b.modifier().getAsBoolean()) {
                modifierHeld = true;
                break;
            }
        }
        for (int i = 0; i < bindings.size(); i++) {
            Binding b = bindings.get(i);
            if (b.bare() == modifierHeld) {
                continue;
            }
            if ((b.bare() || b.modifier().getAsBoolean())
                    && b.button().getAsBoolean()
                    && b.enabled().getAsBoolean()) {
                return i;
            }
        }
        return NONE;
    }

    private int active() {
        double now = clockSeconds.getAsDouble();
        if (now == lastTime) {
            return output;
        }
        lastTime = now;
        int raw = resolve();
        if (!isBare(raw) || raw == output || (isBare(output) && raw > output)) {
            pending = NONE;
            output = raw;
            return output;
        }
        if (raw != pending) {
            pending = raw;
            pendingSince = now;
        }
        if (now - pendingSince >= settleSeconds) {
            pending = NONE;
            output = raw;
        } else if (!isBare(output)) {
            output = NONE;
        }
        return output;
    }
}
