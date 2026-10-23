package frc.robot.subsystems.hood;

/**
 * Finds the hood's bottom stop, so its angle does not depend on where it rested at power-on. Pure
 * logic: {@link Hood} fills an {@link Inputs} from logged inputs each loop and applies the {@link
 * Output}, so homing replays and is unit tested.
 *
 * <p>Homing drives the hood down at {@link Params#volts} until its speed stays under {@link
 * Params#stallDegPerSec} for {@link Params#stallSeconds}, then sets the position to the stop. It
 * runs at auto start, at teleop start unless already zeroed, and each time the hood is sent home
 * and gets within {@link Params#nearHomeDeg} of the stop (1678's re-home on stow). Until the first
 * success the hood refuses closed-loop setpoints and holds in brake. A reboot of the hood motor
 * restarts its position, so it un-zeroes the hood and re-homes it at once if enabled.
 */
public final class HoodHoming {

    /** Logged as {@code Hood/HomingState}. */
    public enum State {
        /** Never homed since boot: the hood holds in brake. */
        NOT_ZEROED("NotZeroed"),
        HOMING("Homing"),
        /** The last attempt ran out of time before finding the stop, and the hood is not zeroed. */
        TIMED_OUT("TimedOut"),
        ZEROED("Zeroed");

        public final String label;

        State(String label) {
            this.label = label;
        }
    }

    public enum Action {
        /** Neutral output; the hood is in brake. */
        HOLD,
        /** Open-loop {@link Params#volts}, ignoring the soft limits. */
        DRIVE_TO_STOP,
        /** Follow the wanted state's setpoint. */
        CLOSED_LOOP,
    }

    /**
     * Homing constants.
     *
     * @param stopDeg hood angle at the bottom stop, degrees
     * @param volts output while homing, toward the stop (negative)
     * @param stallDegPerSec the hood is stalled while slower than this, deg/s
     * @param stallSeconds how long it must stay stalled to count as on the stop
     * @param timeoutSeconds how long an attempt may run before giving up
     * @param nearHomeDeg re-home on stow once this close to the stop, degrees
     */
    public record Params(
            double stopDeg,
            double volts,
            double stallDegPerSec,
            double stallSeconds,
            double timeoutSeconds,
            double nearHomeDeg) {

        /**
         * FM's numbers. CALIBRATE on the bench (README, "Hood homing").
         *
         * <ul>
         *   <li>Stop 9 deg: the angle FM's code has always seeded at boot and sent home to.
         *   <li>-1.5 V: about 0.6 V goes to the 25 A kS (Kraken FOC is about 0.025 ohm), the rest
         *       moves the hood at roughly 50 deg/s, so the 40 deg of travel takes under a second.
         *       Stalled, the 60 A stator limit caps the push. 6328 homes at -2 V.
         *   <li>Stalled under 4 deg/s for 0.2 s: 4 deg/s is a tenth of the homing speed, and 0.2 s
         *       covers the hood breaking away from rest and the velocity signal's lag, so a hood
         *       that starts above the stop is not taken as stalled before it moves.
         *   <li>Timeout 3 s: three times the full-travel time.
         *   <li>Re-home on stow within 2 deg: a stow re-home then corrects at most a couple of
         *       degrees, and costs about 0.25 s at the stop.
         * </ul>
         *
         * @return the constants
         */
        public static Params fm() {
            return new Params(9.0, -1.5, 4.0, 0.2, 3.0, 2.0);
        }
    }

    /**
     * One loop's view of the hood, every field from a logged input.
     *
     * @param nowSeconds the robot clock
     * @param enabled the robot is enabled
     * @param autonomous the Driver Station is in autonomous
     * @param teleop the Driver Station is in teleop
     * @param motorConnected the hood motor's signals are arriving
     * @param motorReset the hood motor rebooted this loop, restarting its position
     * @param positionDeg measured hood angle
     * @param degPerSec measured hood speed
     * @param wantsHome the hood is being sent to its stop
     */
    public record Inputs(
            double nowSeconds,
            boolean enabled,
            boolean autonomous,
            boolean teleop,
            boolean motorConnected,
            boolean motorReset,
            double positionDeg,
            double degPerSec,
            boolean wantsHome) {}

    /**
     * The homing answer for one loop.
     *
     * @param action what the hood motor does
     * @param resetPosition set the motor's position to {@link Params#stopDeg} this loop
     * @param zeroed the hood's angle can be trusted
     * @param state logged state
     * @param timedOut the last attempt timed out (drives the alert)
     */
    public record Output(
            Action action, boolean resetPosition, boolean zeroed, State state, boolean timedOut) {}

    private final Params params;

    private boolean zeroed = false;
    private boolean timedOut = false;
    private boolean homing = false;

    /** An attempt is due: set by a mode start, kept until it can run or the robot disables. */
    private boolean requested = false;

    /** The current attempt may stop when the hood is wanted elsewhere (a re-home on stow). */
    private boolean abortable = false;

    /** Homed on this visit home: rest on the stop, and do not re-home until the hood leaves. */
    private boolean homedThisStow = false;

    private boolean wasAutoEnabled = false;
    private boolean wasTeleopEnabled = false;
    private double homingStart = Double.NaN;
    private double stallSince = Double.NaN;

    public HoodHoming(Params params) {
        this.params = params;
    }

    public Params getParams() {
        return params;
    }

    public boolean isZeroed() {
        return zeroed;
    }

    /**
     * Advances homing one loop.
     *
     * @param in this loop's view of the hood
     * @return what the hood does
     */
    public Output update(Inputs in) {
        boolean autoEnabled = in.enabled() && in.autonomous();
        boolean teleopEnabled = in.enabled() && in.teleop();
        if (autoEnabled && !wasAutoEnabled) {
            requested = true;
        }
        if (teleopEnabled && !wasTeleopEnabled && !zeroed) {
            requested = true;
        }
        wasAutoEnabled = autoEnabled;
        wasTeleopEnabled = teleopEnabled;
        // The reboot erased the zero, and an attempt under way would take the reboot's standstill
        // for the stop. Re-home now if enabled, else at the next mode start.
        if (in.motorReset()) {
            zeroed = false;
            homing = false;
            homedThisStow = false;
            requested = true;
        }
        if (!in.wantsHome()) {
            homedThisStow = false;
        }

        if (!in.enabled()) {
            homing = false;
            requested = false;
            return output(false);
        }

        if (homing && !abortable) {
            requested = false; // already doing what a mode start asks
        }
        // Any attempt stops if the motor drops out: its speed would read as a stall. A re-home on
        // stow stops when the hood is wanted elsewhere, or for a mode start's attempt.
        if (homing && (!in.motorConnected() || (abortable && (!in.wantsHome() || requested)))) {
            homing = false;
        }

        if (!homing && in.motorConnected()) {
            if (requested) {
                // A mode start re-homes even a zeroed hood, so it does not count as zeroed
                // meanwhile.
                start(in.nowSeconds(), false);
                zeroed = false;
            } else if (zeroed
                    && in.wantsHome()
                    && !homedThisStow
                    && Math.abs(in.positionDeg() - params.stopDeg()) <= params.nearHomeDeg()) {
                start(in.nowSeconds(), true);
            }
        }

        if (!homing) {
            return output(false);
        }

        if (Math.abs(in.degPerSec()) < params.stallDegPerSec()) {
            if (Double.isNaN(stallSince)) {
                stallSince = in.nowSeconds();
            }
        } else {
            stallSince = Double.NaN;
        }
        if (!Double.isNaN(stallSince)
                && in.nowSeconds() - stallSince >= params.stallSeconds() - 1e-9) {
            homing = false;
            zeroed = true;
            timedOut = false;
            homedThisStow = in.wantsHome();
            return output(true);
        }
        if (in.nowSeconds() - homingStart >= params.timeoutSeconds() - 1e-9) {
            homing = false;
            timedOut = true;
            // A re-home on stow that fails keeps the zero it started with.
            homedThisStow = in.wantsHome();
            return output(false);
        }
        return new Output(Action.DRIVE_TO_STOP, false, zeroed, State.HOMING, timedOut);
    }

    private void start(double now, boolean canAbort) {
        homing = true;
        requested = false;
        abortable = canAbort;
        homingStart = now;
        stallSince = Double.NaN;
    }

    private Output output(boolean resetPosition) {
        State state;
        if (homing) {
            state = State.HOMING;
        } else if (zeroed) {
            state = State.ZEROED;
        } else if (timedOut) {
            state = State.TIMED_OUT;
        } else {
            state = State.NOT_ZEROED;
        }
        Action action;
        if (homing) {
            action = Action.DRIVE_TO_STOP;
        } else if (!zeroed || homedThisStow) {
            // Not zeroed: refuse setpoints. Homed on this stow: rest on the stop rather than
            // hold a setpoint against it.
            action = Action.HOLD;
        } else {
            action = Action.CLOSED_LOOP;
        }
        return new Output(action, resetPosition, zeroed, state, timedOut);
    }
}
