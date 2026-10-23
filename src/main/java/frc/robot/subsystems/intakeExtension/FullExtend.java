package frc.robot.subsystems.intakeExtension;

/**
 * The full-extend decisions: drive out until both sides are still, then hold. Pure logic fed from
 * logged motor inputs, so it replays and is unit tested.
 *
 * <p>On the first extend after boot, a stall short of the target that barely moved, with both sides
 * agreeing, means the motors powered on with the intake out, so the stop is taken as max.
 *
 * <p>Nothing is decided while a side is disconnected. A CANivore drop reads every position and
 * velocity as 0, which looks like a stall far short of the stop (TXMAN1 Q34).
 */
final class FullExtend {

    enum Phase {
        DRIVING,
        HOLDING
    }

    enum Action {
        DRIVE,
        HOLD,
        /** Take the current position as max, then hold. */
        RELEARN
    }

    /**
     * @param maxRotations the out stop
     * @param atTargetToleranceRotations a stall this close to the stop counts as reaching it
     * @param steadySecs how long both sides must be still before holding
     * @param steadyMaxRpm still means slower than this on both sides
     * @param reextendRotations while holding, pushed back in by this much drives out again
     * @param relearnMaxTravelRotations relearn only when the extend moved less than this
     * @param relearnMaxSkewRotations relearn only when the sides agree to within this
     */
    record Settings(
            double maxRotations,
            double atTargetToleranceRotations,
            double steadySecs,
            double steadyMaxRpm,
            double reextendRotations,
            double relearnMaxTravelRotations,
            double relearnMaxSkewRotations) {

        static Settings from(IntakeExtension.IntakeExtensionConfig config) {
            return new Settings(
                    config.getMaxRotations(),
                    config.getExtendAtTargetToleranceRotations(),
                    config.getExtendSteadySecs(),
                    config.getExtendSteadyMaxRPM(),
                    config.getExtendReextendRotations(),
                    config.getRelearnMaxTravelRotations(),
                    config.getRelearnMaxSkewRotations());
        }
    }

    /**
     * One loop's readings.
     *
     * @param entered first loop of this full extend
     * @param connected every side in use is connected
     * @param nowMs robot clock, milliseconds, the unit WPILib's {@code Timer} subtracts in
     */
    record Inputs(
            boolean entered,
            boolean enabled,
            boolean connected,
            double nowMs,
            double leftRotations,
            double rightRotations,
            double leftRpm,
            double rightRpm) {}

    private final Settings settings;
    private Phase phase = Phase.DRIVING;
    private double steadySince = Double.NaN;
    private boolean startKnown = false;
    private double startRotations = 0;
    private double holdStartRotations = 0;
    // A power-on with the intake out is only possible before the first extend settles.
    private boolean outPointKnown = false;
    private int relearns = 0;

    FullExtend(Settings settings) {
        this.settings = settings;
    }

    Phase phase() {
        return phase;
    }

    int relearns() {
        return relearns;
    }

    Action update(Inputs in) {
        // Disabled, every extend looks like a stall.
        if (in.entered() || !in.enabled()) {
            phase = Phase.DRIVING;
            steadySince = Double.NaN;
            startKnown = false;
        }
        if (!in.connected()) {
            steadySince = Double.NaN;
            return phase == Phase.HOLDING ? Action.HOLD : Action.DRIVE;
        }

        double position = (in.leftRotations() + in.rightRotations()) / 2.0;
        if (!startKnown) {
            startKnown = true;
            startRotations = position;
        }

        boolean still =
                Math.abs(in.leftRpm()) < settings.steadyMaxRpm()
                        && Math.abs(in.rightRpm()) < settings.steadyMaxRpm();
        if (!still || !in.enabled()) {
            steadySince = Double.NaN;
        } else if (Double.isNaN(steadySince)) {
            steadySince = in.nowMs();
        }
        boolean steady =
                !Double.isNaN(steadySince)
                        && (in.nowMs() - steadySince) / 1000.0 >= settings.steadySecs();

        return switch (phase) {
            case DRIVING -> {
                if (!steady) {
                    yield Action.DRIVE;
                }
                boolean atTarget =
                        position >= settings.maxRotations() - settings.atTargetToleranceRotations();
                boolean barelyMoved =
                        Math.abs(position - startRotations) < settings.relearnMaxTravelRotations();
                boolean sidesAgree =
                        Math.abs(in.leftRotations() - in.rightRotations())
                                <= settings.relearnMaxSkewRotations();
                boolean relearn = !outPointKnown && !atTarget && barelyMoved && sidesAgree;
                outPointKnown = true;
                phase = Phase.HOLDING;
                if (relearn) {
                    relearns++;
                    holdStartRotations = settings.maxRotations();
                    yield Action.RELEARN;
                }
                holdStartRotations = position;
                yield Action.HOLD;
            }
            case HOLDING -> {
                if (holdStartRotations - position >= settings.reextendRotations()) {
                    phase = Phase.DRIVING;
                    startRotations = position;
                    steadySince = Double.NaN;
                    yield Action.DRIVE;
                }
                yield Action.HOLD;
            }
        };
    }
}
