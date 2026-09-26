package frc.robot.subsystems;

/**
 * Decides, once per loop, whether fuel may be fed into the flywheel. Pure logic: {@link
 * SuperStructure} fills an {@link Inputs} from logged inputs (motor inputs, the fused pose, the
 * Driver Station) and applies the {@link Decision}, so the whole gate replays and is unit tested.
 *
 * <p>FM on 2026 {@code main} fed the moment a launch state began. This is the offseason bot's feed
 * gate (commits dcb88dc, 067af02, ddd564a) with FM's drivetrain in place of that bot's turret:
 *
 * <ul>
 *   <li><b>Start</b> a volley only once the flywheel is within {@link Tolerances#flywheelStartRpm}
 *       of its target, the hood within {@link Tolerances#hoodStartDeg}, the heading within the aim
 *       tolerance, and the shot inside the model's range, all held for {@link
 *       Tolerances#settleSeconds}.
 *   <li><b>Keep</b> feeding on wider tolerances. Every ball loads the flywheel, so a gate that had
 *       to re-earn the strict window between balls would chop the feed on and off several times a
 *       second. 9470 latches flywheel readiness for the rest of the trigger hold; this keeps
 *       feeding while the flywheel stays above {@link Tolerances#flywheelKeepFraction} of target,
 *       which is the same latch with a floor for a flywheel that really has stalled. Range is not
 *       part of keeping (offseason ddd564a): it comes from the pose, and one bad frame mid-volley
 *       would chop the feed.
 *   <li><b>Time out</b>: blocked for {@link Tolerances#timeoutSeconds} in a launch state, feed
 *       anyway for the rest of that launch, so a stuck sensor or an unreachable target can delay a
 *       shot but never stop one. That is also what makes the gate safe in autos.
 *   <li><b>Override</b>: the operator's bypass feeds while it is held.
 * </ul>
 *
 * <p>Leaving the launch state resets everything, so the next volley re-earns the strict window.
 */
public final class ShotGate {

    /** Why the feed is being held, or {@link #NONE}. Logged as {@code Shot/BlockedReason}. */
    public enum Blocker {
        NONE("None"),
        /** Nothing to shoot at: the flywheel is not on a shot (idle, intaking). */
        NO_SHOT("NoShot"),
        FLYWHEEL("FlywheelNotReady"),
        HOOD("HoodNotReady"),
        AIM("NotAimed"),
        RANGE("InvalidShot"),
        /** Every check passes, but has not yet for {@link Tolerances#settleSeconds}. */
        SETTLING("Settling");

        public final String label;

        Blocker(String label) {
            this.label = label;
        }
    }

    /** Why fuel is (or is not) going in. Logged as {@code Shot/FeedReason}. */
    public enum FeedReason {
        NOT_LAUNCHING("NotLaunching"),
        HELD("Held"),
        READY("Ready"),
        OVERRIDE("Override"),
        TIMEOUT("Timeout");

        public final String label;

        FeedReason(String label) {
            this.label = label;
        }
    }

    /**
     * Gate tolerances.
     *
     * @param flywheelStartRpm largest flywheel error to start a volley at, RPM
     * @param flywheelKeepFraction smallest fraction of target RPM to keep feeding at
     * @param hoodStartDeg largest hood error to start a volley at, degrees
     * @param hoodKeepDeg largest hood error to keep feeding at, degrees
     * @param hoodStartMaxDegPerSec fastest the hood may be moving to start a volley, deg/s
     * @param aimKeepScale how much wider the keep-feeding heading tolerance is than the start one
     * @param settleSeconds how long every start check must hold before the volley starts
     * @param timeoutSeconds how long the gate may hold a launch before feeding anyway
     */
    public record Tolerances(
            double flywheelStartRpm,
            double flywheelKeepFraction,
            double hoodStartDeg,
            double hoodKeepDeg,
            double hoodStartMaxDegPerSec,
            double aimKeepScale,
            double settleSeconds,
            double timeoutSeconds) {

        /**
         * FM's numbers. Each is the offseason bot's, or FM's own constant, unless noted; CALIBRATE
         * against {@code Shot/*} and {@code ShotLog/*} from the first practice session.
         *
         * <ul>
         *   <li>Flywheel: {@code Launcher.onTargetToleranceRPM} (150 RPM; see there).
         *   <li>Keep feeding above 75% of target (offseason; its bench spin-up held inside the
         *       window during bursts, so this only has to ride the per-ball dip).
         *   <li>Hood 1 deg to start, 3 to keep. The offseason bot started at 0.5; FM's hood moves
         *       on Motion Magic toward a target that moves with distance while shooting on the
         *       move, and the model moves about 0.5 deg per 15 cm, so 1 deg is about 30 cm of
         *       range.
         *   <li>And the hood moving under 20 deg/s to start. In the sim the hood overshoots a 3.5
         *       deg move by 3 deg and swings back through the target at 35 deg/s; without this the
         *       volley opened on the way through, then chopped when the overshoot passed 3 deg.
         *       Shooting on the move the target itself moves a few deg/s at most.
         *   <li>Heading keep tolerance twice the start one (the offseason bot's turret was 2 to 6).
         *   <li>Settle 0.04 s: the offseason bot's three 20 ms loops, rounded to FM's 10 ms loop.
         *   <li>Timeout 1.0 s: longer than a spin-up from idle plus a half turn, short enough that
         *       an auto's 2.5 s launch window still has 1.5 s of feeding in the worst case.
         * </ul>
         *
         * @param flywheelStartRpm the flywheel's start tolerance
         * @return the tolerances
         */
        public static Tolerances fm(double flywheelStartRpm) {
            return new Tolerances(flywheelStartRpm, 0.75, 1.0, 3.0, 20.0, 2.0, 0.04, 1.0);
        }
    }

    /**
     * One loop's view of the shot, every field from a logged input or something computed from one.
     *
     * @param launching the super state is one that feeds
     * @param override the operator is holding the bypass
     * @param nowSeconds the robot clock (replayed, pinned per loop)
     * @param flywheelRpm measured flywheel speed
     * @param flywheelTargetRpm flywheel speed the shot wants; 0 or less when not on a shot
     * @param hoodDeg measured hood angle
     * @param hoodDegPerSec measured hood speed
     * @param hoodTargetDeg hood angle the shot wants
     * @param checkAim whether heading votes (false when nothing is aiming the drivetrain: the set
     *     shot, where the driver aims, and the X-brake launch)
     * @param headingErrorRad fused heading minus the heading being held
     * @param aimToleranceRad start tolerance for {@code headingErrorRad}
     * @param checkRange whether range votes (false for the set shot, and while the pose is not
     *     trusted, when range is not measuring anything)
     * @param shotValid the distance is inside the shot model's fitted range
     */
    public record Inputs(
            boolean launching,
            boolean override,
            double nowSeconds,
            double flywheelRpm,
            double flywheelTargetRpm,
            double hoodDeg,
            double hoodDegPerSec,
            double hoodTargetDeg,
            boolean checkAim,
            double headingErrorRad,
            double aimToleranceRad,
            boolean checkRange,
            boolean shotValid) {}

    /**
     * The gate's answer for one loop.
     *
     * @param ready every start check passes right now (what the driver waits for; also shown while
     *     tracking, before the trigger is pulled)
     * @param feed fuel may go into the flywheel this loop
     * @param reason why it may or may not
     * @param blocker what is holding the feed ({@link Blocker#NONE} while a volley is open); fed by
     *     the override or the timeout, what those are bypassing; not launching, what would hold a
     *     volley that started now
     * @param secondsHeld how long this launch has been held so far, 0 while feeding
     */
    public record Decision(
            boolean ready, boolean feed, FeedReason reason, Blocker blocker, double secondsHeld) {
        public static final Decision IDLE =
                new Decision(false, false, FeedReason.NOT_LAUNCHING, Blocker.NO_SHOT, 0);
    }

    private final Tolerances tolerances;

    /** True between a volley starting on the strict checks and the keep checks failing. */
    private boolean volleyOpen = false;

    /** When every strict check started passing, or NaN. */
    private double readySince = Double.NaN;

    /** When the feed was last held, or NaN while feeding. */
    private double heldSince = Double.NaN;

    /** The timeout fired this launch: feed for the rest of it. */
    private boolean timedOut = false;

    public ShotGate(Tolerances tolerances) {
        this.tolerances = tolerances;
    }

    public Tolerances getTolerances() {
        return tolerances;
    }

    /**
     * Heading tolerance for a shot: 4499's {@code atan((hubRadius - ballWidth) / distance)}, the
     * angle at which a ball's edge reaches the rim, for hub shots; a flat 10 deg for feed shots,
     * whose target is a patch of floor (9470 uses 10 deg for feed shots against 2.5 for the hub).
     * Clamped to 2 to 10 deg so a very long shot is not held to an angle the heading PID cannot
     * settle to and a very close one is not waved through at any heading.
     *
     * @param distanceMeters distance to the aim point
     * @param feedShot a feed (passing) shot rather than a hub shot
     * @return radians
     */
    public static double aimToleranceRad(double distanceMeters, boolean feedShot) {
        if (feedShot) {
            return FEED_AIM_TOLERANCE_RAD;
        }
        if (!(distanceMeters > 0)) {
            return MAX_AIM_TOLERANCE_RAD;
        }
        double tol = Math.atan((HUB_OPENING_RADIUS_METERS - FUEL_DIAMETER_METERS) / distanceMeters);
        return Math.clamp(tol, MIN_AIM_TOLERANCE_RAD, MAX_AIM_TOLERANCE_RAD);
    }

    /** Half of the hub's 41.7 in inner opening ({@code Field.BlueHub.innerOpeningWidth}). */
    public static final double HUB_OPENING_RADIUS_METERS = 41.7 * 0.0254 / 2.0;

    /** A fuel ball, 5.91 in (4499 uses 0.15 m). */
    public static final double FUEL_DIAMETER_METERS = 0.150;

    public static final double MIN_AIM_TOLERANCE_RAD = Math.toRadians(2.0);
    public static final double MAX_AIM_TOLERANCE_RAD = Math.toRadians(10.0);
    public static final double FEED_AIM_TOLERANCE_RAD = Math.toRadians(10.0);

    /**
     * Advances the gate one loop.
     *
     * @param in this loop's view of the shot
     * @return whether to feed, and why
     */
    public Decision update(Inputs in) {
        Tolerances tol = tolerances;
        boolean onShot = in.flywheelTargetRpm() > 0;
        double hoodError = Math.abs(in.hoodDeg() - in.hoodTargetDeg());
        double headingError = Math.abs(in.headingErrorRad());

        // Strict: what starting a volley takes. NaN compares false, so a missing value blocks.
        Blocker strict = Blocker.NONE;
        if (!onShot) {
            strict = Blocker.NO_SHOT;
        } else if (!(Math.abs(in.flywheelRpm() - in.flywheelTargetRpm())
                <= tol.flywheelStartRpm())) {
            strict = Blocker.FLYWHEEL;
        } else if (!(hoodError <= tol.hoodStartDeg())
                || !(Math.abs(in.hoodDegPerSec()) <= tol.hoodStartMaxDegPerSec())) {
            strict = Blocker.HOOD;
        } else if (in.checkAim() && !(headingError <= in.aimToleranceRad())) {
            strict = Blocker.AIM;
        } else if (in.checkRange() && !in.shotValid()) {
            strict = Blocker.RANGE;
        }
        boolean ready = strict == Blocker.NONE;

        if (!in.launching()) {
            volleyOpen = false;
            timedOut = false;
            readySince = Double.NaN;
            heldSince = Double.NaN;
            return new Decision(ready, false, FeedReason.NOT_LAUNCHING, strict, 0);
        }

        if (!ready) {
            readySince = Double.NaN;
        } else if (Double.isNaN(readySince)) {
            readySince = in.nowSeconds();
        }
        boolean settled = ready && in.nowSeconds() - readySince >= tol.settleSeconds() - 1e-9;

        boolean keep =
                onShot
                        && in.flywheelRpm() >= in.flywheelTargetRpm() * tol.flywheelKeepFraction()
                        && hoodError <= tol.hoodKeepDeg()
                        && (!in.checkAim()
                                || headingError <= in.aimToleranceRad() * tol.aimKeepScale());

        volleyOpen = volleyOpen ? keep : settled;

        if (volleyOpen || in.override()) {
            heldSince = Double.NaN;
        } else if (Double.isNaN(heldSince)) {
            heldSince = in.nowSeconds();
        }
        if (!Double.isNaN(heldSince)
                && in.nowSeconds() - heldSince >= tol.timeoutSeconds() - 1e-9) {
            timedOut = true;
        }

        FeedReason reason;
        if (volleyOpen) {
            reason = FeedReason.READY;
        } else if (in.override()) {
            reason = FeedReason.OVERRIDE;
        } else if (timedOut) {
            reason = FeedReason.TIMEOUT;
        } else {
            reason = FeedReason.HELD;
        }
        boolean feed = reason != FeedReason.HELD;
        // Fed by the override or the timeout, the blocker still says what is not ready, so the log
        // shows what was bypassed.
        Blocker blocker;
        if (reason == FeedReason.READY) {
            blocker = Blocker.NONE;
        } else {
            blocker = ready ? Blocker.SETTLING : strict;
        }
        double held = feed || Double.isNaN(heldSince) ? 0 : in.nowSeconds() - heldSince;
        return new Decision(ready, feed, reason, blocker, held);
    }
}
