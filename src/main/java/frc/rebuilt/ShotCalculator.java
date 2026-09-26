package frc.rebuilt;

import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

@SuppressWarnings("unused")
public class ShotCalculator {

    // =========================================================================
    // Singleton
    // =========================================================================

    private static ShotCalculator instance;

    /** Robot-centre to launcher offset. Zero = launcher is at robot centre. */
    private static final Transform2d robotToLauncher = Transform2d.kZero;

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    // =========================================================================
    // Shot Parameters Record
    // =========================================================================

    /**
     * Immutable snapshot of all quantities needed to command the drive, hood, and flywheel
     * subsystems for a single shot.
     */
    public record ShootingParameters(
            /** {@code true} when distance is within the polynomial's fitted range. */
            boolean isValid,
            /** Field-relative heading the robot must face to aim at the goal. */
            Rotation2d driveAngle,
            /**
             * How fast the bearing to the goal is turning (rad/s, counter-clockwise positive), for
             * the drivetrain's heading feedforward. See {@link #bearingRateRadPerSec}.
             */
            double driveAngularVelocity,
            /** Commanded hood/pivot angle (degrees), including {@link #HOOD_ANGLE_OFFSET}. */
            double hoodAngle,
            /** Rate of change of {@code hoodAngle} (deg/s) for pivot feedforward. */
            double hoodVelocity,
            /** Commanded flywheel speed (RPM). */
            double flywheelSpeed,
            /** Ball exit speed from the polynomial (m/s), before RPM conversion. */
            double exitSpeedMs,
            /** Shoot-on-move compensated distance to goal (metres). */
            double distance,
            /** Raw uncompensated distance to goal (metres). */
            double distanceNoLookahead,
            /** Estimated ball time-of-flight (seconds). */
            double timeOfFlight,
            /** Launcher velocity toward the goal (m/s), positive closing. */
            double radialVelocity,
            /** Launcher velocity across the line to the goal (m/s). */
            double tangentialVelocity,
            /** A feed (passing) shot rather than a hub shot. */
            boolean feedShot,
            /** Name of the fitted model the shot was evaluated on. */
            String modelName) {}

    private ShootingParameters latestParameters = null;

    // =========================================================================
    // Runtime-Adjustable Offsets
    // =========================================================================

    public static final double STARTING_HOOD_ANGLE_OFFSET = -2; // degrees
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    public static final double STARTING_DRIVE_ANGLE_OFFSET = 0; // degrees
    public static double DRIVE_ANGLE_OFFSET = STARTING_DRIVE_ANGLE_OFFSET;

    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET += 0.1).ignoringDisable(true);
    }

    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET -= 0.1).ignoringDisable(true);
    }

    public static Command increaseDriveAngleOffset() {
        return Commands.runOnce(() -> DRIVE_ANGLE_OFFSET += 1).ignoringDisable(true);
    }

    public static Command decreaseDriveAngleOffset() {
        return Commands.runOnce(() -> DRIVE_ANGLE_OFFSET -= 1).ignoringDisable(true);
    }

    // =========================================================================
    // Polynomial Model
    // =========================================================================
    // 2D degree-3 polynomial surface:
    //   f(distance_m, radialVel_ms) → { exitSpeed_ms, launchAngle_deg }
    // Monomial basis: 1, d, v, d², d·v, v², d³, d²·v, d·v², v³

    /**
     * Global exit-speed scale factor. Adjust post-characterization to correct for ball compression,
     * wear, or temperature without re-fitting the polynomial. 1.0 = no scaling. Applied to both the
     * hub and feed models.
     */
    private static final double MPS_FACTOR = 0.8;

    /** Scale factor converting polynomial exit speed (m/s) to flywheel RPM. */
    private static final double RPM_PER_MPS = 255.0;

    /**
     * The ball exit speed a flywheel speed stands for, the inverse of the model's RPM conversion.
     *
     * @param rpm flywheel speed
     * @return exit speed, m/s
     */
    public static double exitSpeedForFlywheelRpm(double rpm) {
        return rpm / RPM_PER_MPS;
    }

    /**
     * A fitted degree-3 polynomial surface plus its input domain and normalisation. Inputs are
     * mapped to zero-mean unit-variance before evaluation, so the coefficients live in normalised
     * space and must not be applied to raw (metres / m/s) inputs directly.
     *
     * @param name descriptive name for telemetry
     * @param distMin fitted distance lower bound (metres); inputs clamped, shots outside flagged
     *     invalid
     * @param distMax fitted distance upper bound (metres)
     * @param rvMin fitted radial-velocity lower bound (m/s)
     * @param rvMax fitted radial-velocity upper bound (m/s)
     * @param dMean distance normalisation mean
     * @param dStd distance normalisation standard deviation
     * @param vMean radial-velocity normalisation mean
     * @param vStd radial-velocity normalisation standard deviation
     * @param speedCoeffs exit-speed coefficients in the monomial basis 1, d, v, d², d·v, v², d³,
     *     d²·v, d·v², v³
     * @param angleCoeffs launch-angle coefficients in the same basis
     */
    private record PolyModel(
            String name,
            double distMin,
            double distMax,
            double rvMin,
            double rvMax,
            double dMean,
            double dStd,
            double vMean,
            double vStd,
            double[] speedCoeffs,
            double[] angleCoeffs) {}

    /** Hub-shot model — used when the robot is in a scoring zone. */
    private static final PolyModel NO_CEILING_HUB_MODEL =
            new PolyModel(
                    "No Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.7946224256, // dMean
                    1.9514199579, // dStd
                    -0.0434782609, // vMean
                    1.9813242725, // vStd
                    new double[] {
                        /* 1    */ 1.1143628795e+1,
                        /* d    */ 1.0138658152e+0,
                        /* v    */ -3.2159777567e-1,
                        /* d²   */ -5.1612304349e-2,
                        /* d·v  */ 3.6484374359e-1,
                        /* v²   */ -1.7402563290e-1,
                        /* d³   */ 7.9863642916e-2,
                        /* d²·v */ -1.2476148148e-1,
                        /* d·v² */ 3.8502387398e-1,
                        /* v³   */ -3.2039252056e-1
                    },
                    new double[] {
                        /* 1    */ 6.6464926591e+1,
                        /* d    */ -7.7256852841e+0,
                        /* v    */ 1.1734641473e+1,
                        /* d²   */ 8.1692458382e-3,
                        /* d·v  */ 5.2897492237e-1,
                        /* v²   */ -2.6845198119e-1,
                        /* d³   */ 1.7588060294e-1,
                        /* d²·v */ -2.0564699991e-3,
                        /* d·v² */ 1.2509312471e+0,
                        /* v³   */ -1.4532157984e+0
                    });

    /** 3 meter ceiling hub model - used when the robot is testing at home */
    private static final PolyModel CEILING_3M_HUB_MODEL =
            new PolyModel(
                    "3 Meter Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.7330253114, // dMean
                    1.8890844725, // dStd
                    0.0229007634, // vMean
                    1.9319161427, // vStd
                    new double[] {
                        /* 1    */ 9.1291597222e+0,
                        /* d    */ 1.4704411927e+0,
                        /* v    */ -1.1245274618e+0,
                        /* d²   */ 5.9711528113e-2,
                        /* d·v  */ -1.0395358193e-1,
                        /* v²   */ 6.1638746813e-2,
                        /* d³   */ -3.2358373131e-2,
                        /* d²·v */ 1.7465238201e-2,
                        /* d·v² */ 3.5205649680e-2,
                        /* v³   */ -1.6138070441e-2
                    },
                    new double[] {
                        /* 1    */ 5.4780057238e+1,
                        /* d    */ -8.0553943910e+0,
                        /* v    */ 9.8969071974e+0,
                        /* d²   */ 9.3479980881e-1,
                        /* d·v  */ -2.3620060557e+0,
                        /* v²   */ 6.9825040437e-1,
                        /* d³   */ -4.9506580821e-1,
                        /* d²·v */ -6.4209468217e-1,
                        /* d·v² */ 9.0324521099e-1,
                        /* v³   */ -5.4437632941e-1
                    });

    /** Feed-shot model — used when the robot is in a feed zone. */
    private static final PolyModel FEED_MODEL =
            new PolyModel(
                    "Feed Model",
                    5.0, // distMin (m)
                    10.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    7.5, // dMean
                    1.5430334996, // dStd
                    0.0, // vMean
                    2.0, // vStd
                    new double[] {
                        /* 1    */ 1.2074547373e+1,
                        /* d    */ 1.1124598419e+0,
                        /* v    */ -9.2720217607e-1,
                        /* d²   */ -5.8674357317e-2,
                        /* d·v  */ -7.2912571960e-2,
                        /* v²   */ -7.0818070818e-2,
                        /* d³   */ 5.6161425197e-2,
                        /* d²·v */ -6.0497571810e-3,
                        /* d·v² */ 2.1986814335e-1,
                        /* v³   */ -1.5954415954e-1
                    },
                    new double[] {
                        /* 1    */ 5.7369141664e+1,
                        /* d    */ -4.3735130912e+0,
                        /* v    */ 1.0080892292e+1,
                        /* d²   */ 7.8858336234e-2,
                        /* d·v  */ -1.5120778737e+0,
                        /* v²   */ 3.9384615385e-1,
                        /* d³   */ 2.6249905834e-1,
                        /* d²·v */ 2.9750087017e-1,
                        /* d·v² */ 1.0186869775e+0,
                        /* v³   */ -1.2099829060e+0
                    });

    /**
     * Dashboard selector for which hub surface to shoot with, so the ceiling-limited model can be
     * picked when testing indoors without a redeploy. Feed shots always use {@link #FEED_MODEL} and
     * are unaffected.
     */
    private final LoggedDashboardChooser<PolyModel> hubModelChooser =
            new LoggedDashboardChooser<>("Hub Model Chooser");

    // An AdvantageKit dashboard input (same dashboard key as the 2026 SendableChooser): the shot
    // model changes every hood angle and RPM, so replay has to know which one was selected.
    private ShotCalculator() {
        hubModelChooser.addDefaultOption(NO_CEILING_HUB_MODEL.name(), NO_CEILING_HUB_MODEL);
        hubModelChooser.addOption(CEILING_3M_HUB_MODEL.name(), CEILING_3M_HUB_MODEL);
        Telemetry.logDash("ShotCalc/SetShot", selectedSetShot.label);
    }

    /**
     * The hub model selected on the dashboard, falling back to the no-ceiling model if nothing has
     * been selected yet.
     */
    private PolyModel selectedHubModel() {
        PolyModel selected = hubModelChooser.get();
        return selected != null ? selected : NO_CEILING_HUB_MODEL;
    }

    // =========================================================================
    // Set shots: fixed shots from known parking spots, for when the pose is gone
    // =========================================================================

    /**
     * FM's centre to its bumper face, metres: half the 0.84 m bumpered frame PathPlanner uses
     * ({@code settings.json}). CALIBRATE: measure FM's bumper-to-centre, both ways.
     */
    public static final double FM_HALF_LENGTH_METERS = 0.42;

    /**
     * The y of the tower's centreline, metres. CALIBRATE on the field. The offseason bot parked on
     * tag 31's y (147.469 in in the 2026 welded layout, 3.746 m) and found "squared up dead flat is
     * 5.3 deg off, because the tower's centreline follows tag 31 and the hub sits on the field
     * centreline". FM's own {@code Field.BlueTower.tag31Y} puts the tower on the field centreline
     * instead (4.035 m); the distance differs by under a centimetre, only the heading moves.
     */
    public static final double TOWER_CENTRE_Y_METERS = 147.469 * 0.0254;

    /**
     * The fixed shots, one per parking spot, from the offseason bot's set shots (463c465, fa1376b)
     * less its hub-face shot, which FM cannot make. Each is a place to park, blue alliance, the
     * robot centre and the heading that points the launcher (FM's back) at the hub centre; on red
     * the spot is the same one rotated about the field centre. The hood and flywheel are not stored
     * here: they are FM's selected hub model evaluated at the spot's range at a standstill ({@link
     * #setShotSolution}), so a set shot follows the shot map, and the operator's hood trim, without
     * anyone retyping numbers.
     *
     * <p>The driver aims, as on the offseason bot: park where the spot says, point the back of the
     * robot at the hub (the heading below), hold the chord. Nothing reads the pose, so the shot is
     * only as good as the parking. The model moves about 0.5 deg of hood and 35 RPM per 15 cm here,
     * so lining up by eye against the field element is good enough.
     *
     * <p>Hub centre: {@code Field.BlueHub} (the midpoint of tags 26 and 20, 4.626, 4.035 m).
     */
    public enum SetShot {
        /**
         * Intake (front bumper) against the tower's field-facing wall, on its centreline, back of
         * the robot to the hub: (1.525, 3.746) m, 3.11 m out. Squared up to the wall the launcher
         * points 5.3 deg right of the hub, so turn the robot about 5 deg counter-clockwise (from
         * above) off square.
         */
        TOWER("Tower", Field.BlueTower.frontFaceX + FM_HALF_LENGTH_METERS, TOWER_CENTRE_Y_METERS),
        /**
         * In the left trench lane (the driver's left: +y on blue) just clear of the trench on the
         * alliance side, intake to the wall, back to the hub. Lane centre is half the 50.34 in
         * opening from the wall; the trench is the hub's 47 in deep, centred on the hub's x.
         * (3.609, 7.430) m, 3.54 m out. CALIBRATE both: the offseason bot used the same spot, 3.53
         * m with its smaller frame.
         */
        LEFT_TRENCH(
                "LeftTrench",
                Field.BlueHub.centerX - Field.BlueTrench.depth / 2.0 - FM_HALF_LENGTH_METERS,
                Field.fieldWidth - Field.BlueTrench.openingWidth / 2.0),
        /** Mirror of {@link #LEFT_TRENCH} across the field's long centreline. */
        RIGHT_TRENCH(
                "RightTrench",
                Field.BlueHub.centerX - Field.BlueTrench.depth / 2.0 - FM_HALF_LENGTH_METERS,
                Field.BlueTrench.openingWidth / 2.0);

        /** Short name, logged to {@code ShotCalc/SetShot}. */
        public final String label;

        /** Where to park on the blue side, and the heading to hold there. */
        public final Pose2d bluePose;

        /** Robot centre (the launcher: {@code robotToLauncher} is zero) to the hub centre. */
        public final double distanceMeters;

        SetShot(String label, double blueX, double blueY) {
            this.label = label;
            Translation2d spot = new Translation2d(blueX, blueY);
            Translation2d toHub =
                    new Translation2d(Field.BlueHub.centerX, Field.BlueHub.centerY).minus(spot);
            this.bluePose = new Pose2d(spot, toHub.getAngle().plus(Rotation2d.k180deg));
            this.distanceMeters = toHub.getNorm();
        }

        /** The spot and heading for the alliance the robot is on (rotated about centre for red). */
        public Pose2d pose() {
            return FieldHelpers.flipIfRed(bluePose);
        }
    }

    private static SetShot selectedSetShot = SetShot.TOWER;

    /**
     * Picks which fixed shot the SET_SHOT super state runs. Called from the pilot binding before
     * the state is requested; the selection sticks until the next binding changes it. Main thread
     * only; the binding comes from logged Driver Station inputs, so replay makes the same pick.
     *
     * @param shot the parking spot
     */
    public static void selectSetShot(SetShot shot) {
        selectedSetShot = shot;
        Telemetry.logDash("ShotCalc/SetShot", shot.label);
    }

    /** The fixed shot currently selected, {@link SetShot#TOWER} until a binding picks one. */
    public static SetShot getSelectedSetShot() {
        return selectedSetShot;
    }

    /**
     * Hood angle and flywheel speed for a set shot: the hub model at the spot's range at a
     * standstill, exactly what {@link #getParameters()} commands standing still at that range (with
     * no velocity the virtual-target solver evaluates the model at the real distance), with the
     * same hood trim and 9 deg floor. Never reads the pose.
     *
     * @param model the hub model to evaluate
     * @param distanceMeters range to the hub centre
     * @return {@code {hoodDegrees, flywheelRPM}}
     */
    static double[] setShotSolution(PolyModel model, double distanceMeters) {
        double[] raw = evalPolyRaw(model, distanceMeters, 0.0);
        double hoodDegrees = Math.max(90 - raw[1] + HOOD_ANGLE_OFFSET, 9);
        double rpm = raw[0] * MPS_FACTOR * RPM_PER_MPS;
        return new double[] {hoodDegrees, rpm};
    }

    /** {@link #setShotSolution} for a spot on the no-ceiling (competition) hub model. */
    static double[] setShotSolution(SetShot shot) {
        return setShotSolution(NO_CEILING_HUB_MODEL, shot.distanceMeters);
    }

    /** Hood angle for the selected set shot, degrees, on the dashboard's hub model. */
    public static double getSetShotHoodDegrees() {
        return setShotSolution(getInstance().selectedHubModel(), selectedSetShot.distanceMeters)[0];
    }

    /** Flywheel speed for the selected set shot, RPM, on the dashboard's hub model. */
    public static double getSetShotFlywheelRPM() {
        return setShotSolution(getInstance().selectedHubModel(), selectedSetShot.distanceMeters)[1];
    }

    // =========================================================================
    // State — Velocity Derivative Filters
    // =========================================================================

    /** Robot loop period, for the ~100 ms filter windows below. */
    private static final double LOOP_PERIOD_SECS =
            frc.spectrumLib.framework.RobotLoop.periodSeconds();

    /**
     * Phase delay applied to the estimated robot pose before computing shot parameters,
     * compensating for sensor and network latency (seconds).
     */
    private static final double PHASE_DELAY_SECS = 0.03;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private double lastHoodAngle = Double.NaN;

    // =========================================================================
    // Main API
    // =========================================================================

    /**
     * Returns the current shooting parameters, computing them from the robot's live pose and
     * velocity if not already cached this loop.
     *
     * <p>Approach:
     *
     * <ol>
     *   <li>Apply a phase delay to the odometry pose to account for sensor latency.
     *   <li>Compute the launcher's field-relative velocity, including the tangential component from
     *       robot rotation about its centre.
     *   <li>Decompose that velocity into radial (toward target) and tangential (perpendicular)
     *       components.
     *   <li>Run the 1690 Orbit iterative virtual-target solver to determine the optimal exit speed,
     *       launch angle, and yaw correction for shoot-on-the-move.
     *   <li>Derive the drive angle, hood angle, and flywheel RPM from the result.
     * </ol>
     *
     * <p>Call {@link #clearShootingParameters()} at the start of each loop to allow re-computation
     * on the next call.
     *
     * @return the latest {@link ShootingParameters}
     */
    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        // ── Target selection ─────────────────────────────────────────────────
        boolean feed = Robot.getSuperStructure().isRobotInFeedZone();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();
        // Feed and hub shots use separately-fitted polynomial surfaces.
        PolyModel model = feed ? FEED_MODEL : selectedHubModel();

        // ── Phase-delayed pose estimate ──────────────────────────────────────
        Pose2d estimatedPose = Robot.getSwerve().getRobotPose();
        ChassisVelocities robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.plus(
                        new Twist2d(
                                        robotRelativeVelocity.vx * PHASE_DELAY_SECS,
                                        robotRelativeVelocity.vy * PHASE_DELAY_SECS,
                                        robotRelativeVelocity.omega * PHASE_DELAY_SECS)
                                .exp());

        // ── Launcher pose + static distance ──────────────────────────────────
        Pose2d launcherPose = estimatedPose.transformBy(robotToLauncher);
        Translation2d launcherToTarget = target.minus(launcherPose.getTranslation());
        double distanceNoLookahead = launcherToTarget.getNorm();

        // ── Field-relative launcher velocity (includes rotation arm) ─────────
        ChassisVelocities fieldVelocity =
                robotRelativeVelocity.toFieldRelative(estimatedPose.getRotation());
        double robotAngle = estimatedPose.getRotation().getRadians();
        double launcherVelocityX =
                fieldVelocity.vx
                        - fieldVelocity.omega
                                * (robotToLauncher.getX() * Math.sin(robotAngle)
                                        + robotToLauncher.getY() * Math.cos(robotAngle));
        double launcherVelocityY =
                fieldVelocity.vy
                        + fieldVelocity.omega
                                * (robotToLauncher.getX() * Math.cos(robotAngle)
                                        - robotToLauncher.getY() * Math.sin(robotAngle));

        // ── Decompose velocity into radial and tangential components ──────────
        // Unit vector from launcher toward target
        double ux = launcherToTarget.getX() / distanceNoLookahead;
        double uy = launcherToTarget.getY() / distanceNoLookahead;
        // Positive radialVelocity = closing on target
        double radialVelocity = launcherVelocityX * ux + launcherVelocityY * uy;
        // Tangential: perpendicular to the radial axis
        double tangentialVelocity = -launcherVelocityX * uy + launcherVelocityY * ux;

        // ── Polynomial + 1690 virtual-target solver ───────────────────────────
        // Returns: { exitSpeed_ms, launchAngle_deg, yawOffset_deg, virtualDist_m, tof_s }
        double[] poly =
                solveVirtualTarget(model, distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = poly[0];
        double rawHoodAngle = 90 - poly[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = poly[2];
        double lookaheadDist = poly[3];
        double tofFinal = poly[4];

        // ── Drive angle: static bearing + shoot-on-move yaw + user offset ────
        Rotation2d driveAngle =
                launcherToTarget
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(yawOffsetDeg))
                        .plus(Rotation2d.fromDegrees(DRIVE_ANGLE_OFFSET))
                        .plus(Rotation2d.k180deg);

        // ── Lookahead pose: estimated launcher position when the ball arrives ────
        // Useful for Field2d visualization and validating shoot-on-move compensation.
        Pose2d lookaheadPose =
                new Pose2d(
                        launcherPose
                                .getTranslation()
                                .plus(
                                        new Translation2d(
                                                launcherVelocityX * tofFinal,
                                                launcherVelocityY * tofFinal)),
                        driveAngle);

        // ── Drive angular velocity (rad/s, CCW positive) for heading feedforward ──
        double driveAngularVelocity =
                bearingRateRadPerSec(launcherToTarget, launcherVelocityX, launcherVelocityY);

        // ── Hood angle + velocity ─────────────────────────────────────────────
        // Compute velocity on the raw (un-offset) angle so HOOD_ANGLE_OFFSET (a
        // near-constant) does not bleed into the derivative.
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle = Math.max(rawHoodAngle + HOOD_ANGLE_OFFSET, 9);

        // ── Flywheel speed: exit speed (m/s) → RPM ───────────────────────────
        double flywheelSpeed = exitSpeedMs * RPM_PER_MPS;

        // ── Validity ──────────────────────────────────────────────────────────
        boolean isValid =
                distanceNoLookahead >= model.distMin() && distanceNoLookahead <= model.distMax();

        latestParameters =
                new ShootingParameters(
                        isValid,
                        driveAngle,
                        driveAngularVelocity,
                        hoodAngle,
                        hoodVelocity,
                        flywheelSpeed,
                        exitSpeedMs,
                        lookaheadDist,
                        distanceNoLookahead,
                        tofFinal,
                        radialVelocity,
                        tangentialVelocity,
                        feed,
                        model.name());

        Telemetry.log("ShotCalc/LookaheadPose", lookaheadPose);
        Telemetry.log("ShotCalc/DistanceMeters", lookaheadDist, "meters");
        Telemetry.log("ShotCalc/DistanceNoLookahead", distanceNoLookahead, "meters");
        Telemetry.log("ShotCalc/DriveAngleDeg", driveAngle.getDegrees(), "degrees");
        Telemetry.log("ShotCalc/YawOffsetDeg", yawOffsetDeg, "degrees");
        Telemetry.log("ShotCalc/HoodAngleDeg", hoodAngle, "degrees");
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", flywheelSpeed, "RPM");
        Telemetry.log("ShotCalc/ExitSpeedMs", exitSpeedMs, "m/s");
        Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
        Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
        Telemetry.log("ShotCalc/TimeOfFlight", tofFinal, "seconds");
        Telemetry.log("ShotCalc/FeedShot", feed);
        Telemetry.log("ShotCalc/HubPolyModel", model.name());
        Telemetry.log("ShotCalc/DriveAngleOffsetDegrees", DRIVE_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/Target", target);

        return latestParameters;
    }

    /**
     * How fast the bearing from the launcher to the goal turns while the launcher moves: minus the
     * tangential velocity over the distance (581's {@code AimParameterUtil}), in rad/s,
     * counter-clockwise positive, the convention of the heading it feeds forward. {@code
     * driveAngle} is that bearing plus a constant half turn plus the shoot-on-move yaw offset,
     * which barely moves at a steady velocity, so this is very nearly the rate it turns at.
     *
     * <p>Until 2026-09 this was a numerical derivative of {@code driveAngle} itself, in rotations
     * per second (2 pi too small; nothing read it yet). Fixed to rad/s and handed to the heading
     * request it made the aim oscillate in simulation (mean heading error 6-9 deg strafing at 1
     * m/s, against a steady 4-5 deg lag with no feedforward): differentiating {@code driveAngle}
     * also differentiates the yaw offset, which follows the measured velocity, which the rotation
     * itself disturbs. The analytic rate has no derivative in it to amplify that.
     *
     * @param launcherToTarget field-relative vector from the launcher to the goal, metres
     * @param launcherVx field-relative launcher velocity, x, m/s
     * @param launcherVy field-relative launcher velocity, y, m/s
     * @return rad/s, counter-clockwise positive; 0 when on top of the goal
     */
    static double bearingRateRadPerSec(
            Translation2d launcherToTarget, double launcherVx, double launcherVy) {
        double distance = launcherToTarget.getNorm();
        if (distance < 1e-6) {
            return 0;
        }
        double ux = launcherToTarget.getX() / distance;
        double uy = launcherToTarget.getY() / distance;
        // Same decomposition as getParameters(): positive tangential is the launcher moving to
        // the left of its line of sight, so the bearing to the goal turns clockwise.
        double tangential = -launcherVx * uy + launcherVy * ux;
        return -tangential / distance;
    }

    /**
     * Clears the cached parameters so they are recomputed on the next call to {@link
     * #getParameters()}.
     */
    public void clearShootingParameters() {
        latestParameters = null;
    }

    // =========================================================================
    // Private — Polynomial Solver
    // =========================================================================

    /**
     * 1690 Orbit iterative virtual-target solver.
     *
     * <p>Each pass evaluates the polynomial at the current virtual aim point, estimates
     * time-of-flight from horizontal kinematics, shifts the aim point by how far the launcher moves
     * during that flight, and repeats until TOF converges. Terminates in ≤ 5 iterations (typically
     * 2–3).
     *
     * @param model the polynomial model (hub or feed) to evaluate against
     * @param distance horizontal distance to goal centre (metres)
     * @param radialVelocity launcher velocity toward/away from goal (m/s); positive = closing on
     *     goal
     * @param tangentialVelocity launcher velocity perpendicular to goal line (m/s)
     * @return {@code double[]} with indices:
     *     <ul>
     *       <li>0 — exit speed (m/s), scaled by {@link #MPS_FACTOR}
     *       <li>1 — launch angle (degrees), raw polynomial value
     *       <li>2 — yaw offset (degrees); add to static bearing before firing
     *       <li>3 — converged virtual aim distance (metres)
     *       <li>4 — converged time of flight (seconds)
     *     </ul>
     */
    private static double[] solveVirtualTarget(
            PolyModel model, double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point — radial component (m)
        double vdz = 0.0; // virtual aim point — lateral component (m)
        double tof = 0.0;

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate polynomial at virtual point with rv = 0 (robot motion is
            // already encoded in the shifted aim point)
            double[] raw = evalPolyRaw(model, vDist, 0.0);
            double speed = raw[0] * MPS_FACTOR;
            double cosA = Math.cos(raw[1] * Math.PI / 180.0);
            double prevTof = tof;
            tof = vDist / Math.max(speed * cosA, 0.5); // guard against div-by-zero

            // Shift aim point: where the target will be relative to the launcher
            // when the ball arrives
            vdx = distance - radialVelocity * tof;
            vdz = -tangentialVelocity * tof;

            if (iter > 0 && Math.abs(tof - prevTof) < 0.002) break;
        }

        double virtualDist = Math.sqrt(vdx * vdx + vdz * vdz);
        double yawOffsetDeg =
                Math.atan2(-tangentialVelocity * tof, distance - radialVelocity * tof)
                        * (180.0 / Math.PI);

        double[] result = evalPolyRaw(model, virtualDist, 0.0);
        return new double[] {
            result[0] * MPS_FACTOR, // exitSpeed_ms
            result[1], // launchAngle_deg
            yawOffsetDeg, // yaw correction (degrees)
            virtualDist, // converged lookahead distance (m)
            tof // converged time of flight (s)
        };
    }

    /**
     * Evaluates the given polynomial surface at (distance, radialVel). Inputs are clamped to the
     * model's fitted data range. Returns raw polynomial output — callers are responsible for
     * applying {@link #MPS_FACTOR} to the exit speed and {@code HOOD_ANGLE_OFFSET} to the launch
     * angle.
     *
     * @param model the polynomial model (hub or feed) to evaluate
     * @param distance horizontal distance to the aim point (metres)
     * @param radialVel radial velocity (m/s)
     * @return double[] { exitSpeed_ms (raw, before MPS_FACTOR), launchAngle_deg }
     */
    private static double[] evalPolyRaw(PolyModel model, double distance, double radialVel) {
        double d_raw = Math.max(model.distMin(), Math.min(model.distMax(), distance));
        double v_raw = Math.max(model.rvMin(), Math.min(model.rvMax(), radialVel));
        double d = (d_raw - model.dMean()) / model.dStd();
        double v = (v_raw - model.vMean()) / model.vStd();

        double d2 = d * d;
        double v2 = v * v;
        double d3 = d2 * d;
        double v3 = v2 * v;

        double[] terms = {
            1.0, // 1
            d, // d
            v, // v
            d2, // d²
            d * v, // d·v
            v2, // v²
            d3, // d³
            d2 * v, // d²·v
            d * v2, // d·v²
            v3 // v³
        };

        double[] speedCoeffs = model.speedCoeffs();
        double[] angleCoeffs = model.angleCoeffs();
        double exitSpeed = 0.0, launchAngle = 0.0;
        for (int i = 0; i < terms.length; i++) {
            exitSpeed += speedCoeffs[i] * terms[i];
            launchAngle += angleCoeffs[i] * terms[i];
        }
        return new double[] {exitSpeed, launchAngle};
    }
}
