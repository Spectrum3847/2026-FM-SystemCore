package frc.robot.pilot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import frc.robot.Robot;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;
import org.wpilib.command2.button.Trigger;

/* A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
public class Pilot extends Gamepad {
    public final Trigger LB = leftBumper;
    public final Trigger RB = rightBumper;
    public final Trigger LT = leftTrigger;
    public final Trigger RT = rightTrigger;

    public final Trigger AButton = A;
    public final Trigger BButton = B;
    public final Trigger XButton = X;
    public final Trigger YButton = Y;

    public final Trigger startButton = start;
    public final Trigger selectButton = select;

    public final Trigger leftStickPress = leftStickClick;
    public final Trigger rightStickPress = rightStickClick;

    public final Trigger dPadUp = upDpad;
    public final Trigger dPadDown = downDpad;
    public final Trigger dPadLeft = leftDpad;
    public final Trigger dPadRight = rightDpad;

    /*
     * Fixed shots (ShotCalculator.SetShot), for when the pose is gone: LB + a face button, as on
     * the offseason bot, so they can be held while driving (left index on the bumper, right thumb
     * on the button, left thumb never leaves the drive stick). Teleop only. The offseason bot's
     * fourth, LB+Y from the hub face, is not here: FM cannot make that shot.
     *
     * X and A are also bound bare (track target, unjam). All of these come from one function of the
     * buttons held (FaceChord), so any press or release order lands in what the buttons now say:
     * LB pressed with X held goes to the left-trench shot, LB let go with X still held goes back to
     * tracking, everything let go goes to IDLE (anyFaceChord's falling edge). Bind only the rising
     * edges of the individual triggers: exactly one of them is true at a time. Bare A and X only
     * count after being held alone for 0.12 s (FaceChordFilter), so a button let go a cycle after
     * LB, or pressed a cycle before it, does nothing.
     */
    public final Trigger setShotTower_LB_A = faceChord(FaceChord.SET_SHOT_TOWER);
    public final Trigger setShotLeftTrench_LB_X = faceChord(FaceChord.SET_SHOT_LEFT_TRENCH);
    public final Trigger setShotRightTrench_LB_B = faceChord(FaceChord.SET_SHOT_RIGHT_TRENCH);
    public final Trigger trackTarget_X = faceChord(FaceChord.TRACK_TARGET);
    public final Trigger unjam_A = faceChord(FaceChord.UNJAM);

    /** Any of the above; its falling edge (every face button and chord let go) is IDLE. */
    public final Trigger anyFaceChord = new Trigger(() -> faceChord() != FaceChord.NONE);

    /** Bare A and X wait {@link FaceChordFilter#SETTLE_SECONDS} before they count. */
    private final FaceChordFilter faceChordFilter = new FaceChordFilter();

    /** What LB and the face buttons ask for right now, after the bare-button settle time. */
    public FaceChord faceChord() {
        return faceChordFilter.update(
                org.wpilib.system.Timer.getTimestamp(),
                FaceChord.of(
                        LB.getAsBoolean(),
                        AButton.getAsBoolean(),
                        XButton.getAsBoolean(),
                        BButton.getAsBoolean(),
                        teleop.getAsBoolean()));
    }

    private Trigger faceChord(FaceChord chord) {
        return new Trigger(() -> faceChord() == chord);
    }

    public static class PilotConfig extends Config {
        private double deadzone = 0.15;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(3.0);

            setRightStickDeadzone(deadzone);
            setRightStickExp(3.0);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalar(1);
        }
    }

    @SuppressWarnings("unused")
    private PilotConfig config;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;

        config.setLeftStickScalar(
                Robot.getConfig().swerve.getLinearSpeedAt12Volts().in(MetersPerSecond));
        config.setRightStickScalar(
                Robot.getConfig().swerve.getAngularSpeedAt12Volts().in(RadiansPerSecond));
        leftStickCurve.setScalar(config.getLeftStickScalar());
        rightStickCurve.setScalar(config.getRightStickScalar());

        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        return -1 * ccwPositive; // invert the value
    }

    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}
