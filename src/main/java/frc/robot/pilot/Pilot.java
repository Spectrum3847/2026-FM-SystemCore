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
     * X and A are also bound bare (track target, unjam), so those carry "not LB" or LB + X would
     * fire both. Release order matters at the margin: let go of LB first with the face button still
     * down and the bare binding fires for the rest of the press. Let go of the face button first,
     * or both together.
     */
    public final Trigger setShotTower_LB_A = LB.and(AButton).and(teleop);
    public final Trigger setShotLeftTrench_LB_X = LB.and(XButton).and(teleop);
    public final Trigger setShotRightTrench_LB_B = LB.and(BButton).and(teleop);
    public final Trigger anySetShot =
            setShotTower_LB_A.or(setShotLeftTrench_LB_X).or(setShotRightTrench_LB_B);

    /* Bare face buttons, gated so the chords above own an LB-held press */
    public final Trigger trackTarget_X = XButton.and(LB.negate());
    public final Trigger unjam_A = AButton.and(LB.negate());

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
