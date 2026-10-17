package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class ShotCalculatorTest {
    private static final double DT = 0.001;

    /** Finite difference of the drive angle (bearing to the hub plus a half turn), rad/s. */
    private static double numericDriveAngleRate(
            Translation2d hub, Translation2d launcher, double vx, double vy) {
        Translation2d later = launcher.plus(new Translation2d(vx * DT, vy * DT));
        Rotation2d before = hub.minus(launcher).getAngle().plus(Rotation2d.k180deg);
        Rotation2d after = hub.minus(later).getAngle().plus(Rotation2d.k180deg);
        return after.minus(before).getRadians() / DT;
    }

    /**
     * The sign the drivetrain needs. Robot at the origin strafing +y (left) at 1 m/s with the hub 3
     * m ahead on +x: the hub slides round to the robot's right, so the robot must turn clockwise, a
     * negative rate, of 1/3 rad/s.
     */
    @Test
    void strafingLeftPastTheHubTurnsClockwise() {
        Translation2d hub = new Translation2d(3, 0);
        double rate = ShotCalculator.bearingRateRadPerSec(hub, 0, 1.0);
        assertTrue(rate < 0, "clockwise is negative");
        assertEquals(-1.0 / 3.0, rate, 1e-9);
    }

    @Test
    void matchesTheDriveAngleItFeedsForwardFromAnyDirection() {
        Translation2d hub = new Translation2d(4.6, 4.0);
        double[][] cases = {
            {2.1, 4.0, 0.0, 1.0}, // in front of the hub, strafing; drive angle on the +/-180 wrap
            {2.1, 4.0, 1.0, 0.0}, // driving straight at it: no turn
            {6.0, 1.0, -0.7, 0.9}, // behind and to the right, diagonal
            {3.0, 7.0, 1.2, -0.4}, // off to the left, closing
        };
        for (double[] c : cases) {
            Translation2d launcher = new Translation2d(c[0], c[1]);
            double analytic = ShotCalculator.bearingRateRadPerSec(hub.minus(launcher), c[2], c[3]);
            double numeric = numericDriveAngleRate(hub, launcher, c[2], c[3]);
            assertEquals(numeric, analytic, 1e-3, () -> java.util.Arrays.toString(c));
        }
    }

    @Test
    void standingStillOrOnTheGoalIsNoTurn() {
        assertEquals(0, ShotCalculator.bearingRateRadPerSec(new Translation2d(3, 1), 0, 0), 0);
        assertEquals(0, ShotCalculator.bearingRateRadPerSec(Translation2d.kZero, 1, 1), 0);
    }
}
