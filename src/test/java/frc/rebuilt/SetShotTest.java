package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.rebuilt.ShotCalculator.SetShot;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class SetShotTest {
    private static final double IN = 0.0254;
    private static final Translation2d HUB = new Translation2d((158.61 + 23.5) * IN, 158.85 * IN);

    /**
     * Spots recomputed here from the field drawings, so a changed constant has to be argued for:
     * FM's bumper face 0.42 m from its centre, the tower face 43.51 in out on tag 31's y, the
     * trench 47 in deep centred on the hub's x and its lane half of 50.34 in off the wall.
     */
    @Test
    void spotsAreWhereTheFieldSaysAndDistancesMatch() {
        double half = 0.42;
        Translation2d tower = new Translation2d(43.51 * IN + half, 147.469 * IN);
        Translation2d left = new Translation2d(HUB.getX() - 23.5 * IN - half, 8.0696 - 25.17 * IN);
        Translation2d right = new Translation2d(HUB.getX() - 23.5 * IN - half, 25.17 * IN);
        check(SetShot.TOWER, tower, 3.114);
        check(SetShot.LEFT_TRENCH, left, 3.544);
        check(SetShot.RIGHT_TRENCH, right, 3.544);
    }

    private static void check(SetShot shot, Translation2d spot, double metres) {
        assertEquals(spot.getX(), shot.bluePose.getX(), 0.005, shot.label + " x");
        assertEquals(spot.getY(), shot.bluePose.getY(), 0.005, shot.label + " y");
        assertEquals(metres, shot.distanceMeters, 0.005, shot.label + " range");
        assertEquals(HUB.getDistance(spot), shot.distanceMeters, 0.005, shot.label);
    }

    /** FM launches out of its back, so the heading faces directly away from the hub. */
    @Test
    void headingPointsTheBackAtTheHub() {
        for (SetShot shot : SetShot.values()) {
            Rotation2d toHub = HUB.minus(shot.bluePose.getTranslation()).getAngle();
            Rotation2d back = shot.bluePose.getRotation().plus(Rotation2d.k180deg);
            assertEquals(0, back.minus(toHub).getDegrees(), 0.2, shot.label);
        }
        // The tower is 5.3 deg off square (the launcher squared to the wall points along +x).
        assertEquals(185.3, SetShot.TOWER.bluePose.getRotation().getDegrees() + 360, 0.1);
        assertEquals(106.7, SetShot.LEFT_TRENCH.bluePose.getRotation().getDegrees(), 0.1);
        assertEquals(-106.7, SetShot.RIGHT_TRENCH.bluePose.getRotation().getDegrees(), 0.1);
    }

    /**
     * FM's hub model at each range, standing still: inside the hood's travel and the model's fitted
     * range, and the farther trench shot needs more flywheel and a flatter shot (more hood) than
     * the tower.
     */
    @Test
    void solutionComesFromFmsHubModelAtTheSpotsRange() {
        double[] tower = ShotCalculator.setShotSolution(SetShot.TOWER);
        double[] trench = ShotCalculator.setShotSolution(SetShot.LEFT_TRENCH);
        System.out.printf(
                "set shots (no-ceiling model, hood trim %.1f): tower %.1f deg %.0f RPM, trench %.1f"
                        + " deg %.0f RPM%n",
                ShotCalculator.HOOD_ANGLE_OFFSET, tower[0], tower[1], trench[0], trench[1]);
        for (double[] s : new double[][] {tower, trench}) {
            assertTrue(s[0] >= 9 && s[0] <= 0.137 * 360, "hood inside its travel: " + s[0]);
            assertTrue(s[1] > 1500 && s[1] < 4500, "flywheel plausible: " + s[1]);
        }
        assertTrue(trench[1] > tower[1]);
        assertTrue(trench[0] > tower[0]);
        assertEquals(
                ShotCalculator.setShotSolution(SetShot.LEFT_TRENCH)[1],
                ShotCalculator.setShotSolution(SetShot.RIGHT_TRENCH)[1],
                1e-9);
        for (SetShot shot : SetShot.values()) {
            assertTrue(shot.distanceMeters > 1.5 && shot.distanceMeters < 8.0, shot.label);
        }
    }

    @Test
    void theOperatorsHoodTrimMovesASetShotToo() {
        double before = ShotCalculator.setShotSolution(SetShot.TOWER)[0];
        double trim = ShotCalculator.HOOD_ANGLE_OFFSET;
        try {
            ShotCalculator.HOOD_ANGLE_OFFSET = trim + 1.0;
            assertEquals(before + 1.0, ShotCalculator.setShotSolution(SetShot.TOWER)[0], 1e-9);
        } finally {
            ShotCalculator.HOOD_ANGLE_OFFSET = trim;
        }
    }
}
