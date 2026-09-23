package frc.rebuilt.targetFactories;

import frc.rebuilt.Field;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.math.interpolation.Interpolator;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.util.Units;

public class FeedTargetFactory {

    private static final Swerve swerve = Robot.getSwerve();

    static InterpolatingTreeMap<Double, Double> distanceOffsetMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        distanceOffsetMap.put(0.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static Translation2d generate() {
        boolean inFieldLeft = swerve.inFieldLeft().getAsBoolean();
        boolean inOpposingAllianceZone = swerve.inEnemyAllianceZone().getAsBoolean();
        Translation2d feedTarget;

        if (inOpposingAllianceZone) {
            if (inFieldLeft) {
                feedTarget = Field.isBlue() ? Field.deepFeedBlueLeft : Field.deepFeedRedRight;
            } else {
                feedTarget = Field.isBlue() ? Field.deepFeedBlueRight : Field.deepFeedRedLeft;
            }
        } else {
            if (inFieldLeft) {
                feedTarget = Field.isBlue() ? Field.normalFeedBlueLeft : Field.normalFeedRedRight;
            } else {
                feedTarget = Field.isBlue() ? Field.normalFeedBlueRight : Field.normalFeedRedLeft;
            }
        }

        double distance =
                new Translation2d(feedTarget.getX(), feedTarget.getY())
                        .getDistance(Robot.getSwerve().getRobotPose().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (Field.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        feedTarget =
                new Translation2d(
                        feedTarget.getX() + offSet.getX(), feedTarget.getY() + offSet.getY());
        return feedTarget;
    }
}
