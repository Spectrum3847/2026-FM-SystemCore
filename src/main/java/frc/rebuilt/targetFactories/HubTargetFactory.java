package frc.rebuilt.targetFactories;

import frc.rebuilt.Field;
import frc.robot.Robot;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.math.interpolation.Interpolator;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.util.Units;

public class HubTargetFactory {

    static InterpolatingTreeMap<Double, Double> heightMap =
            new InterpolatingTreeMap<Double, Double>(
                    InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        heightMap.put(0.0, 0.0);
    }

    static InterpolatingTreeMap<Double, Double> distanceOffsetMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        distanceOffsetMap.put(0.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static Translation3d generate() {
        Translation3d hubPose = Field.isRed() ? Field.getRedHubCenter() : Field.getBlueHubCenter();

        double distance =
                new Translation2d(hubPose.getX(), hubPose.getY())
                        .getDistance(Robot.getSwerve().getRobotPose().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (Field.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        hubPose =
                new Translation3d(
                        hubPose.getX() + offSet.getX(),
                        hubPose.getY() + offSet.getY(),
                        hubPose.getZ() + heightMap.get(distance));
        return hubPose;
    }
}
