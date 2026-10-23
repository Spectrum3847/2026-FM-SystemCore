package frc.robot;

import frc.robot.subsystems.hood.Hood.HoodConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.subsystems.vision.CameraParams;
import java.util.List;

/** Every class that declares params. Add a class here when it declares its first param. */
public final class RobotParams {
    private RobotParams() {}

    static final List<Class<?>> HOLDERS =
            List.of(CameraParams.class, HoodConfig.class, IntakeExtensionConfig.class);

    /**
     * Initializes every holder, which declares its params. Without this a param is declared the
     * first time its class is used, which may be mid-match.
     */
    public static void load() {
        for (Class<?> holder : HOLDERS) {
            try {
                Class.forName(holder.getName(), true, holder.getClassLoader());
            } catch (ClassNotFoundException e) {
                throw new IllegalStateException(e);
            }
        }
    }
}
