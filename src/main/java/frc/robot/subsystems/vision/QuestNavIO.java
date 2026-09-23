package frc.robot.subsystems.vision;

import frc.spectrumLib.localization.PoseObservation;
import frc.spectrumLib.localization.PoseObservation.Kind;
import frc.spectrumLib.localization.PoseSourceIO;
import frc.spectrumLib.localization.PoseSourceInputs;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.RawPublisher;
import org.wpilib.networktables.RawSubscriber;
import org.wpilib.networktables.TimestampedRaw;
import org.wpilib.system.Timer;

/**
 * Meta Quest running QuestNav, over NetworkTables table {@code QuestNav}.
 *
 * <p>QuestNav reports the <em>headset's</em> pose in a field frame that the robot sets by sending a
 * pose-reset command ({@link #resetPose}). This IO converts it to a robot pose with the inverse of
 * {@link #robotToQuest}, and records the time and count of resets so the QuestNav gates (see {@link
 * Vision}) can reject frames from before the first reset or just after one.
 *
 * <p>Frame timestamps are the NetworkTables server time at which the frame arrived, as QuestNavLib
 * uses; the Quest app's own clock is kept in {@link PoseSourceInputs#health} for latency analysis.
 *
 * <p>Health values: {@code [appTimestamp, frameCount, batteryPercent, trackingLostCounter,
 * resetCount, lastResetTime]}.
 */
public class QuestNavIO implements PoseSourceIO, QuestNavControl {
    /** Indices into {@link PoseSourceInputs#health}. */
    public static final int H_APP_TIMESTAMP = 0,
            H_FRAME_COUNT = 1,
            H_BATTERY = 2,
            H_TRACKING_LOST = 3,
            H_RESET_COUNT = 4,
            H_LAST_RESET_TIME = 5;

    /** A Quest is disconnected if no frame has arrived in this long. QuestNavLib uses 120 ms. */
    private static final double CONNECTED_TIMEOUT_SECONDS = 0.12;

    private final Transform3d robotToQuest;
    private final RawSubscriber frames;
    private final RawSubscriber deviceData;
    private final RawPublisher requests;

    private int commandId = 0;
    private int resetCount = 0;
    private double lastResetTime = Double.NaN;
    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private QuestNavProtocol.Frame lastFrame = null;
    private QuestNavProtocol.DeviceData lastDevice = null;

    /**
     * @param robotToQuest headset mount, robot frame (x forward, y left, z up)
     */
    public QuestNavIO(Transform3d robotToQuest) {
        this.robotToQuest = robotToQuest;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("QuestNav");
        frames =
                table.getRawTopic("frameData")
                        .subscribe(
                                QuestNavProtocol.FRAME_DATA_TYPE,
                                new byte[0],
                                PubSubOption.periodic(0.01),
                                PubSubOption.SEND_ALL,
                                PubSubOption.pollStorage(20));
        deviceData =
                table.getRawTopic("deviceData")
                        .subscribe(QuestNavProtocol.DEVICE_DATA_TYPE, new byte[0]);
        requests = table.getRawTopic("request").publish(QuestNavProtocol.COMMAND_TYPE);
    }

    /**
     * Tells the Quest where the robot is, re-origining its field frame.
     *
     * @param robotPose the robot's pose on the field right now
     */
    @Override
    public void resetPose(Pose2d robotPose) {
        Pose3d questPose = new Pose3d(robotPose).transformBy(robotToQuest);
        requests.set(QuestNavProtocol.encodePoseReset(++commandId, questPose));
        resetCount++;
        lastResetTime = Timer.getTimestamp();
    }

    @Override
    public void updateInputs(PoseSourceInputs inputs) {
        TimestampedRaw[] queue = frames.readQueue();
        int n = 0;
        PoseObservation[] obs = new PoseObservation[queue.length];
        for (TimestampedRaw raw : queue) {
            QuestNavProtocol.Frame frame = QuestNavProtocol.decodeFrame(raw.value);
            if (frame == null) {
                continue;
            }
            lastFrame = frame;
            double t = raw.serverTime / 1e6;
            lastFrameTime = raw.timestamp / 1e6;
            Pose3d robot = frame.pose().transformBy(robotToQuest.inverse());
            obs[n++] =
                    new PoseObservation(
                            "",
                            Kind.QUESTNAV,
                            t,
                            robot,
                            -1,
                            Double.NaN,
                            Double.NaN,
                            Double.NaN,
                            frame.tracking());
        }
        byte[] device = deviceData.get();
        if (device.length > 0) {
            lastDevice = QuestNavProtocol.decodeDeviceData(device);
        }

        inputs.connected = Timer.getTimestamp() - lastFrameTime < CONNECTED_TIMEOUT_SECONDS;
        inputs.resize(n);
        for (int i = 0; i < n; i++) {
            inputs.set(i, obs[i]);
        }
        inputs.tagIds = new int[0];
        inputs.health =
                new double[] {
                    lastFrame == null ? Double.NaN : lastFrame.appTimestamp(),
                    lastFrame == null ? -1 : lastFrame.frameCount(),
                    lastDevice == null ? -1 : lastDevice.batteryPercent(),
                    lastDevice == null ? -1 : lastDevice.trackingLostCounter(),
                    resetCount,
                    lastResetTime
                };
    }
}
