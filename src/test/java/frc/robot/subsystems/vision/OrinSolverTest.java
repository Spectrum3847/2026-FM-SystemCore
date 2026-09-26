package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.localization.PoseObservation;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;

class OrinSolverTest {
    /** Tag 1 at (5, 4, 0.5) facing -x (180 deg), tag 2 beside it. */
    private static final AprilTagFieldLayout LAYOUT =
            new AprilTagFieldLayout(
                    List.of(
                            new AprilTag(1, new Pose3d(5, 4, 0.5, new Rotation3d(0, 0, Math.PI))),
                            new AprilTag(2, new Pose3d(5, 5, 0.5, new Rotation3d(0, 0, Math.PI)))),
                    16.5,
                    8.1);

    /** Camera at the robot origin, level, facing forward. */
    private static OrinSolver solver() {
        return new OrinSolver(Transform3d.kZero, LAYOUT, 1.0, 1280, 800);
    }

    /** Corners well inside a 1280x800 image. */
    private static final List<TargetCorner> CENTER_CORNERS =
            List.of(
                    new TargetCorner(600, 380),
                    new TargetCorner(680, 380),
                    new TargetCorner(680, 420),
                    new TargetCorner(600, 420));

    private static PhotonTrackedTarget target(
            int id, Transform3d best, Transform3d alt, double ambiguity, List<TargetCorner> c) {
        return new PhotonTrackedTarget(0, 0, 1, 0, id, -1, -1f, best, alt, ambiguity, c, c);
    }

    private static byte[] bytes(PhotonPipelineResult r) {
        Packet p = new Packet(64);
        PhotonPipelineResult.photonStruct.pack(p, r);
        return p.getWrittenDataCopy();
    }

    private static OrinCameraInputs inputsOf(PhotonPipelineResult... results) {
        OrinCameraInputs in = new OrinCameraInputs();
        in.results = new byte[results.length][];
        for (int i = 0; i < results.length; i++) {
            in.results[i] = bytes(results[i]);
        }
        return in;
    }

    private static PhotonPipelineResult result(
            long captureMicros,
            List<PhotonTrackedTarget> targets,
            Optional<MultiTargetPNPResult> multitag) {
        return new PhotonPipelineResult(
                new PhotonPipelineMetadata(captureMicros, captureMicros + 5000, 1, 0),
                targets,
                multitag);
    }

    /** Robot 2 m in front of tag 1, facing it: camera-to-tag is 2 m straight ahead, rotated 180. */
    private static final Transform3d TWO_METRES_AHEAD =
            new Transform3d(new Translation3d(2, 0, 0), new Rotation3d(0, 0, Math.PI));

    @Test
    void singleTagSolvesRobotPose() {
        var r =
                result(
                        1_000_000,
                        List.of(target(1, TWO_METRES_AHEAD, TWO_METRES_AHEAD, 0.1, CENTER_CORNERS)),
                        Optional.empty());
        List<PoseObservation> obs = solver().solve(inputsOf(r), Set.of(), null);
        assertEquals(1, obs.size());
        PoseObservation o = obs.get(0);
        assertEquals(1.0, o.timestampSeconds(), 1e-9);
        assertEquals(3.0, o.pose().getX(), 1e-6);
        assertEquals(4.0, o.pose().getY(), 1e-6);
        assertEquals(0.5, o.pose().getZ(), 1e-6);
        assertEquals(1, o.tagCount());
        assertEquals(2.0, o.avgTagDistanceMeters(), 1e-9);
        assertEquals(0.1, o.maxAmbiguity(), 1e-9);
        assertEquals(1.0, o.stdDevScale(), 1e-9);
    }

    @Test
    void singleTagPicksTheCandidateClosestToTheGyro() {
        // The alternate candidate has the robot turned 30 deg; the gyro says 30 deg.
        Transform3d alt =
                new Transform3d(
                        new Translation3d(2, 0, 0),
                        new Rotation3d(0, 0, Math.PI - Math.toRadians(30)));
        var r =
                result(
                        1_000_000,
                        List.of(target(1, TWO_METRES_AHEAD, alt, 0.3, CENTER_CORNERS)),
                        Optional.empty());
        var withGyro =
                solver().solve(inputsOf(r), Set.of(), t -> Rotation2d.fromDegrees(30)).get(0);
        assertEquals(30, withGyro.pose2d().getRotation().getDegrees(), 1e-6);
        var noGyro = solver().solve(inputsOf(r), Set.of(), null).get(0);
        assertEquals(0, noGyro.pose2d().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void excludedTagsAndGamePiecesAreNeverUsed() {
        var gamePiece =
                new PhotonTrackedTarget(
                        0,
                        0,
                        5,
                        0,
                        -1,
                        0,
                        0.9f,
                        Transform3d.kZero,
                        Transform3d.kZero,
                        0,
                        CENTER_CORNERS,
                        CENTER_CORNERS);
        var r =
                result(
                        1_000_000,
                        List.of(
                                gamePiece,
                                target(
                                        1,
                                        TWO_METRES_AHEAD,
                                        TWO_METRES_AHEAD,
                                        0.05,
                                        CENTER_CORNERS)),
                        Optional.empty());
        OrinSolver s = solver();
        assertTrue(s.solve(inputsOf(r), Set.of(1), null).isEmpty());
        assertEquals(1, s.excludedTargets);
        assertEquals(1, s.solve(inputsOf(r), Set.of(), null).size());
    }

    @Test
    void multiTagCountsOnlyTheTagsInTheSolve() {
        // Three targets visible; the Jetson's multi-tag solve used only tags 1 and 2.
        Transform3d far = new Transform3d(new Translation3d(6, 0, 0), Rotation3d.kZero);
        var targets =
                List.of(
                        target(1, TWO_METRES_AHEAD, TWO_METRES_AHEAD, 0.1, CENTER_CORNERS),
                        target(2, TWO_METRES_AHEAD, TWO_METRES_AHEAD, 0.1, CENTER_CORNERS),
                        target(9, far, far, 0.1, CENTER_CORNERS));
        Transform3d fieldToCamera =
                new Transform3d(new Translation3d(3, 4.5, 0.5), Rotation3d.kZero);
        var multitag =
                new MultiTargetPNPResult(
                        new PnpResult(fieldToCamera, 0.2), List.of((short) 1, (short) 2));
        var obs =
                solver().solve(
                                inputsOf(result(2_000_000, targets, Optional.of(multitag))),
                                Set.of(),
                                null);
        assertEquals(1, obs.size());
        PoseObservation o = obs.get(0);
        assertEquals(2, o.tagCount());
        assertEquals(2.0, o.avgTagDistanceMeters(), 1e-9); // not pulled up by tag 9 at 6 m
        assertEquals(3.0, o.pose().getX(), 1e-9);
        assertEquals(4.5, o.pose().getY(), 1e-9);
        assertTrue(Double.isNaN(o.maxAmbiguity()));
    }

    @Test
    void tagsAtTheImageEdgeAreTrustedLess() {
        List<TargetCorner> edge =
                List.of(
                        new TargetCorner(0, 380),
                        new TargetCorner(80, 380),
                        new TargetCorner(80, 420),
                        new TargetCorner(0, 420));
        var r =
                result(
                        1_000_000,
                        List.of(target(1, TWO_METRES_AHEAD, TWO_METRES_AHEAD, 0.1, edge)),
                        Optional.empty());
        var o = solver().solve(inputsOf(r), Set.of(), null).get(0);
        assertEquals(OrinSolver.EDGE_MAX_SCALE, o.stdDevScale(), 1e-9);
    }

    @Test
    void badBytesAreCountedAndDropped() {
        OrinCameraInputs in = new OrinCameraInputs();
        in.results = new byte[][] {new byte[] {1, 2, 3}};
        OrinSolver s = solver();
        assertTrue(s.solve(in, Set.of(), null).isEmpty());
        assertEquals(1, s.decodeFailures);
    }

    @Test
    void emptyResultsDecodeButGiveNoObservation() {
        var r = result(1_000_000, List.of(), Optional.empty());
        OrinSolver s = solver();
        assertTrue(s.solve(inputsOf(r), Set.of(), null).isEmpty());
        assertEquals(0, s.decodeFailures);
        assertEquals(1.0, s.lastTimestampSeconds, 1e-9);
    }

    @Test
    void excludedTagListParses() {
        assertEquals(Set.of(7, 12, 13), Jetson.parseTags(" 7, 12,13 ,x"));
        assertTrue(Jetson.parseTags("").isEmpty());
    }
}
