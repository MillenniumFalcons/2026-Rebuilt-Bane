package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static final Distance kFieldLength = Meters.of(16.541);
    public static final double kFieldLengthM = kFieldLength.in(Meters);
    public static final Distance kFieldWidth = Meters.of(8.0692);
    public static final double kFieldWidthM = kFieldWidth.in(Meters);

    public static final double kBlueTrenchX = 4.8;
    public static final double kRedTrenchX = kFieldLengthM - kBlueTrenchX;

    // Tower uprights ("poles"). When auto-aligning to shoot up close, the robot orbits the
    // nearer of these two uprights instead of spinning about its own center. Upright A is the
    // lower-y pole, upright B the higher-y pole.
    public static final Translation2d blueTowerPoleA = new Translation2d(1.02, 3.317);
    public static final Translation2d blueTowerPoleB = new Translation2d(1.02, 4.174);
    public static final Translation2d redTowerPoleA = new Translation2d(15.52, 3.895);
    public static final Translation2d redTowerPoleB = new Translation2d(15.52, 4.752);

    public static final Pose2d blueHubPose = new Pose2d(4.624, 4.034, new Rotation2d());
    public static final Pose2d redHubPose =
            new Pose2d(kFieldLengthM - 4.624, kFieldWidthM - 4.034, new Rotation2d());

    // Feed targets: aim 2 m in from the near field corners (along the diagonal into the field).
    // Right = low-y corner (0, 0), left = high-y corner (0, width). Alliance-flipped in
    // RobotTracker. kFeedCornerOffset is the straight-line distance from the corner.
    public static final double kFeedCornerOffset = 2.0;
    private static final double kFeedCornerOffsetXY = kFeedCornerOffset / Math.sqrt(2.0);
    public static final Pose2d blueFeedPoseRight =
            new Pose2d(kFeedCornerOffsetXY, kFeedCornerOffsetXY, new Rotation2d());
    public static final Pose2d blueFeedPoseLeft =
            new Pose2d(kFeedCornerOffsetXY, kFieldWidthM - kFeedCornerOffsetXY, new Rotation2d());
    public static final Pose2d feedPoseRight =
            new Pose2d(
                    kFieldLengthM - 2.225435972213745,
                    kFieldWidthM - 2.17506742477417,
                    new Rotation2d());
    public static final Pose2d feedPoseLeft =
            new Pose2d(
                    kFieldLengthM - 2.0832128524780273,
                    kFieldWidthM - 6.238598823547363,
                    new Rotation2d());
}
