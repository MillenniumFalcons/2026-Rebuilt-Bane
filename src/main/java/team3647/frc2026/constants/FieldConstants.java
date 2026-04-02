package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static final Distance kFieldLength = Meters.of(16.541);
    public static final double kFieldLengthM = kFieldLength.in(Meters);
    public static final Distance kFieldWidth = Meters.of(8.0692);
    public static final double kFieldWidthM = kFieldWidth.in(Meters);

    public static final Pose2d blueHubPose = new Pose2d(4.624, 4.034, new Rotation2d());
    public static final Pose2d redHubPose =
            new Pose2d(kFieldLengthM - 4.624, kFieldWidthM - 4.034, new Rotation2d());

    public static final Pose2d blueFeedPoseRight =
            new Pose2d(2.225435972213745, 2.17506742477417, new Rotation2d());
    public static final Pose2d blueFeedPoseLeft =
            new Pose2d(2.0832128524780273, 6.238598823547363, new Rotation2d());
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
