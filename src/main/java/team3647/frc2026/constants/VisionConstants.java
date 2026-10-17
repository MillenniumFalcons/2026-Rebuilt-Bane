package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final AprilTagFieldLayout kAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // ---- Legacy base std devs (kept for backwards compat with old system) ----
    public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

    // ---- Large variance used to downweight unreliable vision measurements ----
    public static final double kLargeVariance = 1e6;

    // ---- Standard deviation array indices (12-element array from Limelight) ----
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;
    public static final int kMegatag2XStdDevIndex = 6;
    public static final int kMegatag2YStdDevIndex = 7;
    public static final int kMegatag2YawStdDevIndex = 11;
    public static final int kExpectedStdDevArrayLength = 12;

    // ---- Validation thresholds (from 254) ----
    public static final double kDefaultAmbiguityThreshold = 0.19;
    public static final double kDefaultYawDiffThreshold = 5.0; // degrees
    public static final double kTagAreaThresholdForYawCheck = 2.0;
    public static final double kTagMinAreaForSingleTagMegatag = 0.34;
    public static final double kDefaultZThreshold = 0.2; // meters
    public static final double kDefaultNormThreshold = 1.0; // meters

    // ---- Pose-fusion guards (added to stop vision from snapping odometry at auto start) ----
    // Reject vision estimates this far (meters) from current odometry while enabled, so a noisy
    // single-tag MegaTag1 pose can't teleport the robot and stall the path follower.
    public static final double kMaxOdometryJumpMeters = 1.0;
    // If we reject this many frames in a row, assume odometry is the wrong one and accept vision
    // again so we can recover (never lock vision out permanently).
    public static final int kOdoJumpRejectRecoveryFrames = 50;
    // Ignore all vision fusion for this long at autonomous start while the pose/path initializes.
    public static final double kAutoStartVisionSuppressSeconds = 0.75;
    // Floor on the fused XY std dev so a single optimistic frame can't near-absolutely trust vision
    // (previously the acceptor pinned this at 0.05 m).
    public static final double kMinVisionXYStdDev = 0.1; // meters

    // ---- Camera specs ----
    public static final int kCameraImageWidth = 1280;
    public static final int kCameraImageHeight = 800;
    public static final double kCameraHorizontalFOVDegrees = 81.0;
    public static final double kCameraVerticalFOVDegrees = 55.0;

    // ---- 3D camera transforms ---- (these are all fake)
    //     public static final Transform3d midMount =
    //             new Transform3d(
    //                     Inches.of(6.75),
    //                     Inches.of(9),
    //                     Inches.of(15),
    //                     new Rotation3d(Degrees.of(0), Degrees.of(-19.653), Degrees.of(0)));

    //     public static final Transform3d leftMount =
    //             new Transform3d(
    //                     Inches.of(7.110),
    //                     Inches.of(-11.201),
    //                     Inches.of(19.995),
    //                     new Rotation3d(Degrees.of(180), Degrees.of(28.1), Degrees.of(31)));

    //     public static final Transform3d rightMount =
    //             new Transform3d(
    //                     Inches.of(6.828),
    //                     Inches.of(11.031),
    //                     Inches.of(20.612),
    //                     new Rotation3d(Degrees.of(0), Degrees.of(28.1), Degrees.of(-31)));

    // so basically i named these facing the intake but the front is actually the shooter
    // ---- Camera A (Left Camera) - 254-style flat config for VisionIOHardwareLimelight ----
    public static final String kLimelightATableName = "limelight-left";
    public static final double kCameraAPitchDegrees =
            12; // not sure if this is supposed to be 12 or -12
    public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
    public static final double kCameraAHeightOffGroundMeters = Inches.of(7.62).in(Meters);
    public static final double kRobotToCameraAForward = Inches.of(4.215).in(Meters);
    public static final double kRobotToCameraASide = Inches.of(13.623).in(Meters);
    public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(-90);
    public static final Transform2d kRobotToCameraA =
            new Transform2d(
                    new Translation2d(kRobotToCameraAForward, kRobotToCameraASide),
                    kCameraAYawOffset);

    // ---- Camera B (Right Camera) ----
    public static final String kLimelightBTableName = "limelight-right";
    public static final double kCameraBPitchDegrees = 12;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBHeightOffGroundMeters = Inches.of(7.62).in(Meters);
    public static final double kRobotToCameraBForward = Inches.of(4.215).in(Meters);
    public static final double kRobotToCameraBSide = Inches.of(-13.623).in(Meters);
    public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(90);
    public static final Transform2d kRobotToCameraB =
            new Transform2d(
                    new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide),
                    kCameraBYawOffset);

    // ---- Camera C (Mid/Shooter Camera) ----
    public static final String kLimelightCTableName = "limelight-mid";
    public static final double kCameraCPitchDegrees = 33.1;
    public static final double kCameraCPitchRads = Units.degreesToRadians(kCameraCPitchDegrees);
    public static final double kCameraCHeightOffGroundMeters = Inches.of(16.235).in(Meters);
    public static final double kRobotToCameraCForward = Inches.of(11.786).in(Meters);
    public static final double kRobotToCameraCSide = Inches.of(-0.116).in(Meters);
    public static final Rotation2d kCameraCYawOffset = Rotation2d.fromDegrees(0);
    public static final Transform2d kRobotToCameraC =
            new Transform2d(
                    new Translation2d(kRobotToCameraCForward, kRobotToCameraCSide),
                    kCameraCYawOffset);

    // ---- Camera names (old format, kept for legacy references) ----
    public static final String kShooterCam = "limelight-mid";
    public static final String kLeftCam = "limelight-left";
    public static final String kRightCam = "limelight-right";
}
