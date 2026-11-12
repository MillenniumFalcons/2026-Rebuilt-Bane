package team3647.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.VisionConstants;

/**
 * Vision subsystem that processes AprilTag detections and provides robot pose estimates. Supports
 * three Limelight cameras (A/B/C) using MegaTag1 only.
 *
 * <p>Based on Team 254's 2025 vision architecture, with per-camera measurement submission (no
 * pairwise fusion), distance-scaled std devs, per-camera timestamp gating, and a gyro-fused
 * single-tag fallback.
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionState state;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    private boolean useVision = true;

    // Vision fusion is ignored until this FPGA timestamp (set at auto start). Prevents a noisy
    // single-tag estimate from snapping odometry while the pose/path follower is initializing.
    private double visionSuppressedUntil = 0.0;

    // Per-camera last-used MegaTag timestamp. A slower camera must only be gated against its own
    // most-recently-accepted frame, never against a faster camera's newer fused timestamp (which is
    // what the old shared state.lastUsedMegatagTimestamp() did, dropping genuinely new frames).
    private final Map<String, Double> lastUsedTimestampByCamera = new HashMap<>();

    public VisionSubsystem(VisionIO io, VisionState state) {
        this.io = io;
        this.state = state;
    }

    /**
     * Ignore all vision fusion for {@code seconds} from now. Call at autonomous start so a jittery
     * single-tag MegaTag1 pose can't teleport odometry while the path follower is spinning up.
     *
     * <p>NOTE: with disabled multi-tag pose seeding (see {@link #processMegatagPoseEstimate}) the
     * odometry heading/position should already be correct before auto starts, so this suppression
     * window should be unnecessary. Kept (and call sites left in place) as a safety net until the
     * seeding behavior is validated on the field.
     */
    public void suppressVisionForSeconds(double seconds) {
        visionSuppressedUntil = Timer.getFPGATimestamp() + seconds;
    }

    /** Small holder pairing an accepted estimate with its source camera label. */
    private static final class AcceptedEstimate {
        final String label;
        final VisionFieldPoseEstimate estimate;

        AcceptedEstimate(String label, VisionFieldPoseEstimate estimate) {
            this.label = label;
            this.estimate = estimate;
        }
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.readInputs(inputs);

        // VisionIOInputs is a plain POJO (not @AutoLog / LoggableInputs), so Logger.processInputs()
        // cannot be called on it and raw inputs are not available for AdvantageKit replay. All of
        // the derived, decision-relevant values below are logged via recordOutput instead. If full
        // replay is later needed, convert VisionIOInputs to a LoggableInputs and add
        // Logger.processInputs("Vision", inputs) here.

        logCameraInputs("Vision/CameraA", inputs.cameraA);
        logCameraInputs("Vision/CameraB", inputs.cameraB);
        logCameraInputs("Vision/CameraC", inputs.cameraC);

        boolean suppressed = Timer.getFPGATimestamp() < visionSuppressedUntil;
        Logger.recordOutput("Vision/suppressed", suppressed);

        if (!useVision || suppressed) {
            Logger.recordOutput("Vision/usingVision", false);
            Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
            Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
            return;
        }

        Logger.recordOutput("Vision/usingVision", true);

        // Collect each camera's accepted estimate independently (no pairwise fusion).
        List<AcceptedEstimate> accepted = new ArrayList<>();
        processCamera(inputs.cameraA, "CameraA", VisionConstants.kRobotToCameraA)
                .ifPresent(e -> accepted.add(new AcceptedEstimate("CameraA", e)));
        processCamera(inputs.cameraB, "CameraB", VisionConstants.kRobotToCameraB)
                .ifPresent(e -> accepted.add(new AcceptedEstimate("CameraB", e)));
        processCamera(inputs.cameraC, "CameraC", VisionConstants.kRobotToCameraC)
                .ifPresent(e -> accepted.add(new AcceptedEstimate("CameraC", e)));

        // Over-weighting guard: WPILib's pose estimator applies a fixed Kalman gain with no
        // covariance update, so N agreeing cameras in one loop pull N times as hard. Inflate each
        // XY std dev by sqrt(N) when more than one is accepted.
        int n = accepted.size();
        double countScale =
                (VisionConstants.kScaleStdDevsByCameraCount && n > 1) ? Math.sqrt(n) : 1.0;
        Logger.recordOutput("Vision/acceptedCount", n);
        Logger.recordOutput("Vision/countScale", countScale);

        // Ordering: the pose estimator discards any stored vision update newer than an incoming
        // measurement's timestamp (PoseEstimator.addVisionMeasurement step 8:
        // m_visionUpdates.tailMap(timestamp, false).entrySet().clear()). Submitting a newer frame
        // first and an older frame second would therefore throw away the newer frame. Submit in
        // ascending timestamp order so every measurement is retained.
        accepted.sort(Comparator.comparingDouble(a -> a.estimate.getTimestampSeconds()));

        for (AcceptedEstimate a : accepted) {
            Matrix<N3, N1> std = a.estimate.getVisionMeasurementStdDevs();
            double sx = sanitizeXyStdDev(std.get(0, 0) * countScale);
            double sy = sanitizeXyStdDev(std.get(1, 0) * countScale);
            std.set(0, 0, sx);
            std.set(1, 0, sy);
            // Never let a 0/NaN/inf rotation std dev reach the estimator either.
            double sr = std.get(2, 0);
            if (!Double.isFinite(sr) || sr <= 0.0) {
                std.set(2, 0, VisionConstants.kLargeRotationStdDev);
            }

            state.updateMegatagEstimate(a.estimate);
            lastUsedTimestampByCamera.put(a.label, a.estimate.getTimestampSeconds());

            Logger.recordOutput(
                    "Vision/" + a.label + "/SubmittedPose", a.estimate.getVisionRobotPoseMeters());
            Logger.recordOutput("Vision/" + a.label + "/SubmittedXYStd", sx);
        }

        Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
        Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
    }

    private void logCameraInputs(String prefix, VisionIO.VisionIOInputs.CameraInputs cam) {
        Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
        Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);

        if (DriverStation.isDisabled()) {
            SmartDashboard.putBoolean(prefix + "/SeesTarget", cam.seesTarget);
            SmartDashboard.putNumber(prefix + "/MegatagCount", cam.megatagCount);
        }

        if (cam.pose3d != null) {
            Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
        }

        if (cam.megatagPoseEstimate != null) {
            Logger.recordOutput(
                    prefix + "/MegatagPoseEstimate", cam.megatagPoseEstimate.fieldToRobot());
            Logger.recordOutput(prefix + "/Quality", cam.megatagPoseEstimate.quality());
            Logger.recordOutput(prefix + "/AvgTagArea", cam.megatagPoseEstimate.avgTagArea());
            Logger.recordOutput(prefix + "/AvgTagDist", cam.megatagPoseEstimate.avgTagDist());
        }

        if (cam.fiducialObservations != null) {
            Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialObservations.length);
        }

        // Raw Limelight-reported std devs (12-element MT1/MT2 array). Also log the MT1 XY/yaw
        // entries individually for quick inspection.
        if (cam.standardDeviations != null) {
            Logger.recordOutput(prefix + "/LimelightStdDevs", cam.standardDeviations);
            if (cam.standardDeviations.length >= VisionConstants.kExpectedStdDevArrayLength) {
                Logger.recordOutput(
                        prefix + "/LimelightMT1XStd",
                        cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex]);
                Logger.recordOutput(
                        prefix + "/LimelightMT1YStd",
                        cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex]);
                Logger.recordOutput(
                        prefix + "/LimelightMT1YawStd",
                        cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex]);
            }
        }
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs cam, String label, Transform2d robotToCamera) {

        String logPrefix = "Vision/" + label;

        if (!cam.seesTarget || cam.megatagPoseEstimate == null) {
            Logger.recordOutput(logPrefix + "/AcceptedSource", "rejected");
            Logger.recordOutput(logPrefix + "/RejectionReason", "noTarget");
            return Optional.empty();
        }

        MegatagPoseEstimate poseEstimate = cam.megatagPoseEstimate;

        // Per-camera staleness gate: only reject frames this camera has already consumed.
        double lastUsed = lastUsedTimestampByCamera.getOrDefault(label, 0.0);
        if (poseEstimate.timestampSeconds() <= lastUsed) {
            Logger.recordOutput(logPrefix + "/AcceptedSource", "rejected");
            Logger.recordOutput(logPrefix + "/RejectionReason", "stale");
            return Optional.empty();
        }

        Optional<VisionFieldPoseEstimate> mtEstimate =
                processMegatagPoseEstimate(poseEstimate, cam, logPrefix);
        Optional<VisionFieldPoseEstimate> gyroEstimate = fuseWithGyro(poseEstimate, cam, logPrefix);

        mtEstimate.ifPresent(
                est ->
                        Logger.recordOutput(
                                logPrefix + "/AcceptedMegatagEstimate",
                                est.getVisionRobotPoseMeters()));
        gyroEstimate.ifPresent(
                est ->
                        Logger.recordOutput(
                                logPrefix + "/FuseWithGyroEstimate",
                                est.getVisionRobotPoseMeters()));

        // Prefer MegaTag when it passed; otherwise fall back to the gyro-fused single-tag estimate.
        Optional<VisionFieldPoseEstimate> estimate;
        if (mtEstimate.isPresent()) {
            estimate = mtEstimate;
            Logger.recordOutput(logPrefix + "/AcceptedSource", "MT1");
            Logger.recordOutput(logPrefix + "/AcceptMegatag", true);
            Logger.recordOutput(logPrefix + "/AcceptGyro", false);
        } else if (gyroEstimate.isPresent()) {
            estimate = gyroEstimate;
            Logger.recordOutput(logPrefix + "/AcceptedSource", "gyro-fallback");
            Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
            Logger.recordOutput(logPrefix + "/AcceptGyro", true);
        } else {
            estimate = Optional.empty();
            Logger.recordOutput(logPrefix + "/AcceptedSource", "rejected");
            Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
            Logger.recordOutput(logPrefix + "/AcceptGyro", false);
        }

        estimate.ifPresent(
                est ->
                        Logger.recordOutput(
                                logPrefix + "/AcceptedXYStd",
                                est.getVisionMeasurementStdDevs().get(0, 0)));
        Logger.recordOutput(logPrefix + "/AvgTagDistUsed", poseEstimate.avgTagDist());

        return estimate;
    }

    /**
     * When exactly 1 tag is visible, fuse the vision XY with the gyro heading. This gives a
     * position fix without trusting the noisy single-tag yaw estimate. Used when the direct
     * single-tag MegaTag estimate is rejected (ambiguity, area, distance, yaw diff).
     */
    private Optional<VisionFieldPoseEstimate> fuseWithGyro(
            MegatagPoseEstimate poseEstimate,
            VisionIO.VisionIOInputs.CameraInputs cam,
            String logPrefix) {

        // Only runs for exactly one tag. (Was dead code: the old guard `length > 0` returned empty
        // every time, and fiducialIds()[0] would also have thrown on an empty array.)
        if (poseEstimate.fiducialIds().length != 1) {
            return Optional.empty();
        }

        // Ambiguity still gates the fallback: a single highly-ambiguous tag is not trustworthy.
        for (var fiducial : cam.fiducialObservations) {
            if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
                return Optional.empty();
            }
        }

        // Reject if the robot is yawing rapidly (time-sync unreliable)
        final double kHighYawLookbackS = 0.3;
        final double kHighYawVelocityRadS = 5.0;

        if (state.getMaxAbsDriveYawAngularVelocityInRange(
                                poseEstimate.timestampSeconds() - kHighYawLookbackS,
                                poseEstimate.timestampSeconds())
                        .orElse(Double.POSITIVE_INFINITY)
                > kHighYawVelocityRadS) {
            return Optional.empty();
        }

        var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
        if (priorPose.isEmpty()) {
            return Optional.empty();
        }

        var maybeFieldToTag =
                VisionConstants.kAprilTagLayout.getTagPose(poseEstimate.fiducialIds()[0]);
        if (maybeFieldToTag.isEmpty()) {
            return Optional.empty();
        }

        Pose2d fieldToTag =
                new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);

        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());

        Pose2d posteriorPose =
                new Pose2d(
                        fieldToTag
                                .getTranslation()
                                .minus(
                                        robotToTag
                                                .getTranslation()
                                                .rotateBy(priorPose.get().getRotation())),
                        priorPose.get().getRotation());

        // Distance-scaled XY std devs, same model as the direct MegaTag path (single tag here).
        double xyStd = distanceScaledXyStdDev(poseEstimate.avgTagDist(), 1);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        posteriorPose,
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeRotationStdDev),
                        poseEstimate.fiducialIds().length));
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            MegatagPoseEstimate poseEstimate,
            VisionIO.VisionIOInputs.CameraInputs cam,
            String logPrefix) {

        boolean multiTag = poseEstimate.fiducialIds().length >= 2;

        // Yaw-diff gate: reject frames whose heading disagrees with odometry by too much. Bypassed
        // while disabled (odometry heading may still be wrong, and vision must be able to fix it)
        // and for multi-tag estimates (their yaw is reliable). Kept only for single-tag while
        // enabled, where a large yaw diff usually means a bad/ambiguous single-tag fix.
        boolean bypassYawGate = DriverStation.isDisabled() || multiTag;
        if (!bypassYawGate) {
            var gyroPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
            if (gyroPose.isPresent()) {
                double yawDiffDeg =
                        Math.abs(
                                poseEstimate
                                        .fieldToRobot()
                                        .getRotation()
                                        .minus(gyroPose.get().getRotation())
                                        .getDegrees());
                if (yawDiffDeg > VisionConstants.kMaxVisionGyroYawDiffDegrees) {
                    Logger.recordOutput(
                            logPrefix + "/RejectedGyroYawDiff", poseEstimate.fieldToRobot());
                    Logger.recordOutput(logPrefix + "/RejectionReason", "yawDiff");
                    return Optional.empty();
                }
            }
        }

        // Single-tag extra checks
        if (!multiTag) {
            for (var fiducial : cam.fiducialObservations) {
                if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
                    Logger.recordOutput(logPrefix + "/RejectionReason", "ambiguity");
                    return Optional.empty();
                }
            }

            if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
                Logger.recordOutput(logPrefix + "/RejectionReason", "area");
                return Optional.empty();
            }

            if (poseEstimate.avgTagDist() > VisionConstants.kMaxSingleTagDistanceMeters) {
                Logger.recordOutput(logPrefix + "/RejectionReason", "singleTagDistance");
                return Optional.empty();
            }
        }

        if (poseEstimate.fieldToRobot().getTranslation().getNorm()
                < VisionConstants.kDefaultNormThreshold) {
            Logger.recordOutput(logPrefix + "/RejectionReason", "norm");
            return Optional.empty();
        }

        if (cam.pose3d != null
                && Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
            Logger.recordOutput(logPrefix + "/RejectionReason", "zHeight");
            return Optional.empty();
        }

        // Reject poses outside the field boundary (plus a small margin)
        if (isOutsideField(poseEstimate.fieldToRobot().getTranslation())) {
            Logger.recordOutput(logPrefix + "/RejectedOutOfField", poseEstimate.fieldToRobot());
            Logger.recordOutput(logPrefix + "/RejectionReason", "outOfField");
            return Optional.empty();
        }

        // Exclusive-tag filtering
        var exclusiveTag = state.getExclusiveTag();
        boolean hasExclusiveId =
                exclusiveTag.isPresent()
                        && java.util.Arrays.stream(poseEstimate.fiducialIds())
                                .anyMatch(id -> id == exclusiveTag.get());

        if (exclusiveTag.isPresent() && !hasExclusiveId) {
            Logger.recordOutput(logPrefix + "/RejectionReason", "exclusiveTag");
            return Optional.empty();
        }

        Pose2d estimatePose = poseEstimate.fieldToRobot();

        // Distance-scaled XY std dev: trust vision less as tags get farther and as fewer tags are
        // seen. Floored and sanitized so a 0/NaN/inf value can never reach the estimator.
        double xyStd =
                distanceScaledXyStdDev(
                        poseEstimate.avgTagDist(), poseEstimate.fiducialIds().length);

        // Heading trust: lean on the gyro (large yaw std dev) while enabled. While disabled, a
        // reliable multi-tag MT1 fix is trusted tightly so it seeds/corrects odometry heading
        // before
        // auto starts (see suppressVisionForSeconds note).
        double rotStd;
        if (multiTag && DriverStation.isDisabled()) {
            rotStd = VisionConstants.kVisionYawStdDevSeedMultiTag;
        } else if (multiTag) {
            rotStd = VisionConstants.kVisionYawStdDevMultiTag;
        } else {
            rotStd = VisionConstants.kVisionYawStdDevSingleTag;
        }

        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

        Logger.recordOutput(logPrefix + "/RejectionReason", "accepted");

        return Optional.of(
                new VisionFieldPoseEstimate(
                        estimatePose,
                        poseEstimate.timestampSeconds(),
                        visionStdDevs,
                        poseEstimate.fiducialIds().length));
    }

    /**
     * Distance-scaled XY std dev: {@code kXYStdDevBase * avgTagDist / tagCount}, clamped to {@code
     * >= kMinVisionXYStdDev} and sanitized against 0/NaN/inf.
     */
    private static double distanceScaledXyStdDev(double avgTagDist, int tagCount) {
        int tags = Math.max(tagCount, 1);
        double xyStd = VisionConstants.kXYStdDevBase * avgTagDist / tags;
        return sanitizeXyStdDev(Math.max(xyStd, VisionConstants.kMinVisionXYStdDev));
    }

    /** Guarantees a strictly-positive, finite XY std dev (never 0/NaN/inf). */
    private static double sanitizeXyStdDev(double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            return VisionConstants.kMinVisionXYStdDev;
        }
        return value;
    }

    /** Returns true if the translation lies outside the field perimeter plus a small margin. */
    private static boolean isOutsideField(Translation2d translation) {
        double margin = VisionConstants.kFieldBoundsMarginMeters;
        return translation.getX() < -margin
                || translation.getX() > VisionConstants.kFieldLengthMeters + margin
                || translation.getY() < -margin
                || translation.getY() > VisionConstants.kFieldWidthMeters + margin;
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }

    private static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};
    private static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};
    private static final int[] kAllTags = {};

    private void applyTagFilter(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.kLimelightATableName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.kLimelightBTableName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.kLimelightCTableName, ids);
    }

    /** Restricts all limelights to the current alliance's hub AprilTags. */
    public Command setHubOnlyFilter() {
        return Commands.runOnce(
                        () -> {
                            int[] ids =
                                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                                                    == DriverStation.Alliance.Red
                                            ? kRedHubTags
                                            : kBlueHubTags;
                            applyTagFilter(ids);
                        })
                .ignoringDisable(true);
    }

    /** Clears the AprilTag filter on all limelights (accept all tags). */
    public Command clearTagFilter() {
        return Commands.runOnce(() -> applyTagFilter(kAllTags)).ignoringDisable(true);
    }
}
