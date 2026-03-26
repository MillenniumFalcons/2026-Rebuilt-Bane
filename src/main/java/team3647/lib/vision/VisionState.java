package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import team3647.lib.util.ConcurrentTimeInterpolatableBuffer;

/**
 * Tracks robot state needed by the vision subsystem. Provides pose history interpolation, yaw
 * velocity tracking, and exclusive tag filtering.
 *
 * <p>Inspired by Team 254's RobotState, stripped down to only vision-related functionality. The
 * swerve drive should call {@link #addOdometryMeasurement} and {@link #addDriveYawVelocity} each
 * cycle to keep state current.
 */
public class VisionState {

    public static final double LOOKBACK_TIME = 1.0;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    private double lastUsedMegatagTimestamp = 0;
    private Pose2d lastUsedMegatagPose = new Pose2d();

    private final AtomicReference<Optional<Integer>> exclusiveTag =
            new AtomicReference<>(Optional.empty());

    public VisionState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        fieldToRobot.addSample(0.0, new Pose2d());
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    /** Called by swerve drive each cycle to record odometry pose. */
    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    /** Called by swerve drive each cycle to record yaw angular velocity (rad/s). */
    public void addDriveYawVelocity(double timestamp, double yawAngularVelocityRadS) {
        driveYawAngularVelocity.addSample(timestamp, yawAngularVelocityRadS);
    }

    /** Get interpolated pose at a given timestamp. */
    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    /** Get the most recent pose. */
    public Pose2d getLatestFieldToRobot() {
        var latest = fieldToRobot.getLatest();
        return latest == null ? new Pose2d() : latest.getValue();
    }

    /** Get max absolute yaw angular velocity in a time range. */
    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(
            double minTime, double maxTime) {
        if (RobotBase.isReal()) {
            var submap =
                    driveYawAngularVelocity.getInternalBuffer().subMap(minTime, maxTime).values();
            var max = submap.stream().max(Double::compare);
            var min = submap.stream().min(Double::compare);
            if (max.isEmpty() || min.isEmpty()) return Optional.empty();
            if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
            else return min;
        }
        return Optional.of(0.0);
    }

    /** Called by VisionSubsystem when an estimate is accepted. */
    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
        lastUsedMegatagPose = megatagEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public Pose2d lastUsedMegatagPose() {
        return lastUsedMegatagPose;
    }

    public void setExclusiveTag(int id) {
        exclusiveTag.set(Optional.of(id));
    }

    public void clearExclusiveTag() {
        exclusiveTag.set(Optional.empty());
    }

    public Optional<Integer> getExclusiveTag() {
        return exclusiveTag.get();
    }
}
