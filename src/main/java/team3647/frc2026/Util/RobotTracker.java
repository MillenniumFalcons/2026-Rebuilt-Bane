package team3647.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;

public class RobotTracker extends VirtualSubsystem implements AllianceObserver {

    private Pose2d hubPose = FieldConstants.blueHubPose;
    private Pose2d feedPoseRight = FieldConstants.blueFeedPoseRight;
    private Pose2d feedPoseLeft = FieldConstants.blueFeedPoseLeft;
    private Pose2d feedPose = feedPoseRight;
    private Alliance color = Alliance.Blue;

    private final Field2d smartDashboardField;

    Supplier<Pose2d> drivePose;
    Supplier<ChassisSpeeds> driveSpeeds;

    private PeriodicIO periodicIO = new PeriodicIO();

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    static {
        timeOfFlightMap.put(1.0, 1.0);
        timeOfFlightMap.put(2.0, 1.1);
        timeOfFlightMap.put(3.0, 1.05);
        timeOfFlightMap.put(4.0, 1.16);
        timeOfFlightMap.put(5.0, 1.25);
    }

    public static class PeriodicIO {
        private double distanceFromHub = 0;
        private Pose2d pose = new Pose2d();
        private ChassisSpeeds speeds = new ChassisSpeeds();
    }

    public RobotTracker(Supplier<Pose2d> drivePose, Supplier<ChassisSpeeds> driveSpeeds) {

        this.drivePose = drivePose;
        this.driveSpeeds = driveSpeeds;

        this.smartDashboardField = new Field2d();
        SmartDashboard.putData("Field", smartDashboardField);
    }

    @Override
    public void periodic() {
        // setPose();
        setSpeeds();
        setPose();
        setDistanceFromHub();
        Logger.recordOutput("Robot/DistanceFromHub", getDistanceFromHub());
        Logger.recordOutput("Robot/Pose", periodicIO.pose);
        Logger.recordOutput("Field/hubPose", this.hubPose);
        Logger.recordOutput("Field/feedpose", this.feedPose);
        smartDashboardField.setRobotPose(periodicIO.pose);
    }

    public void setPose() {
        periodicIO.pose = drivePose.get();
    }

    public double getIntakeDirectionSpeed() {
        return periodicIO.speeds.vxMetersPerSecond;
    }

    // public Pose2d compensate(Pose2d pose) {
    //     final double rawShotSpeed =
    // ShooterConstants.shootSpeedMap.get(periodicIO.distanceFromHub);
    //     final double hoodAngle = HoodConstants.hoodAngleMap.get(periodicIO.distanceFromHub);
    //     final double shootSpeed =
    //             rawShotSpeed * Math.cos(Math.toRadians(hoodAngle)) * exitSpeedMultiplier;
    //     double time = periodicIO.distanceFromHub / shootSpeed;

    //     var transform =
    //             new Twist2d(
    //                     periodicIO.speeds.vxMetersPerSecond * time,
    //                     periodicIO.speeds.vyMetersPerSecond * time,
    //                     0);
    //     var middlePose = pose.exp(transform);

    //     double newDistance = getDistanceFromHub(middlePose);

    //     double newRawShotSpeed = ShooterConstants.shootSpeedMap.get(newDistance);
    //     double newHoodAngle = HoodConstants.hoodAngleMap.get(newDistance);
    //     double newSpeed =
    //             newRawShotSpeed * Math.cos(Math.toRadians(newHoodAngle)) * exitSpeedMultiplier;
    //     double newTime = newDistance / newSpeed;

    //     Logger.recordOutput("compensate time", newTime);
    //     Logger.recordOutput("compensate distance", newDistance);
    //     Logger.recordOutput("compensate speed", newSpeed);

    //     var newTransform =
    //             new Twist2d(
    //                     periodicIO.speeds.vxMetersPerSecond * newTime,
    //                     periodicIO.speeds.vyMetersPerSecond * newTime,
    //                     0);
    //     return pose.exp(newTransform);
    // }

    // Fixed-point iteration (old method, kept for reference)
    public Pose2d compensate(Pose2d pose) {
        double currentDistance = periodicIO.distanceFromHub;
        double time = 0.0;
        Pose2d lookaheadPose = pose;
        ChassisSpeeds fieldRelSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
        double vx = fieldRelSpeeds.vxMetersPerSecond;
        double vy = fieldRelSpeeds.vyMetersPerSecond;

        for (int i = 0; i < 10; i++) {
            time = timeOfFlightMap.get(currentDistance);
            lookaheadPose =
                    new Pose2d(
                            pose.getX() + vx * time, pose.getY() + vy * time, pose.getRotation());
            currentDistance = getDistanceFromHub(lookaheadPose);
            Logger.recordOutput("SOTMAutoAlign/lookahead", lookaheadPose);
        }

        Logger.recordOutput("SOTMValues/compensate time", time);
        Logger.recordOutput("SOTMValues/compensate distance", currentDistance);
        return lookaheadPose;
    }

    public double getSpeedToHub() {
        ChassisSpeeds fieldRelSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
        Translation2d toHub = hubPose.getTranslation().minus(getPose().getTranslation());
        return (toHub.getX() * fieldRelSpeeds.vxMetersPerSecond
                        + toHub.getY() * fieldRelSpeeds.vyMetersPerSecond)
                / toHub.getNorm();
    }

    // Newton's method for dynamic shooting per:
    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
    //
    // Solves: E(τ) = τ - TOF(D(τ)) = 0
    //   where D(τ) = distance from lookahead shooter position to hub
    //
    // E'(τ) = 1 - τ'(D) * dD/dτ
    //   dD/dτ = -(dx*vx + dy*vy) / D   (analytic, from displacement vector)
    //   τ'(D) = numerical derivative of TOF lookup table
    //
    // Initial guess: τ = D / (vp + |v|cos(θ))  where vp ≈ D/TOF(D)
    // public Pose2d compensate(Pose2d pose) {
    //     double currentDistance = periodicIO.distanceFromHub;
    //     double time = timeOfFlightMap.get(currentDistance);
    //     Pose2d lookaheadPose = pose;
    //     ChassisSpeeds fieldRelSpeeds =
    //             ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(),
    // getPose().getRotation());
    //     double vx = fieldRelSpeeds.vxMetersPerSecond;
    //     double vy = fieldRelSpeeds.vyMetersPerSecond;

    //     for (int i = 0; i < 10; i++) {
    //         currentDistance = getDistanceFromHub(lookaheadPose);
    //         double error = time - timeOfFlightMap.get(currentDistance);
    //         double distanceDerivative =
    //                 (-(hubPose.getX() - lookaheadPose.getX()) * vx
    //                         - (hubPose.getY() - lookaheadPose.getY()) * vy)/currentDistance;
    //         double distanceDerivativeOfTime = time/currentDistance;
    //         double errorDerivative = 1 - distanceDerivativeOfTime * distanceDerivative;
    //         double timeChange = error / errorDerivative;
    //         // Limit time change to prevent divergence
    //         timeChange = MathUtil.clamp(timeChange, -0.2, 0.2);
    //         time = time - timeChange;
    //         lookaheadPose = new Pose2d(pose.getX() + vx * time, pose.getY() + vy * time,
    // pose.getRotation());
    //         Logger.recordOutput("SOTMValues/error", error);
    //         Logger.recordOutput("SOTMValues/error derivative", errorDerivative);
    //     }

    //     Logger.recordOutput("SOTMAutoAlign/lookahead", lookaheadPose);
    //     Logger.recordOutput("SOTMValues/compensate time", time);
    //     Logger.recordOutput("SOTMValues/compensate distance", currentDistance);
    //     return lookaheadPose;
    // }

    public Pose2d compensateFeed(Pose2d pose) {

        double time = 0.3;
        var transform =
                new Twist2d(
                        periodicIO.speeds.vxMetersPerSecond * time,
                        periodicIO.speeds.vyMetersPerSecond * time,
                        0);
        return pose.exp(transform);
    }

    public void setDistanceFromHub() {
        var currentPose = periodicIO.pose;
        periodicIO.distanceFromHub = hubPose.minus(currentPose).getTranslation().getNorm();
    }

    public double getDistanceFromHub() {
        return periodicIO.distanceFromHub;
    }

    public double getDistanceFromHub(Pose2d pose) {
        return pose.minus(hubPose).getTranslation().getNorm();
    }

    public Pose2d getPose() {
        return periodicIO.pose;
    }

    public void setSpeeds() {
        periodicIO.speeds = driveSpeeds.get();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return periodicIO.speeds;
    }

    public Alliance getColor() {
        return color;
    }

    public Pose2d getHub() {
        return this.hubPose;
    }

    public Pose2d getFeed() {
        if (this.color == Alliance.Blue) {
            feedPose = getPose().getY() < 4 ? feedPoseRight : feedPoseLeft;
        } else {
            feedPose = getPose().getY() > 4 ? feedPoseRight : feedPoseLeft;
        }
        return this.feedPose;
    }

    @Override
    public void onAllianceFound(Alliance color) {
        // remove debug stmts

        this.color = color;

        hubPose = AllianceFlip.flip(FieldConstants.blueHubPose, color);

        feedPose = AllianceFlip.flip(feedPose, color);

        feedPoseLeft = AllianceFlip.flip(FieldConstants.blueFeedPoseLeft, color);
        feedPoseRight = AllianceFlip.flip(FieldConstants.blueFeedPoseRight, color);
    }
}
