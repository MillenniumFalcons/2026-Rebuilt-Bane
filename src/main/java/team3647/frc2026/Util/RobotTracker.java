package team3647.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;

public class RobotTracker extends VirtualSubsystem implements AllianceObserver {

    private Pose2d hubPose;
    private Pose2d feedPose;
    private Alliance color;

    private final double exitSpeedMultiplier = 0.33;

    private final Field2d smartDashboardField;

    Supplier<Pose2d> drivePose;
    Supplier<ChassisSpeeds> driveSpeeds;

    private final Transform2d robotToShooter;

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

    public RobotTracker(
            Pose2d hubPose,
            Pose2d feedPose,
            Transform2d robotToShooter,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> driveSpeeds) {
        // this.color = Alliance.Red;

        this.hubPose = AllianceFlip.flip(hubPose, this.color);
        this.robotToShooter = robotToShooter;
        this.feedPose = AllianceFlip.flip(feedPose, this.color);
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
        Logger.recordOutput("Robot/Distance", getDistanceFromHub());
        Logger.recordOutput("Robot/Pose", periodicIO.pose);
        Logger.recordOutput("field/hubPose", this.hubPose);
        Logger.recordOutput("field/feedpose", this.feedPose);
        smartDashboardField.setRobotPose(periodicIO.pose);
    }

    public void setPose() {
        periodicIO.pose = drivePose.get();
    }

    // public Pose2d compensate(Pose2d pose) {
    //     final double rawShotSpeed = ShooterConstants.shootSpeedMap.get(periodicIO.distanceFromHub);
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

    public Pose2d compensate(Pose2d pose) {
        // Start with the current actual distance
        double currentDistance = periodicIO.distanceFromHub;
        double time = 0.0;
        double speed = 0.0;
        Pose2d lookaheadPose = pose;

        // Iterate 5 times to converge on the accurate lookahead pose
        for (int i = 0; i < 5; i++) {
            // double rawShotSpeed = ShooterConstants.shootSpeedMap.get(currentDistance);
            // double hoodAngle = HoodConstants.hoodAngleMap.get(currentDistance);

            // // Calculate the horizontal exit speed
            // speed = rawShotSpeed * Math.cos(Math.toRadians(hoodAngle)) * exitSpeedMultiplier;

            // Calculate estimated Time of Flight
            //time = currentDistance / speed;

            time = timeOfFlightMap.get(currentDistance);

            // Calculate how far the robot will travel during that Time of Flight
            var transform = new Twist2d(
                    periodicIO.speeds.vxMetersPerSecond * time,
                    periodicIO.speeds.vyMetersPerSecond * time,
                    0);

            // Apply the offset to get the predicted position
            lookaheadPose = pose.exp(transform);

            // Update the distance based on this new position for the next loop
            currentDistance = getDistanceFromHub(lookaheadPose);
        }

        // Log the final converged values
        Logger.recordOutput("compensate time", time);
        Logger.recordOutput("compensate distance", currentDistance);
        Logger.recordOutput("compensate speed", speed);

        return lookaheadPose;
    }

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
        var shooterPose = currentPose.transformBy(robotToShooter);
        var shooterDistance = hubPose.minus(shooterPose);
        periodicIO.distanceFromHub = shooterDistance.getTranslation().getNorm();
    }

    public double getDistanceFromHub() {
        return periodicIO.distanceFromHub;
    }

    public double getDistanceFromHub(Pose2d pose) {
        return pose.transformBy(robotToShooter).minus(hubPose).getTranslation().getNorm();
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
        return this.feedPose;
    }

    @Override
    public void onAllianceFound(Alliance color) {
        // remove debug stmts

        this.color = color;

        hubPose = AllianceFlip.flip(hubPose, color);

        feedPose = AllianceFlip.flip(feedPose, color);
    }
}
