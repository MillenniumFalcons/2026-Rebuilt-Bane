package team3647.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;

public class RobotTracker extends VirtualSubsystem implements AllianceObserver {

    private Pose2d hubPose;
    private Pose2d feedPose;
    private Alliance color;

    private final Field2d smartDashboardField;

    Supplier<Pose2d> drivePose;
    Supplier<ChassisSpeeds> driveSpeeds;

    private final Transform2d robotToShooter;

    private PeriodicIO periodicIO = new PeriodicIO();

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
        this.color = Alliance.Red;

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

    public Pose2d compensate(Pose2d pose) {
        final double shootSpeed = 20 /* * (1 - periodicIO.distanceFromHub / 40) */;

        double time = periodicIO.distanceFromHub / shootSpeed;
        var transform =
                new Twist2d(
                        periodicIO.speeds.vxMetersPerSecond * time,
                        periodicIO.speeds.vyMetersPerSecond * time,
                        0);
        var middlePose = periodicIO.pose.exp(transform);
        double newDistance = getDistanceFromHub(middlePose);
        // keep newspeed just in case i make the velocity not constant in the future
        double newSpeed = 20 /* * (1 - newDistance / 40) */;
        double newTime = newDistance / newSpeed;
        var newTransform =
                new Twist2d(
                        periodicIO.speeds.vxMetersPerSecond * newTime,
                        periodicIO.speeds.vyMetersPerSecond * newTime,
                        0);
        return pose.exp(newTransform);
    }

    // public Pose2d compensateFeed(Pose2d pose) {

    //     double time = 0.3;
    //     var transform =
    //             new Twist2d(
    //                     periodicIO.speeds.vxMetersPerSecond * time,
    //                     periodicIO.speeds.vyMetersPerSecond * time,
    //                     0);
    //     return pose.exp(transform);
    // }

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
