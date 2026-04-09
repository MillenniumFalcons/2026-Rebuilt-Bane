package team3647.frc2026.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.util.TargetingUtil.AimingParameters;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;

public class AutoDrive extends VirtualSubsystem implements AllianceObserver {

    private final Supplier<Pose2d> odoPoseFunction;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    // these are for sim
    // public final PIDController xController;
    // public final PIDController yController;
    // public final PIDController rotController;

    // this is for real robot
    public final PIDController rotController = new PIDController(2.75, 0, 0);

    private Alliance color = Alliance.Red;

    private DriveMode wantedMode = DriveMode.SHOOT_ON_THE_MOVE;

    public boolean enabled = false;
    public boolean isRotAligned = false;

    private final TargetingUtil targeting;

    public AutoDrive(
            Supplier<Pose2d> odoPoseFunction,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            TargetingUtil targeting) {
        super();
        this.odoPoseFunction = odoPoseFunction;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.targeting = targeting;

        rotController.setTolerance(Units.degreesToRadians(1.5), Units.degreesToRadians(1.5));
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public enum DriveMode {
        SHOOT_ON_THE_MOVE, // regular
        SHOOT, // prealign shoot
        FEED, // shoot feed
        INTAKE_FEED, // reverse intake feed
        INTAKE
    }

    public boolean rotAligned() {
        final Pose2d robotPose = getPose();
        Rotation2d angleToTarget =
                (wantedMode == DriveMode.SHOOT_ON_THE_MOVE)
                        ? targeting.shootAtHubOnTheMove().rotation
                        : targeting.shootAtPoseOnTheMove(targeting.robotTracker.getFeed()).rotation;
        if (wantedMode == DriveMode.INTAKE) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(180));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(0));
            }
        }
        if (wantedMode == DriveMode.INTAKE_FEED) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(0));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(180));
            }
        }
        Logger.recordOutput(
                "Shooting/Rotation/robot rotation", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Shooting/Rotation/wanted rotation", angleToTarget.getDegrees());
        Logger.recordOutput(
                "Shooting/Rotation/difference in angle",
                Math.abs(robotPose.getRotation().getDegrees() - angleToTarget.getDegrees()));
        return Math.abs(robotPose.getRotation().getDegrees() - angleToTarget.getDegrees()) < 5.0;
    }

    public double getHoodAngle() {
        if (enabled) {
            if (wantedMode == DriveMode.FEED) {
                return HoodConstants.kFeedAngle.magnitude();
            } else if (wantedMode == DriveMode.INTAKE_FEED
                    || wantedMode == DriveMode.INTAKE
                    || wantedMode == DriveMode.SHOOT) {
                return 1;
            }
            return (wantedMode == DriveMode.SHOOT_ON_THE_MOVE)
                    ? targeting.shootAtHubOnTheMove().pivot
                    : targeting.shootAtPoseOnTheMove(targeting.robotTracker.getFeed()).pivot;
        }
        return 1;
    }

    public double getShotSpeed() {
        return (wantedMode == DriveMode.SHOOT_ON_THE_MOVE || wantedMode == DriveMode.SHOOT)
                ? targeting.shootAtHubOnTheMove().shootSpeed
                : targeting.shootAtPoseOnTheMove(targeting.robotTracker.getFeed()).shootSpeed;
    }

    public Pose2d getPose() {
        return odoPoseFunction.get();
    }

    public Rotation2d getOdoRot() {
        return odoPoseFunction.get().getRotation();
    }

    public Command enableAutoDrive() {
        return Commands.runOnce(
                () -> {
                    this.rotController.reset();
                    this.enabled = true;
                });
    }

    public Command disableAutoDrive() {
        return Commands.runOnce(() -> this.enabled = false);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean getAutoDriveEnabled() {
        return enabled;
    }

    public Command setDriveMode(DriveMode mode) {
        return Commands.runOnce(() -> this.wantedMode = mode).ignoringDisable(false);
    }

    /** Returns true if the robot is beyond the trench line (in mid). */
    public boolean isInMid() {
        double robotX = getPose().getX();
        if (color == Alliance.Blue) {
            return robotX > FieldConstants.kBlueTrenchX;
        } else {
            return robotX < FieldConstants.kRedTrenchX;
        }
    }

    /** Updates drive mode based on robot pose: feed if in mid, shoot if not. */
    public void updateDriveModeByPose() {
        if (isInMid()) {
            this.wantedMode = DriveMode.FEED;
        } else {
            this.wantedMode = DriveMode.SHOOT_ON_THE_MOVE;
        }
    }

    public DriveMode getWantedMode() {
        return wantedMode;
    }

    public Twist2d getVelocities() {

        final Pose2d robotPose = getPose();
        final ChassisSpeeds currentSpeeds = chassisSpeedsSupplier.get();
        final ChassisSpeeds fieldSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, getOdoRot());

        AimingParameters targetData =
                wantedMode == DriveMode.SHOOT_ON_THE_MOVE
                        ? targeting.shootAtHubOnTheMove()
                        : targeting.shootAtPoseOnTheMove(targeting.robotTracker.getFeed());

        if (wantedMode == DriveMode.SHOOT) {
            targetData = targeting.shootAtHubStationary();
        }

        Translation2d robotToTarget = targetData.robotToTarget;
        double distanceToTarget = targetData.effectiveDistance;
        Rotation2d angleToTarget = targetData.rotation;

        if (wantedMode == DriveMode.INTAKE) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(180));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(0));
            }
        }

        if (wantedMode == DriveMode.INTAKE_FEED) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(0));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(180));
            }
        }

        Logger.recordOutput(
                "SOTMAutoAlign/wantedAngle",
                new Pose2d(robotPose.getX(), robotPose.getY(), angleToTarget));

        // Standard PID output
        double rotFeedback =
                rotController.calculate(
                        robotPose.getRotation().getRadians(), angleToTarget.getRadians());

        // goofy ahh feedforward
        // Calculate the magnitude of linear velocity from ChassisSpeeds
        // double linearVelocityMag =
        //         Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // Calculate angular error to help determine direction of FF
        // double angularError = angleToTarget.minus(robotPose.getRotation()).getRadians();

        // FF = (Linear Speed * FF Gain) * sign of the error
        // This effectively "boosts" the rotation command more when moving fast
        // double angleROC = targeting.getHubAngleROC();
        double customFF =
                (robotToTarget.getY() * fieldSpeeds.vxMetersPerSecond
                                - robotToTarget.getX() * fieldSpeeds.vyMetersPerSecond)
                        / (distanceToTarget * distanceToTarget);

        // Logger.recordOutput("linear velocity", linearVelocityMag);
        // Logger.recordOutput("angular error", angularError);
        Logger.recordOutput("SOTMAutoAlign/DEBUG/ff output", customFF);
        Logger.recordOutput("SOTMAutoAlign/DEBUG/pid output", rotFeedback);

        double rotComp = rotFeedback + customFF;

        return new Twist2d(0, 0, rotComp);
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;
        // Logger.recordOutput("@autodrive", color);
        // DriverStation.reportError("MUSTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARD", false);
    }

    @Override
    public void periodic() {}
}
