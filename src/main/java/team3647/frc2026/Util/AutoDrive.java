package team3647.frc2026.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.HoodConstants;
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
    public final PIDController rotController = new PIDController(4.0, 0, 0);
    // public final double kFF = 0.5;

    // Distance (m) from the nearer tower upright (the outer edge of the tower) within which
    // auto-align orbits that upright instead of spinning about the robot center.
    public static final double kTowerPivotDistance = 1.0;

    private Alliance color = Alliance.Red;

    // Tower uprights for the current alliance (defaults to blue; swapped in onAllianceFound).
    private Translation2d towerPoleA = FieldConstants.blueTowerPoleA;
    private Translation2d towerPoleB = FieldConstants.blueTowerPoleB;

    private DriveMode wantedMode = DriveMode.SHOOT;

    public boolean enabled = false;
    public boolean isRotAligned = false;

    private Angle manualHoodAngle = Degrees.of(1);

    private final TargetingUtil targeting;

    private TargetingUtil.AimingParameters cachedAimingParams = null;

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
        SHOOT_ON_THE_MOVE, // dont use
        SHOOT, // prealign shoot
        FEED, // shoot feed
        INTAKE_FEED, // reverse intake feed
        INTAKE,
        MANUAL_SHOOT
    }

    public boolean rotAligned() {
        if (wantedMode == DriveMode.FEED) {
            return true;
        }
        final Pose2d robotPose = getPose();
        Rotation2d angleToTarget =
                cachedAimingParams != null ? cachedAimingParams.rotation : new Rotation2d();
        if (wantedMode == DriveMode.INTAKE) {
            // Past the trench, face the opposite wall (swap 0 and 180).
            boolean past = isInMid();
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(past ? 0 : 180));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(past ? 180 : 0));
            }
        } else if (wantedMode == DriveMode.INTAKE_FEED) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(0));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(180));
            }
        }
        Logger.recordOutput(
                "Shooting/Rotation/robot rotation",
                Math.round(robotPose.getRotation().getDegrees() * 100.0) / 100.0);
        Logger.recordOutput(
                "Shooting/Rotation/wanted rotation",
                Math.round(angleToTarget.getDegrees() * 100.0) / 100.0);
        Logger.recordOutput(
                "Shooting/Rotation/difference in angle",
                Math.round(
                                Math.abs(
                                                robotPose.getRotation().getDegrees()
                                                        - angleToTarget.getDegrees())
                                        * 100.0)
                        / 100.0);

        return Math.abs(robotPose.getRotation().getDegrees() - angleToTarget.getDegrees()) < 4;
    }

    /** Absolute heading error (degrees) between the robot and the current aim target. */
    public double headingErrorDegrees() {
        final Pose2d robotPose = getPose();
        Rotation2d angleToTarget =
                cachedAimingParams != null ? cachedAimingParams.rotation : new Rotation2d();
        return Math.abs(robotPose.getRotation().minus(angleToTarget).getDegrees());
    }

    public boolean rotAlignedAuto() {
        final Pose2d robotPose = getPose();
        Rotation2d angleToTarget =
                cachedAimingParams != null ? cachedAimingParams.rotation : new Rotation2d();
        return Math.abs(robotPose.getRotation().getDegrees() - angleToTarget.getDegrees()) < 6;
    }

    public double getHoodAngle() {
        if (enabled) {
            if (wantedMode == DriveMode.FEED) {
                return HoodConstants.kFeedAngle.magnitude();
            } else if (wantedMode == DriveMode.INTAKE_FEED || wantedMode == DriveMode.INTAKE) {
                return 1;
            }
            return cachedAimingParams != null ? cachedAimingParams.pivot : 1;
        } else if (wantedMode == DriveMode.MANUAL_SHOOT) {
            return manualHoodAngle.in(Degrees);
        }
        return 1;
    }

    public double getShotSpeed() {
        return cachedAimingParams != null ? cachedAimingParams.shootSpeed : 0;
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

    public Command incrementHood(double angleDegrees) {
        return Commands.runOnce(
                () -> {
                    Angle newAngle = manualHoodAngle.plus(Degrees.of(angleDegrees));
                    Logger.recordOutput("Shooting/Hood/manual angle", manualHoodAngle.in(Degrees));
                    Logger.recordOutput("Shooting/Hood/new angle", newAngle.in(Degrees));
                    if (newAngle.gt(HoodConstants.kMinAngle)
                            && newAngle.lt(HoodConstants.kMaxAngle)) {
                        manualHoodAngle = newAngle;
                    }
                });
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

        if (cachedAimingParams == null) {
            return new Twist2d();
        }

        Translation2d robotToTarget = cachedAimingParams.robotToTarget;
        double distanceToTarget = cachedAimingParams.effectiveDistance;
        Rotation2d angleToTarget = cachedAimingParams.rotation;

        if (wantedMode == DriveMode.INTAKE) {
            // Past the trench, face the opposite wall (swap 0 and 180).
            boolean past = isInMid();
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(past ? 0 : 180));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(past ? 180 : 0));
            }
        }

        if (wantedMode == DriveMode.INTAKE_FEED) {
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(0));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(180));
            }
        }

        if (wantedMode == DriveMode.FEED) {
            // Point directly toward the alliance zone (180 deg on blue, flipped on red).
            if (color == Alliance.Blue) {
                angleToTarget = new Rotation2d(Degrees.of(180));
            } else {
                angleToTarget = new Rotation2d(Degrees.of(0));
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
        // double customFF = kFF * Math.signum(robotPose.getRotation().getRadians() -
        // angleToTarget.getRadians());
        // (robotToTarget.getY() * fieldSpeeds.vxMetersPerSecond
        //                 - robotToTarget.getX() * fieldSpeeds.vyMetersPerSecond)
        //         / (distanceToTarget * distanceToTarget);

        // Logger.recordOutput("linear velocity", linearVelocityMag);
        // Logger.recordOutput("angular error", angularError);
        // Logger.recordOutput("SOTMAutoAlign/DEBUG/ff output", customFF);
        Logger.recordOutput("SOTMAutoAlign/DEBUG/pid output", rotFeedback);

        double rotComp = rotFeedback;

        // When auto-aligning to shoot up close to the tower, orbit the nearer upright instead of
        // spinning about the robot center. For a rigid body rotating about a pivot P at angular
        // rate w, the center C must translate with v = w x (C - P). Here w = rotComp (CCW, field
        // frame), so vx = -w * (Cy - Py), vy = w * (Cx - Px). The heading PID keeps correcting the
        // aim as the robot swings around, so the two stay coupled.
        // double dx = 0;
        // double dy = 0;
        // if (enabled
        //         && (wantedMode == DriveMode.SHOOT || wantedMode == DriveMode.SHOOT_ON_THE_MOVE))
        // {
        //     Translation2d pivot = getPivotPole();
        //     if (pivot != null) {
        //         double rx = robotPose.getX() - pivot.getX();
        //         double ry = robotPose.getY() - pivot.getY();
        //         dx = -rotComp * ry;
        //         dy = rotComp * rx;
        //         Logger.recordOutput(
        //                 "SOTMAutoAlign/orbitPivot", new Pose2d(pivot, new Rotation2d()));
        //     }
        // }
        // Logger.recordOutput("SOTMAutoAlign/orbiting", dx != 0 || dy != 0);
        // Logger.recordOutput("SOTMAutoAlign/orbitVelX", dx);
        // Logger.recordOutput("SOTMAutoAlign/orbitVelY", dy);

        return new Twist2d(0, 0, rotComp);
    }

    /**
     * Returns the tower upright to orbit when auto-aligning to shoot, or {@code null} to rotate
     * about the robot center as usual. Only orbits an upright when the robot's y is beyond that
     * upright (to its side) and the robot is within {@link #kTowerPivotDistance} of it: orbit the
     * low-y ("right") pole only when the robot is to the right of it, the high-y ("left") pole only
     * when to the left of it. Between the two uprights, returns {@code null}.
     */
    private Translation2d getPivotPole() {
        Translation2d robot = getPose().getTranslation();
        Translation2d lowPole = towerPoleA.getY() <= towerPoleB.getY() ? towerPoleA : towerPoleB;
        Translation2d highPole = towerPoleA.getY() <= towerPoleB.getY() ? towerPoleB : towerPoleA;

        Translation2d pivot;
        if (robot.getY() < lowPole.getY()) {
            pivot = lowPole; // robot is beyond the low-y (right) pole
        } else if (robot.getY() > highPole.getY()) {
            pivot = highPole; // robot is beyond the high-y (left) pole
        } else {
            return null; // between the uprights: rotate about the robot center
        }
        return robot.getDistance(pivot) <= kTowerPivotDistance ? pivot : null;
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;

        if (color == Alliance.Blue) {
            towerPoleA = FieldConstants.blueTowerPoleA;
            towerPoleB = FieldConstants.blueTowerPoleB;
        } else {
            towerPoleA = FieldConstants.redTowerPoleA;
            towerPoleB = FieldConstants.redTowerPoleB;
        }
        // Logger.recordOutput("@autodrive", color);
        // DriverStation.reportError("MUSTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARD", false);
    }

    @Override
    public void periodic() {
        if (wantedMode == DriveMode.SHOOT) {
            cachedAimingParams = targeting.shootAtHubStationary();
        } else if (wantedMode == DriveMode.SHOOT_ON_THE_MOVE) {
            cachedAimingParams = targeting.shootAtHubOnTheMove();
        } else {
            cachedAimingParams = targeting.feedAtPose(targeting.robotTracker.getFeed());
        }
    }
}
