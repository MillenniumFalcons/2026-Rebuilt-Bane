package team3647.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.AprilTagCamera;
import team3647.lib.vision.NeuralDetector;

public class AutoDrive extends VirtualSubsystem implements AllianceObserver {

    private final Supplier<Pose2d> odoPoseFunction;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private NeuralDetector detector;
    private AprilTagCamera frontLL;

    // these are for sim
    // public final PIDController xController;
    // public final PIDController yController;
    // public final PIDController rotController;

    // this is for real robot
    public final ProfiledPIDController profiledRotController =
            new ProfiledPIDController(3.7, 0, 0.2, new Constraints(10, 10));

    private final PIDController intakeXController, intakeYController, intakeRotController;

    private Alliance color = Alliance.Red;

    private DriveMode wantedMode = DriveMode.SHOOT_ON_THE_MOVE;

    public boolean enabled = false;
    public boolean isRotAligned = false;

    public final Pose2d hubPose = FieldConstants.hubPose;

    private final TargetingUtil targeting;

    private static final double ff = 1.25;

    public AutoDrive(
            Supplier<Pose2d> odoPoseFunction,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            PIDController xController,
            PIDController yController,
            PIDController rotController,
            NeuralDetector detector,
            TargetingUtil targeting) {
        super();
        this.odoPoseFunction = odoPoseFunction;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        // this.xController = Utils.isSimulation() ? new PIDController(2.5, 0, 0) : xController;
        // this.yController = Utils.isSimulation() ? new PIDController(2.5, 0, 0) : yController;
        // this.rotController = Utils.isSimulation() ? new PIDController(2, 0, 0) : rotController;
        // this.rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.targeting = targeting;

        profiledRotController.setTolerance(Units.degreesToRadians(3), Units.degreesToRadians(3));
        profiledRotController.enableContinuousInput(-Math.PI, Math.PI);

        this.intakeXController = new PIDController(4, 0, 0);
        this.intakeYController = new PIDController(4, 0, 0);
        this.intakeRotController = new PIDController(2, 0, 0);
    }

    public enum DriveMode {
        SHOOT_ON_THE_MOVE
    }

    public double getHoodAngle() {
        return targeting.shootAtHubOnTheMove().pivot;
    }

    public double getShotSpeed() {
        return targeting.shootAtHubOnTheMove().shootSpeed;
    }

    public Pose2d getPose() {
        // var inputs = frontLL.QueueToInputs();
        // if (inputs.isPresent()) {
        //    return inputs.get().pose;
        //     // return frontLL.QueueToInputs().get().pose;
        // }
        // DriverStation.reportError(
        //         "@getposeautodrive " + frontLL.QueueToInputs().isPresent(), false);
        return odoPoseFunction.get();
    }

    public Rotation2d getOdoRot() {
        return odoPoseFunction.get().getRotation();
    }

    public Command enableAutoDrive() {
        return Commands.runOnce(() -> this.enabled = true);
    }

    public Command disableAutoDrive() {
        return Commands.runOnce(() -> this.enabled = false);
    }

    public boolean getAutoDriveEnabled() {
        return enabled;
    }

    public Command setDriveMode(DriveMode mode) {
        return Commands.runOnce(() -> this.wantedMode = mode).ignoringDisable(true);
    }

    public DriveMode getWantedMode() {
        return wantedMode;
    }

    public Twist2d getVelocities() {

        final Pose2d robotPose = getPose();
        // final ChassisSpeeds currentSpeeds = chassisSpeedsSupplier.get();

        Rotation2d angleToTarget = targeting.shootAtHubOnTheMove().rotation;

        Logger.recordOutput(
                "wantedAngle", new Pose2d(robotPose.getX(), robotPose.getY(), angleToTarget));

        profiledRotController.setGoal(angleToTarget.getRadians());

        // Standard PID output
        double rotFeedback = profiledRotController.calculate(robotPose.getRotation().getRadians());

        // goofy ahh feedforward
        // Calculate the magnitude of linear velocity from ChassisSpeeds
        // double linearVelocityMag =
        //         Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // Calculate angular error to help determine direction of FF
        // double angularError = angleToTarget.minus(robotPose.getRotation()).getRadians();

        // FF = (Linear Speed * FF Gain) * sign of the error
        // This effectively "boosts" the rotation command more when moving fast
        double customFF =
                profiledRotController.getSetpoint().velocity
                        * MathUtil.clamp(
                                Math.abs(chassisSpeedsSupplier.get().vyMetersPerSecond / 1.5),
                                1,
                                100);

        // Logger.recordOutput("linear velocity", linearVelocityMag);
        // Logger.recordOutput("angular error", angularError);
        Logger.recordOutput("ff output", customFF);

        double rotComp = -(rotFeedback + customFF);

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
