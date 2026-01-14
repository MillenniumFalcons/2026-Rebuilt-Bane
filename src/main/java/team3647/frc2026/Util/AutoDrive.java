package team3647.frc2026.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.TickMeter;

import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.AprilTagCamera;
import team3647.lib.vision.NeuralDetector;

public class AutoDrive extends VirtualSubsystem implements AllianceObserver {

    private final Supplier<Pose2d> odoPoseFunction;

    private NeuralDetector detector;
    private AprilTagCamera frontLL;

    public final PIDController xController;
    public final PIDController yController;
    public final PIDController rotController;
    public final ProfiledPIDController translationController =
            new ProfiledPIDController(4, 0, 0, new Constraints(4, 7));
    public final ProfiledPIDController profiledRotController =
            new ProfiledPIDController(5, 0, 0, new Constraints(4, 7));
    public final ProfiledPIDController bargeRotController =
            new ProfiledPIDController(3, 0, 0, new Constraints(3, 5));

    private final PIDController intakeXController, intakeYController, intakeRotController;

    private Alliance color = Alliance.Red;

    private DriveMode wantedMode = DriveMode.NONE;

    public boolean enabled = false;
    public boolean isYAligned = false;
    public boolean isRotAligned = false;
    public double taTarget = 6.8;

    public boolean shouldStop = false;

    private Superstructure superstructure;

    public AutoDrive(
            Supplier<Pose2d> odoPoseFunction,
            PIDController xController,
            PIDController yController,
            PIDController rotController,
            NeuralDetector detector,
            Superstructure superstructure) {
        super();
        this.odoPoseFunction = odoPoseFunction;
        this.xController = Utils.isSimulation() ? new PIDController(2.5, 0, 0) : xController;
        this.yController = Utils.isSimulation() ? new PIDController(2.5, 0, 0) : yController;
        this.rotController = Utils.isSimulation() ? new PIDController(2, 0, 0) : rotController;
        this.superstructure = superstructure;

        translationController.setTolerance(Units.inchesToMeters(1.5), Units.inchesToMeters(1.5));
        profiledRotController.setTolerance(Units.degreesToRadians(3), Units.degreesToRadians(3));
        profiledRotController.enableContinuousInput(-Math.PI, Math.PI);

        this.rotController.enableContinuousInput(-Math.PI, Math.PI);

        this.intakeXController = new PIDController(4, 0, 0);
        this.intakeYController = new PIDController(4, 0, 0);
        this.intakeRotController = new PIDController(2, 0, 0);
    }

    public enum DriveMode {
        SCORE,
        NONE
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

    public boolean hasScoringTarget() {
        return frontLL.hasTarget();
    }

    public Command clearDriveMode() {
        return setDriveMode(DriveMode.NONE);
    }


    public DriveMode getWantedMode() {
        return wantedMode;
    }

    public Command setShouldStop() {
        return Commands.runOnce(() -> shouldStop = true);
    }

    public Command setShouldNotStop() {
        return Commands.runOnce(() -> shouldStop = false);
    }

    public Twist2d getVelocitiesAuto() {
        // final Pose2d robotPose = getPose();
        // Pose2d wantedPose = AllianceFlip.flip(wantedScoringPos.pose, color);

        // Pose2d robotToTarget = wantedPose.relativeTo(robotPose);
        // Rotation2d angleToTarget = robotToTarget.getTranslation().getAngle();
        // double pidOutput =
        //         translationController.calculate(robotToTarget.getTranslation().getNorm());
        // Translation2d pidTranslation = new Translation2d(pidOutput, angleToTarget);

        // profiledRotController.setGoal(wantedPose.getRotation().getRadians());
        // translationController.setGoal(0);

        // double xComp = -pidTranslation.getX();
        // double yComp = -pidTranslation.getY();
        // double rotComp = profiledRotController.calculate(robotPose.getRotation().getRadians());

        // xComp = translationController.atGoal() ? 0 : xComp;
        // yComp = translationController.atGoal() ? 0 : yComp;
        // rotComp = profiledRotController.atGoal() ? 0 : rotComp;

        // return new Twist2d(xComp, yComp, rotComp);
        return new Twist2d();
    }

    public Twist2d getVelocities() {
        // final Pose2d robotPose = getPose();
        // Pose2d wantedPose;

        // switch (wantedMode) {
        //     case SCORE:
        //         wantedPose = AllianceFlip.flip(wantedScoringPos.pose, color);
        //         break;
        //     case ALGAE_REEF:
        //         wantedPose = AllianceFlip.flip(wantedAlgaeReefPos, color);
        //         break;
        //     case BARGE:
        //         Pose2d bargePose = AllianceFlip.flip(FieldConstants.algaeBargePose, color);
        //         double low = color == Alliance.Blue ? bargePose.getY() : Units.inchesToMeters(20);
        //         double high =
        //                 color == Alliance.Blue
        //                         ? FieldConstants.kFieldWidthM - Units.inchesToMeters(20)
        //                         : bargePose.getY();
        //         double y = MathUtil.clamp(getPose().getMeasureY().in(Meters), low, high);
        //         wantedPose = new Pose2d(bargePose.getX(), y, bargePose.getRotation());
        //         Logger.recordOutput("Wanted Barge pose", wantedPose);
        //         bargeRotController.setGoal(bargePose.getRotation().getRadians());
        //         break;
        //     default:
        //         wantedPose = AllianceFlip.flip(wantedScoringPos.pose, color);
        //         break;
        // }

        // Pose2d robotToTarget = wantedPose.relativeTo(robotPose);
        // Rotation2d angleToTarget = robotToTarget.getTranslation().getAngle();
        // double pidOutput =
        //         translationController.calculate(robotToTarget.getTranslation().getNorm());
        // Translation2d pidTranslation = new Translation2d(pidOutput, angleToTarget);

        // profiledRotController.setGoal(wantedPose.getRotation().getRadians());
        // translationController.setGoal(0);

        // double xComp = -pidTranslation.getX();
        // double yComp = -pidTranslation.getY();
        // double rotComp =
        //         wantedMode == DriveMode.BARGE
        //                 ? bargeRotController.calculate(robotPose.getRotation().getRadians())
        //                 : profiledRotController.calculate(robotPose.getRotation().getRadians());

        // xComp = translationController.atGoal() ? 0 : xComp;
        // yComp = translationController.atGoal() ? 0 : yComp;
        // rotComp = profiledRotController.atGoal() ? 0 : rotComp;

        // if (RobotModeTriggers.autonomous().getAsBoolean()) {
        //     xComp =
        //             detector.getTY() != 0
        //                     ? intakeXController.calculate(Math.toRadians(detector.getTY()))
        //                     : -1;

        //     yComp =
        //             detector.getTX() != 0
        //                     ? intakeYController.calculate(
        //                             Math.toRadians(0), Math.toRadians(detector.getTX()))
        //                     : 0;
        //     rotComp = -yComp * 2;
        // }

        // return new Twist2d(xComp, yComp, rotComp);
        return new Twist2d();
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;
        // Logger.recordOutput("@autodrive", color);
        // DriverStation.reportError("MUSTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARD", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("xController", xController);
        SmartDashboard.putData("yController", yController);
        SmartDashboard.putData("rotController", rotController);
    }
}
