// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2026.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.ShooterCommands;
import team3647.frc2026.commands.SwerveDriveCommands;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.GlobalConstants;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.KickerConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.SwerveDriveConstants;
import team3647.frc2026.constants.TunerConstants;
import team3647.frc2026.constants.VisionConstants;
import team3647.frc2026.subsystems.Hood;
import team3647.frc2026.subsystems.Kicker;
import team3647.frc2026.subsystems.ShooterLeft;
import team3647.frc2026.subsystems.ShooterRight;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.SwerveDrive;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.LEDTriggers;
import team3647.frc2026.util.RobotTracker;
import team3647.frc2026.util.TargetingUtil;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
// import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.NeuralDetectorLimelight;

public class RobotContainer {
    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public Field2d smartDashboardField = new Field2d();

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        configureSmartDashboardLogging();

        swerve.setRobotPose(new Pose2d(2, 2, Rotation2d.k180deg));

        CommandScheduler.getInstance()
                .registerSubsystem(swerve, shooterOne, shooterTwo, kicker, hood, printer);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive);
    } // 0.6324678425924254

    private void configureBindings() {
        mainController.leftBumper.whileTrue(shooterCommands.setVelocity(10));
        mainController.rightTrigger.whileTrue(superstructure.shoot(20, 1.5));
    }

    private void configureSmartDashboardLogging() {
        // Logger.recordOutput("right cam transform", VisionConstants.rightMount);
        // Logger.recordOutput("left cam transform", VisionConstants.leftMount);

        SmartDashboard.putData(
                "Autos", autoChooser); // , superstructure.score();, superstructure.score();
        SmartDashboard.putData("Field", smartDashboardField);
        printer.addDouble("Match Time", Timer::getMatchTime);
    }

    public void updateRobotPoseForSmartdashboard() {
        smartDashboardField.setRobotPose(swerve.getOdoPose());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                swerveCommands.drive(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        autoDrive::getVelocities,
                        autoDrive::getWantedMode,
                        autoDrive::getAutoDriveEnabled,
                        mainController.rightJoyStickPress));
        shooterOne.setDefaultCommand(superstructure.shooterCommands.kill());
        shooterTwo.setDefaultCommand(superstructure.shooterCommands.kill());
        kicker.setDefaultCommand(superstructure.kickerCommands.kill());
        hood.setDefaultCommand(hoodCommands.setAngleDeg(autoDrive::getHoodAngle));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getAutoCommand();
    }

    @SuppressWarnings("unchecked")
    public final SwerveDrive swerve =
            new SwerveDrive(
                    TunerConstants.DrivetrainConstants,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt,
                    SwerveDriveConstants.kTeleopKinematicLimits,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);

    public final ShooterLeft shooterOne =
            new ShooterLeft(
                    ShooterConstants.kShooterOne,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final ShooterRight shooterTwo =
            new ShooterRight(
                    ShooterConstants.kShooterTwo,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Kicker kicker =
            new Kicker(
                    KickerConstants.kMaster,
                    0.319185814,
                    0,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Hood hood =
            new Hood(
                    HoodConstants.kMaster,
                    HoodConstants.kMaxAngle,
                    HoodConstants.kMinAngle,
                    HoodConstants.kNativeToDeg,
                    HoodConstants.kNativeToDeg,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    NeuralDetectorLimelight detector = new NeuralDetectorLimelight(VisionConstants.kCrossbarCam);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);

    public final ShooterCommands shooterCommands = new ShooterCommands(shooterOne, shooterTwo);

    public final KickerCommands kickerCommands = new KickerCommands(kicker);

    public final HoodCommands hoodCommands = new HoodCommands(hood);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    private final GroupPrinter printer = GroupPrinter.getInstance();

    public final RobotTracker robotTracker =
            new RobotTracker(
                    FieldConstants.hubPose,
                    FieldConstants.feedPose,
                    ShooterConstants.robotToBetweenShooter,
                    swerve::getOdoPose,
                    swerve::getChassisSpeeds);

    public final TargetingUtil targetingUtil = new TargetingUtil(robotTracker);

    public final AutoDrive autoDrive =
            new AutoDrive(
                    swerve::getOdoPose,
                    AutoConstants.autoAlignXController,
                    AutoConstants.autoAlignYController,
                    AutoConstants.rotAlignController,
                    detector,
                    targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(shooterOne, shooterTwo, kicker, hood, autoDrive::getHoodAngle);

    private final LEDTriggers triggers = new LEDTriggers(superstructure);
    // private final LEDs leds = new LEDs(LEDConstants.m_candle, triggers);

    // cams
    AprilTagLimelight drivingCam =
            new AprilTagLimelight(
                    VisionConstants.kDriveCam,
                    VisionConstants.driveMount,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs,
                    autoDrive::getAutoDriveEnabled);

    public final AutoCommands autoCommands =
            new AutoCommands(
                    swerve,
                    superstructure,
                    autoDrive::getVelocities,
                    detector::hasTarget,
                    autoDrive,
                    autoDrive::getVelocitiesAuto);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);
}
