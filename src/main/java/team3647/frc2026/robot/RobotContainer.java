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
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.RollerCommands;
import team3647.frc2026.commands.ShooterCommands;
import team3647.frc2026.commands.SwerveDriveCommands;
import team3647.frc2026.commands.WristCommands;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.GlobalConstants;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.IndexerConstants;
import team3647.frc2026.constants.KickerConstants;
import team3647.frc2026.constants.RollerConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.SwerveDriveConstants;
import team3647.frc2026.constants.TunerConstants;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Hood;
import team3647.frc2026.subsystems.Indexer;
import team3647.frc2026.subsystems.Kicker;
import team3647.frc2026.subsystems.Rollers;
import team3647.frc2026.subsystems.ShooterLeft;
import team3647.frc2026.subsystems.ShooterRight;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.SwerveDrive;
import team3647.frc2026.subsystems.Wrist;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.RobotTracker;
import team3647.frc2026.util.TargetingUtil;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.tunables.TunableNumber;

// import team3647.lib.vision.AprilTagLimelight;
// import team3647.lib.vision.NeuralDetectorLimelight;
// import team3647.lib.vision.VisionController;

public class RobotContainer {
    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public Field2d smartDashboardField = new Field2d();

    public TunableNumber hoodAngle = new TunableNumber("hood angle", 72);

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        configureSmartDashboardLogging();

        wrist.setEncoderAngle(WristConstants.kInitialAngle);
        hood.setEncoderAngle(HoodConstants.kInitialAngle);

        swerve.setRobotPose(new Pose2d(2, 2, Rotation2d.k180deg));

        CommandScheduler.getInstance()
                .registerSubsystem(
                        swerve,
                        shooterLeft,
                        shooterRight,
                        indexer,
                        hood,
                        wrist,
                        rollers,
                        kicker,
                        printer);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive, robotTracker);
    }

    private void configureBindings() {
        mainController.leftTrigger.whileTrue(rollerCommands.intake());
        mainController.leftTrigger.onFalse(rollerCommands.kill());
        mainController.leftBumper.whileTrue(wristCommands.deploy());
        mainController.leftBumper.onFalse(wristCommands.retract());

        mainController.rightTrigger.whileTrue(superstructure.shoot());
        mainController.rightTrigger.whileTrue(autoDrive.enableAutoDrive());
        mainController.rightTrigger.onFalse(autoDrive.disableAutoDrive());
        mainController.rightBumper.whileTrue(indexerCommands.reverse());

        // sysid
        // mainController.dPadUp.onTrue(swerve.driveRoutineDynamic());
        // mainController.dPadDown.onTrue(swerve.driveRoutineQuasiStatic());
        // mainController.dPadLeft.onTrue(swerve.steerRoutineDynamic());
        // mainController.dPadRight.onTrue(swerve.steerRoutineQuasiStatic());
        // mainController.buttonY.whileTrue(shooterRight.getSysIdDynamic());
        // mainController.buttonA.whileTrue(shooterRight.getSysIdQuasiStatic());
        // mainController.buttonX.whileTrue(shooterRight.getSysIdDynamicReverse());
        // mainController.buttonB.whileTrue(shooterRight.getSysIdQuasistaticReverse());

        mainController.buttonA.whileTrue(shooterCommands.tunableVelocity());
    }

    private void configureSmartDashboardLogging() {
        // Logger.recordOutput("right cam transform", VisionConstants.rightMount);
        // Logger.recordOutput("left cam transform", VisionConstants.leftMount);

        SmartDashboard.putData(
                "Autos", autoChooser); // , superstructure.score();, superstructure.score();
        SmartDashboard.putData("Field", smartDashboardField);
        printer.addDouble("Match Time", Timer::getMatchTime);
        printer.addDouble("left shooter velocity", shooterLeft::getVelocity);
        printer.addDouble("right shooter velocity", shooterRight::getVelocity);
        printer.addDouble("wantedhoodangle", autoDrive::getHoodAngle);
        printer.addDouble("distance to hub", targetingUtil::getCompensatedDistance);
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
        indexer.setDefaultCommand(superstructure.indexerCommands.kill());
        shooterLeft.setDefaultCommand(shooterCommands.killLeft());
        shooterRight.setDefaultCommand(shooterCommands.killRight());
        hood.setDefaultCommand(hoodCommands.setAngleDeg(autoDrive::getHoodAngle));
        // hood.setDefaultCommand(hoodCommands.setAngleDeg(hoodAngle::get));
        rollers.setDefaultCommand(rollerCommands.kill());
        wrist.setDefaultCommand(wristCommands.holdPositionAtCall());
        kicker.setDefaultCommand(kickerCommands.kill());
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

    public final ShooterLeft shooterLeft =
            new ShooterLeft(
                    ShooterConstants.kShooterOne,
                    ShooterConstants.kShooterOneSlave,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final ShooterRight shooterRight =
            new ShooterRight(
                    ShooterConstants.kShooterTwo,
                    ShooterConstants.kShooterTwoSlave,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Indexer indexer =
            new Indexer(
                    IndexerConstants.kLeftMaster,
                    IndexerConstants.kRightSlave,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Kicker kicker =
            new Kicker(
                    KickerConstants.kMaster,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Rollers rollers =
            new Rollers(
                    RollerConstants.kMaster,
                    0,
                    1,
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

    public final Wrist wrist =
            new Wrist(
                    WristConstants.kMaster,
                    WristConstants.kMaxAngle,
                    WristConstants.kMinAngle,
                    WristConstants.kNativeToDeg,
                    WristConstants.kNativeToDeg,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    // NeuralDetectorLimelight detector = new NeuralDetectorLimelight(VisionConstants.kCrossbarCam);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);

    public final ShooterCommands shooterCommands = new ShooterCommands(shooterLeft, shooterRight);

    public final IndexerCommands indexerCommands = new IndexerCommands(indexer);

    public final KickerCommands kickerCommands = new KickerCommands(kicker);

    public final RollerCommands rollerCommands = new RollerCommands(rollers);

    public final WristCommands wristCommands = new WristCommands(wrist);

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
                    swerve::getChassisSpeeds,
                    AutoConstants.autoAlignXController,
                    AutoConstants.autoAlignYController,
                    AutoConstants.rotAlignController,
                    // detector,
                    targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(
                    shooterLeft,
                    shooterRight,
                    indexer,
                    hood,
                    kicker,
                    autoDrive::getHoodAngle,
                    autoDrive::getShotSpeed);

    // private final LEDTriggers triggers = new LEDTriggers(superstructure);
    // private final LEDs leds = new LEDs(LEDConstants.m_candle, triggers);

    // cams (need to change transforms)
    //     AprilTagLimelight rightCam =
    //             new AprilTagLimelight(
    //                     VisionConstants.kRightCam,
    //                     VisionConstants.rightMount,
    //                     swerve::getPigeonOrientation,
    //                     VisionConstants.baseStdDevs,
    //                     autoDrive::getAutoDriveEnabled);

    //     AprilTagLimelight leftCam =
    //             new AprilTagLimelight(
    //                     VisionConstants.kLeftCam,
    //                     VisionConstants.leftMount,
    //                     swerve::getPigeonOrientation,
    //                     VisionConstants.baseStdDevs,
    //                     autoDrive::getAutoDriveEnabled);

    //     public final VisionController controller =
    //             new VisionController(
    //                     swerve::addVisionData,
    //                     swerve::shouldAddData,
    //                     swerve::resetPose,
    //                     rightCam,
    //                     leftCam);

    public final AutoCommands autoCommands =
            new AutoCommands(
                    swerve,
                    superstructure,
                    autoDrive::getVelocities,
                    // detector::hasTarget,
                    autoDrive);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);
}
