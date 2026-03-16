// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2026.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.commands.ClimbCommands;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.RollerCommands;
import team3647.frc2026.commands.ShooterCommands;
import team3647.frc2026.commands.SwerveDriveCommands;
import team3647.frc2026.commands.WristCommands;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.constants.ClimbConstants;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.GlobalConstants;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.IndexerConstants;
import team3647.frc2026.constants.KickerConstants;
import team3647.frc2026.constants.RollerConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.SwerveDriveConstants;
import team3647.frc2026.constants.TunerConstants;
import team3647.frc2026.constants.VisionConstants;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Climb;
import team3647.frc2026.subsystems.Hood;
import team3647.frc2026.subsystems.Indexer;
import team3647.frc2026.subsystems.Kicker;
import team3647.frc2026.subsystems.Rollers;
import team3647.frc2026.subsystems.ShooterLeft;
import team3647.frc2026.subsystems.ShooterRight;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.subsystems.Swerve.SwerveDriveReal;
import team3647.frc2026.subsystems.Swerve.SwerveDriveSim;
import team3647.frc2026.subsystems.Wrist;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.frc2026.util.RobotTracker;
import team3647.frc2026.util.TargetingUtil;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.tunables.TunableNumber;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.VisionController;

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
        hood.setEncoderAngleCANcoder();

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
                        climb,
                        printer);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive, robotTracker);
    }

    private void configureSmartDashboardLogging() {
        SmartDashboard.putData("Autos", autoChooser);
        SmartDashboard.putData("Field", smartDashboardField);
        printer.addDouble("Match Time", Timer::getMatchTime);
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
        wrist.setDefaultCommand(wristCommands.retract());
        kicker.setDefaultCommand(kickerCommands.kill());
        climb.setDefaultCommand(climbCommands.kill());
    }

    private void configureBindings() {
        // intake
        mainController.leftBumper.whileTrue(superstructure.intake());
        mainController.leftBumper.onFalse(superstructure.retractIntake());
        mainController.leftTrigger.whileTrue(superstructure.intakeFast());
        mainController.leftTrigger.onFalse(superstructure.retractIntake());

        // shooting
        mainController.rightTrigger.whileTrue(superstructure.shoot());
        mainController.buttonB.whileTrue(superstructure.bumpShot());
        mainController.rightTrigger.whileTrue(autoDrive.enableAutoDrive());
        mainController.rightTrigger.onFalse(autoDrive.disableAutoDrive());
        mainController.rightBumper.whileTrue(superstructure.reverse());
        mainController.buttonA.onTrue(autoDrive.setDriveMode(DriveMode.SHOOT_ON_THE_MOVE));
        mainController.buttonY.onTrue(autoDrive.setDriveMode(DriveMode.FEED));

        mainController.dPadRight.onTrue(autoDrive.setDriveMode(DriveMode.SHOOT));
        mainController.dPadRight.whileTrue(autoDrive.enableAutoDrive());
        mainController.dPadRight.whileTrue(superstructure.prep());
        mainController.dPadRight.onFalse(autoDrive.disableAutoDrive());
        mainController.dPadRight.onFalse(autoDrive.setDriveMode(DriveMode.SHOOT_ON_THE_MOVE));

        // cocontroller
        coController.rightTrigger.whileTrue(superstructure.prep());
        coController.buttonB.onTrue(autoDrive.disableAutoDrive());
        coController.leftBumper.whileTrue(superstructure.bumpKicker());
        // coController.buttonA.onTrue(hood.setEncoderAngleCANcoder());

        // mainController.leftTrigger.whileTrue(superstructure.shotTuning());
        // mainController.leftTrigger.whileTrue(autoDrive.enableAutoDrive());
        // mainController.leftTrigger.onFalse(autoDrive.disableAutoDrive());

        // sysid
        // mainController.dPadUp.whileTrue(swerve.driveRoutineDynamic());
        // mainController.dPadDown.whileTrue(swerve.driveRoutineQuasiStatic());
        // mainController.dPadLeft.whileTrue(swerve.driveRoutineDynamicReverse());
        // mainController.dPadRight.whileTrue(swerve.driveRoutineQuasiStaticReverse());
        // mainController.buttonY.whileTrue(shooterLeft.getSysIdDynamic());
        // mainController.buttonA.whileTrue(shooterLeft.getSysIdQuasiStatic());
        // mainController.buttonX.whileTrue(shooterLeft.getSysIdDynamicReverse());
        // mainController.buttonB.whileTrue(shooterLeft.getSysIdQuasistaticReverse());
    }

    public void updateRobotPoseForSmartdashboard() {
        smartDashboardField.setRobotPose(swerve.getOdoPose());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getAutoCommand();
    }

    @SuppressWarnings("unchecked")
    public final SwerveDrive swerve =
            Utils.isSimulation()
                    ? new SwerveDriveSim(
                            SwerveDriveConstants.driveTrainSimulationConfig, GlobalConstants.kDt)
                    : new SwerveDriveReal(
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
                    RollerConstants.kNativeToSurfaceRPS,
                    RollerConstants.kNativeToSurfaceRPS,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Hood hood =
            new Hood(
                    HoodConstants.kMaster,
                    HoodConstants.kCanCoder,
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

    public final Climb climb =
            new Climb(
                    ClimbConstants.kMaster,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);
    //     NeuralDetectorLimelight detector = new
    // NeuralDetectorLimelight(VisionConstants.kShooterCam);

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

    public final ClimbCommands climbCommands = new ClimbCommands(climb);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    private final GroupPrinter printer = GroupPrinter.getInstance();

    public final RobotTracker robotTracker =
            new RobotTracker(
                    FieldConstants.hubPose,
                    FieldConstants.feedPoseRight,
                    FieldConstants.feedPoseLeft,
                    ShooterConstants.robotToBetweenShooter,
                    swerve::getOdoPose,
                    swerve::getChassisSpeeds);

    public final TargetingUtil targetingUtil = new TargetingUtil(robotTracker);

    public final AutoDrive autoDrive =
            new AutoDrive(swerve::getOdoPose, swerve::getChassisSpeeds, targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(
                    shooterLeft,
                    shooterRight,
                    indexer,
                    hood,
                    kicker,
                    wrist,
                    rollers,
                    robotTracker,
                    autoDrive::getHoodAngle,
                    autoDrive::getShotSpeed,
                    autoDrive::rotAligned);

    // private final LEDTriggers triggers = new LEDTriggers(superstructure);
    // private final LEDs leds = new LEDs(LEDConstants.m_candle, triggers);

    // cams (need to change transforms)
    AprilTagLimelight rightCam =
            new AprilTagLimelight(
                    VisionConstants.kRightCam,
                    VisionConstants.rightMount,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs,
                    autoDrive::getAutoDriveEnabled);

    AprilTagLimelight leftCam =
            new AprilTagLimelight(
                    VisionConstants.kLeftCam,
                    VisionConstants.leftMount,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs,
                    autoDrive::getAutoDriveEnabled);

    AprilTagLimelight midCam =
            new AprilTagLimelight(
                    VisionConstants.kShooterCam,
                    VisionConstants.midMount,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs,
                    autoDrive::getAutoDriveEnabled);

    public final VisionController controller =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::resetPose,
                    rightCam,
                    leftCam,
                    midCam);

    public final AutoCommands autoCommands =
            new AutoCommands(
                    swerve, superstructure, autoDrive::getVelocities, () -> false, autoDrive);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);
}
