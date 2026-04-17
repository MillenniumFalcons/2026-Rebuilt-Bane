// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2026.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.RollerCommands;
import team3647.frc2026.commands.ShooterCommands;
import team3647.frc2026.commands.SwerveDriveCommands;
import team3647.frc2026.commands.WristCommands;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.constants.GlobalConstants;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.IndexerConstants;
import team3647.frc2026.constants.KickerConstants;
import team3647.frc2026.constants.LEDConstants;
import team3647.frc2026.constants.RollerConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.SwerveDriveConstants;
import team3647.frc2026.constants.TunerConstants;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Hood;
import team3647.frc2026.subsystems.Indexer;
import team3647.frc2026.subsystems.Kicker;
import team3647.frc2026.subsystems.LEDs;
import team3647.frc2026.subsystems.Rollers;
import team3647.frc2026.subsystems.Shooter;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.subsystems.Swerve.SwerveDriveReal;
import team3647.frc2026.subsystems.Swerve.SwerveDriveSim;
import team3647.frc2026.subsystems.Wrist;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.frc2026.util.LEDStates;
import team3647.frc2026.util.RobotTracker;
import team3647.frc2026.util.TargetingUtil;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.VisionFieldPoseEstimate;
import team3647.lib.vision.VisionIOHardwareLimelight;
import team3647.lib.vision.VisionState;
import team3647.lib.vision.VisionSubsystem;

public class RobotContainer {
    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public Field2d smartDashboardField = new Field2d();

    public DoubleSupplier idleSpeed;

    public RobotContainer() {
        shooter.setInMidSupplier(autoDrive::isInMid);
        idleSpeed = shooter::getIdleSpeed;

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
                        shooter,
                        indexer,
                        hood,
                        wrist,
                        rollers,
                        kicker,
                        // climb,
                        printer,
                        visionSubsystem);
    }

    private void configurePathPlanner() {
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config from GUI, using fallback",
                    e.getStackTrace());
            double halfWB = SwerveDriveConstants.kWheelBase / 2.0;
            double halfTW = SwerveDriveConstants.kTrackWidth / 2.0;
            ppConfig =
                    new RobotConfig(
                            52, // mass kg
                            6.55, // MOI
                            new ModuleConfig(
                                    0.049, // wheel radius
                                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                                    1.2, // wheel COF
                                    DCMotor.getKrakenX60(1).withReduction(5.4),
                                    60.0, // current limit
                                    1),
                            new Translation2d(halfWB, halfTW),
                            new Translation2d(halfWB, -halfTW),
                            new Translation2d(-halfWB, halfTW),
                            new Translation2d(-halfWB, -halfTW));
        }

        PathFollowingController ppController =
                new PPHolonomicDriveController(
                        new com.pathplanner.lib.config.PIDConstants(4.5, 0, 0.0),
                        new com.pathplanner.lib.config.PIDConstants(2.75, 0, 0.0));

        AutoBuilder.configure(
                swerve::getOdoPose,
                swerve::resetPose,
                swerve::getChassisSpeeds,
                (speeds, feedforwards) -> {
                    //     System.out.println(
                    //             "PP OUTPUT: vx="
                    //                     + speeds.vxMetersPerSecond
                    //                     + " vy="
                    //                     + speeds.vyMetersPerSecond
                    //                     + " omega="
                    //                     + speeds.omegaRadiansPerSecond);
                    swerve.drive(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                },
                ppController,
                ppConfig,
                () ->
                        DriverStation.getAlliance()
                                .map(a -> a == DriverStation.Alliance.Red)
                                .orElse(false),
                swerve);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive, robotTracker, leds);
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
        shooter.setDefaultCommand(shooterCommands.setVelocity(idleSpeed));
        // shooter.setDefaultCommand(shooterCommands.kill());
        hood.setDefaultCommand(hoodCommands.setAngleDeg(autoDrive::getHoodAngle));
        // hood.setDefaultCommand(hoodCommands.setAngleDeg(hoodAngle::get));
        rollers.setDefaultCommand(rollerCommands.kill());
        kicker.setDefaultCommand(kickerCommands.kill());
        // climb.setDefaultCommand(climbCommands.kill());
    }

    private void configureBindings() {
        // interp
        mainController.buttonX.whileTrue(superstructure.interpShot());
        // intake
        mainController.leftBumper.whileTrue(superstructure.intake());
        mainController.leftBumper.onFalse(rollerCommands.kill());
        mainController.leftTrigger.onTrue(superstructure.jerkOff());
        mainController.buttonB.onTrue(superstructure.retractIntake());
        // shooting / feeding (pose-based: past trench = feed, otherwise = shoot)
        mainController.rightTrigger.whileTrue(
                Commands.run(() -> autoDrive.updateDriveModeByPose())
                        .alongWith(superstructure.shoot()));
        mainController.rightTrigger.onTrue(autoDrive.enableAutoDrive());
        mainController.rightTrigger.onFalse(autoDrive.disableAutoDrive());
        // Manual feed
        mainController.buttonY.whileTrue(
                Commands.run(() -> autoDrive.updateDriveModeByPose())
                        .alongWith(superstructure.shoot()));
        // feed out of intake
        mainController.rightBumper.whileTrue(autoDrive.setDriveMode(DriveMode.INTAKE_FEED));
        mainController.rightBumper.whileTrue(superstructure.feed());
        mainController.rightBumper.onTrue(autoDrive.enableAutoDrive());
        mainController.rightBumper.onFalse(autoDrive.disableAutoDrive());
        // intake autoalign facing other wall — auto-disables once rotation aligned
        mainController.dPadLeft.onTrue(autoDrive.setDriveMode(DriveMode.INTAKE));
        mainController.dPadLeft.whileTrue(superstructure.intake());
        mainController.dPadLeft.onTrue(
                autoDrive
                        .enableAutoDrive()
                        .andThen(Commands.waitUntil(autoDrive::rotAligned))
                        .andThen(autoDrive.disableAutoDrive()));
        mainController.dPadLeft.onFalse(autoDrive.disableAutoDrive());
        // pre align + spin up (no indexing)
        // Gate on !rightTrigger so that when rightTrigger is released while dPadRight is still
        // held, prep() gets re-scheduled (whileTrue only fires on rising edges of the trigger).
        mainController
                .dPadRight
                .and(mainController.rightTrigger.negate())
                .whileTrue(superstructure.prep());
        // cocontroller
        coController.rightTrigger.whileTrue(superstructure.prep());
        coController.buttonB.onTrue(autoDrive.disableAutoDrive());
        coController.leftTrigger.onTrue(superstructure.jerkOff());
        coController.dPadUp.onTrue(Commands.runOnce(() -> shooter.enableIdle()));
        coController.dPadDown.onTrue(Commands.runOnce(() -> shooter.disableIdle()));
        coController.buttonX.onTrue(superstructure.retractIntake());
        coController.rightBumper.whileTrue(superstructure.reverse());
        // defense mode
        coController.buttonA.onTrue(Commands.runOnce(() -> swerveCommands.setRotMultiplier(0.9)));
        coController.buttonA.onFalse(Commands.runOnce(() -> swerveCommands.setRotMultiplier(0.6)));

        // sysid
        // mainController.dPadUp.whileTrue(swerve.driveRoutineDynamic());
        // mainController.dPadDown.whileTrue(swerve.driveRoutineQuasiStatic());
        // mainController.dPadLeft.whileTrue(swerve.driveRoutineDynamicReverse());
        // mainController.dPadRight.whileTrue(swerve.driveRoutineQuasiStaticReverse());
        // mainController.buttonY.onTrue(shooter.getSysIdDynamic());
        // mainController.buttonA.onTrue(shooter.getSysIdQuasiStatic());
        // mainController.buttonX.onTrue(shooter.getSysIdDynamicReverse());
        // mainController.buttonB.onTrue(shooter.getSysIdQuasistaticReverse());
    }

    public void updateRobotPoseForSmartdashboard() {
        smartDashboardField.setRobotPose(swerve.getOdoPose());
    }

    /** Feed the latest odometry and yaw velocity into VisionState each cycle. */
    public void updateVisionState() {
        double timestamp = Timer.getFPGATimestamp();
        visionState.addOdometryMeasurement(timestamp, swerve.getOdoPose());
        visionState.addDriveYawVelocity(timestamp, swerve.getChassisSpeeds().omegaRadiansPerSecond);
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

    public final Shooter shooter =
            new Shooter(
                    ShooterConstants.kMaster,
                    ShooterConstants.kSlave,
                    ShooterConstants.kSlave1,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Indexer indexer =
            new Indexer(
                    IndexerConstants.kMaster,
                    0,
                    1,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Kicker kicker =
            new Kicker(
                    KickerConstants.kMaster,
                    KickerConstants.kSlave2,
                    KickerConstants.kSlave3,
                    KickerConstants.kNativeVelToSurfaceMpS,
                    KickerConstants.kNativeVelToSurfaceMpS,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Rollers rollers =
            new Rollers(
                    RollerConstants.kMaster,
                    RollerConstants.kSlave,
                    RollerConstants.kNativeToSurfaceRPS,
                    RollerConstants.kNativeToSurfaceRPS,
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

    //     public final Climb climb =
    //             new Climb(
    //                     ClimbConstants.kMaster,
    //                     0,
    //                     1,
    //                     GlobalConstants.kNominalVoltage,
    //                     GlobalConstants.kDt);
    //     NeuralDetectorLimelight detector = new
    // NeuralDetectorLimelight(VisionConstants.kShooterCam);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);

    public final ShooterCommands shooterCommands = new ShooterCommands(shooter);

    public final IndexerCommands indexerCommands = new IndexerCommands(indexer);

    public final KickerCommands kickerCommands = new KickerCommands(kicker);

    public final RollerCommands rollerCommands = new RollerCommands(rollers);

    public final WristCommands wristCommands = new WristCommands(wrist);

    public final HoodCommands hoodCommands = new HoodCommands(hood);

    //     public final ClimbCommands climbCommands = new ClimbCommands(climb);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    private final GroupPrinter printer = GroupPrinter.getInstance();

    public final RobotTracker robotTracker =
            new RobotTracker(swerve::getOdoPose, swerve::getChassisSpeeds);

    public final TargetingUtil targetingUtil = new TargetingUtil(robotTracker);

    public final AutoDrive autoDrive =
            new AutoDrive(swerve::getOdoPose, swerve::getChassisSpeeds, targetingUtil);

    public LEDStates ledStates = new LEDStates();

    public final LEDs leds = new LEDs(LEDConstants.m_candle, ledStates);

    public final Superstructure superstructure =
            new Superstructure(
                    shooter,
                    indexer,
                    hood,
                    kicker,
                    wrist,
                    rollers,
                    ledStates,
                    robotTracker,
                    autoDrive::getHoodAngle,
                    autoDrive::getShotSpeed,
                    autoDrive::rotAligned);

    // 254-style vision system
    public final VisionState visionState =
            new VisionState(
                    (VisionFieldPoseEstimate est) -> {
                        var dynamicStdDevs = est.getVisionMeasurementStdDevs();
                        dynamicStdDevs.set(0, 0, 0.05);
                        dynamicStdDevs.set(1, 0, 0.05);
                        swerve.addVisionMeasurement(
                                est.getVisionRobotPoseMeters(),
                                Utils.fpgaToCurrentTime(est.getTimestampSeconds()),
                                dynamicStdDevs);
                    });

    public final VisionIOHardwareLimelight visionIO = new VisionIOHardwareLimelight(visionState);

    public final VisionSubsystem visionSubsystem = new VisionSubsystem(visionIO, visionState);

    {
        // NamedCommands.registerCommand("intake", superstructure.intakeWithDelay());
        // NamedCommands.registerCommand("shoot", superstructure.shoot());
        // NamedCommands.registerCommand(
        //         "autoalign", swerveCommands.driveButInAuto(autoDrive,
        // autoDrive::getVelocities));
        configurePathPlanner();
        NamedCommands.registerCommand(
                "intake", Commands.defer(superstructure::intakeWithDelay, Set.of(wrist, rollers)));
        NamedCommands.registerCommand(
                "shoot",
                Commands.defer(
                        superstructure::shoot, Set.of(shooter, wrist, indexer, kicker, rollers)));
        NamedCommands.registerCommand(
                "autoalign",
                Commands.defer(
                        () -> swerveCommands.driveButInAuto(autoDrive, autoDrive::getVelocities),
                        Set.of(swerve)));
    }

    public final AutoCommands autoCommands =
            new AutoCommands(
                    swerve, superstructure, autoDrive::getVelocities, () -> false, autoDrive);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);

    /**
     * Returns true when our alliance's goal is in an active period (can score). Uses FMS game data
     * to determine auto winner, then checks shift timing. Always returns true during transition
     * (>130s) and endgame (<30s).
     */
    public final BooleanSupplier isActivePeriod =
            () -> {
                double matchTime = Timer.getMatchTime();

                // Transition period or endgame — always active
                if (matchTime > 130 || matchTime <= 30) return true;

                // Determine which shift we're in (odd = 1,3; even = 2,4)
                boolean isOddShift;
                if (matchTime > 105) isOddShift = true; // Shift 1: 105-130
                else if (matchTime > 80) isOddShift = false; // Shift 2: 80-105
                else if (matchTime > 55) isOddShift = true; // Shift 3: 55-80
                else isOddShift = false; // Shift 4: 30-55

                // Game data: "R" or "B" = alliance that LOST auto
                // Won auto → active in odd shifts (1,3)
                // Lost auto → active in even shifts (2,4)
                String gameData = DriverStation.getGameSpecificMessage();
                if (gameData == null || gameData.isEmpty()) return true;

                DriverStation.Alliance ourAlliance =
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                boolean wonAuto;
                if (ourAlliance == DriverStation.Alliance.Red) {
                    wonAuto = gameData.equals("B"); // Blue lost → Red won
                } else {
                    wonAuto = gameData.equals("R"); // Red lost → Blue won
                }

                return wonAuto ? isOddShift : !isOddShift;
            };
}
