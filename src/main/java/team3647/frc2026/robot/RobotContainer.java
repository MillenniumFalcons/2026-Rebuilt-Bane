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
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
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
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.autos.AutoMode;
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
import team3647.frc2026.constants.VisionConstants;
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
import team3647.frc2026.util.CurrentMonitor;
import team3647.frc2026.util.MatchStateLogger;
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
        shooter.setPrepSpeedSupplier(autoDrive::getShotSpeed);
        idleSpeed = shooter::getIdleSpeed;

        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        configureSmartDashboardLogging();

        wrist.initCancoderPos();

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
                        new com.pathplanner.lib.config.PIDConstants(8.0, 0, 0.0),
                        new com.pathplanner.lib.config.PIDConstants(3.0, 0, 0.0)) {
                    @Override
                    public edu.wpi.first.math.kinematics.ChassisSpeeds calculateRobotRelativeSpeeds(
                            Pose2d currentPose, PathPlannerTrajectoryState targetState) {
                        Logger.recordOutput("Auto/ReferencePose", targetState.pose);
                        return super.calculateRobotRelativeSpeeds(currentPose, targetState);
                    }
                };

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
                        autoDrive::rotAligned,
                        () -> autoDrive.headingErrorDegrees() > 8));
        indexer.setDefaultCommand(superstructure.indexerCommands.kill());
        // shooter.setDefaultCommand(shooterCommands.kill());
        shooter.setDefaultCommand(shooterCommands.idleCoast(idleSpeed));
        hood.setDefaultCommand(hoodCommands.setAngle(HoodConstants.kMinAngle));
        // hood.setDefaultCommand(hoodCommands.setAngleDeg(autoDrive::getHoodAngle));
        rollers.setDefaultCommand(rollerCommands.kill());
        kicker.setDefaultCommand(kickerCommands.kill());
    }

    private void configureBindings() {
        // preload balls, commented out for now, need to trigger after intake
        // superstructure.notLoaded.whileTrue(superstructure.loadWhenDetected());
        // X lock: press X to toggle brake on; it cancels automatically once the driver moves a
        // stick (or by pressing X again). Requires swerve so it interrupts the default drive
        // command, and ending restores the default command.
        mainController.buttonX.toggleOnTrue(
                Commands.run(() -> swerve.brake(), swerve).until(mainController::anyStickMoved));
        mainController.buttonA.onTrue(shooterCommands.kill().alongWith(kickerCommands.reverse()));
        // interp
        // mainController.buttonX.whileTrue(superstructure.interpShot());
        // intake
        mainController.leftBumper.whileTrue(superstructure.intakeNoIndex());
        mainController
                .leftBumper
                .and(mainController.rightTrigger.negate())
                .whileTrue(superstructure.indexerCommands.spinSlow());
        mainController.leftBumper.onFalse(rollerCommands.kill());

        mainController.buttonB.onTrue(wristCommands.retract());
        mainController.buttonA.onTrue(superstructure.intakeDown());
        // shooting / feeding (pose-based: past trench = feed, otherwise = shoot)
        mainController.rightTrigger.whileTrue(
                Commands.run(() -> autoDrive.updateDriveModeByPose())
                        .alongWith(superstructure.shoot()));
        mainController.rightTrigger.onTrue(autoDrive.enableAutoDrive());
        mainController.rightTrigger.onFalse(autoDrive.disableAutoDrive());
        // same as normal shoot (dynamic targeting, pose-based feed/shoot) but no auto-align
        // (driver aims rotation manually)
        mainController.leftTrigger.whileTrue(
                Commands.run(() -> autoDrive.updateDriveModeByPose())
                        .alongWith(superstructure.shootNoAlign()));
        // Manual feed
        // mainController.buttonY.whileTrue(superstructure.shootInstantly());
        // feed out of intake

        // mainController.rightBumper.whileTrue(autoDrive.setDriveMode(DriveMode.INTAKE_FEED));
        mainController.rightBumper.whileTrue(superstructure.feed());
        // mainController.rightBumper.onTrue(autoDrive.enableAutoDrive());
        // mainController.rightBumper.onFalse(autoDrive.disableAutoDrive());

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
        // pre-spin: hold ~2s before shooting to raise the shooter idle speed to the target shot
        // speed. This only bumps the idle setpoint (consumed by the shooter default command), so
        // it never commands the shooter directly and can't interrupt an in-progress shoot().
        coController.rightTrigger.whileTrue(
                Commands.startEnd(shooter::startPrepIdle, shooter::stopPrepIdle));
        // force-start indexing on an already-running shoot(): flips an override flag that opens
        // the indexWhenReady gate early. Requires no subsystems, so it won't interrupt the
        // shooter/hood spin-up that shoot() is driving.
        coController.leftTrigger.whileTrue(superstructure.forceIndex());

        coController.buttonB.onTrue(autoDrive.disableAutoDrive());
        coController.dPadRight.onTrue(Commands.runOnce(() -> shooter.enableIdle()));
        coController.dPadLeft.onTrue(Commands.runOnce(() -> shooter.disableIdle()));
        coController.dPadUp.onTrue(
                autoDrive
                        .setDriveMode(DriveMode.MANUAL_SHOOT)
                        .alongWith(autoDrive.incrementHood(1)));
        coController.dPadDown.onTrue(
                autoDrive
                        .setDriveMode(DriveMode.MANUAL_SHOOT)
                        .alongWith(autoDrive.incrementHood(-1)));
        coController.buttonX.toggleOnTrue(
                Commands.run(() -> swerve.brake(), swerve)
                        .until(mainController::anyStickMoved)); // defense mode
        coController.buttonA.onTrue(Commands.runOnce(() -> swerveCommands.setRotMultiplier(0.9)));
        coController.buttonA.onFalse(Commands.runOnce(() -> swerveCommands.setRotMultiplier(0.6)));

        // beached mode: hold to make the pose estimator rely entirely on vision (wheels slipping)
        coController.leftBumper.onTrue(Commands.runOnce(() -> swerve.setVisionOnlyMode(true)));
        coController.leftBumper.onFalse(Commands.runOnce(() -> swerve.setVisionOnlyMode(false)));

        // snap pose estimation to the latest accepted vision pose (one-shot hard reset).
        // Skip if no vision has ever been accepted or the last accepted pose is stale (>2s old).
        coController.rightBumper.onTrue(
                Commands.runOnce(
                        () -> {
                            double lastTs = visionState.lastUsedMegatagTimestamp();
                            double age = Timer.getFPGATimestamp() - lastTs;
                            if (lastTs > 0 && age <= 2.0) {
                                swerve.resetPose(visionState.lastUsedMegatagPose());
                            }
                        }));

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
        smartDashboardField.getObject("VisionPose").setPose(visionState.lastUsedMegatagPose());
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

    // Set when autonomous ends, consumed at teleop start. Lets a shoot trigger held through the
    // auto->teleop transition start shooting immediately once teleop enables. The rightTrigger
    // whileTrue binding needs a fresh rising edge to fire, which a press held across the disabled
    // transition never produces (the button loop already saw it pressed while disabled), so without
    // this the driver has to release and re-press after the match resumes.
    private boolean shootQueuedFromAuto = false;

    /** Call from {@code autonomousExit}. */
    public void onAutonomousExit() {
        shootQueuedFromAuto = true;
    }

    /**
     * Call from {@code teleopInit}. If the driver is holding the shoot trigger coming out of auto,
     * start shooting right away. The command ends when the trigger is released, after which the
     * normal rightTrigger binding (and its onFalse disableAutoDrive) takes over.
     */
    public void startQueuedTeleopShoot() {
        if (shootQueuedFromAuto && mainController.rightTrigger.getAsBoolean()) {
            autoDrive.enableAutoDrive().schedule();
            Commands.run(() -> autoDrive.updateDriveModeByPose())
                    .alongWith(superstructure.shoot())
                    .until(mainController.rightTrigger.negate())
                    .schedule();
        }
        shootQueuedFromAuto = false;
    }

    /**
     * Re-assert the selected auto's starting pose and briefly suppress vision at auto start. The
     * pose is otherwise only set on auto-selection/alliance-found in Disabled, so a
     * vision-corrupted odometry pose could carry into Autonomous and stall the path follower. Call
     * from {@code autonomousInit}.
     */
    public void resetPoseForAuto() {
        AutoMode selected = autoChooser.getSelected();
        if (selected != null) {
            swerve.setRobotPose(selected.getStartingPose());
        }
        visionSubsystem.suppressVisionForSeconds(VisionConstants.kAutoStartVisionSuppressSeconds);
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
                    WristConstants.cancoder,
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
                    swerve, 4.0, SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);

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

    public final CurrentMonitor currentMonitor =
            new CurrentMonitor(swerve, List.of(shooter, indexer, kicker, rollers, hood, wrist));

    public final Superstructure superstructure =
            new Superstructure(
                    shooter,
                    indexer,
                    hood,
                    kicker,
                    wrist,
                    rollers,
                    robotTracker,
                    autoDrive::getHoodAngle,
                    autoDrive::getShotSpeed,
                    autoDrive::rotAligned,
                    () -> autoDrive.getWantedMode() == DriveMode.FEED,
                    autoDrive::getAutoDriveEnabled);

    public final LEDs leds =
            new LEDs(LEDConstants.m_candle, superstructure, mainController, swerve::isBraked);

    // 254-style vision system
    public final VisionState visionState =
            new VisionState(
                    (VisionFieldPoseEstimate est) -> {
                        // Use the pipeline's distance/quality-scaled std devs, floored so a single
                        // optimistic frame can't near-absolutely trust vision. Previously these
                        // were pinned at 0.05 m, which let a noisy pose snap odometry and stall
                        // the auto path follower.
                        var dynamicStdDevs = est.getVisionMeasurementStdDevs();
                        dynamicStdDevs.set(
                                0,
                                0,
                                Math.max(
                                        dynamicStdDevs.get(0, 0),
                                        VisionConstants.kMinVisionXYStdDev));
                        dynamicStdDevs.set(
                                1,
                                0,
                                Math.max(
                                        dynamicStdDevs.get(1, 0),
                                        VisionConstants.kMinVisionXYStdDev));
                        swerve.addVisionMeasurement(
                                est.getVisionRobotPoseMeters(),
                                Utils.fpgaToCurrentTime(est.getTimestampSeconds()),
                                dynamicStdDevs);
                        robotTracker.recordVisionAccepted();
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

    public final MatchStateLogger matchStateLogger = new MatchStateLogger();
}
