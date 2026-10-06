package team3647.frc2026.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.io.IOException;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.autos.AutoCommands.ChoreoController;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AllianceFlip;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.PoseUtils;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.lib.team9442.AllianceObserver;

public class AutoPathFollowers implements AllianceObserver {
    private final PIDController rotController = new PIDController(5.0, 0, 0.1);

    final SwerveDrive swerve;
    final Superstructure superstructure;
    final Supplier<Twist2d> autoDriveVelocities;
    public final BooleanSupplier hasTarget;
    public final AutoDrive autoDrive;

    Alliance color;
    boolean isRed = true;

    public AutoPathFollowers(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.autoDriveVelocities = autoDriveVelocities;
        this.autoDrive = autoDrive;
        this.hasTarget = hasTarget;
    }

    public Command autoAlign() {
        return Commands.sequence(
                        Commands.runOnce(() -> swerve.drive(0, 0, 0)),
                        autoDrive.enableAutoDrive(),
                        autoDrive.setDriveMode(DriveMode.SHOOT),
                        Commands.run(
                                () ->
                                        swerve.driveFieldOriented(
                                                0, 0, autoDriveVelocities.get().dtheta),
                                swerve))
                .finallyDo(() -> autoDrive.setEnabled(false));
    }

    // regular pathfollower
    public Command followChoreoPath(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        AutoConstants.autoXController,
                        AutoConstants.autoYController,
                        rotController),
                (speeds) -> {
                    swerve.drive(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                },
                swerve::getOdoPose);
    }

    // pathfollower with autoalign overrides
    public Command followChoreoPathWithAutoAlignOverride(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                        traj,
                        choreoSwerveController(
                                AutoConstants.autoXController,
                                AutoConstants.autoYController,
                                AutoConstants.rotController),
                        (ChassisSpeeds speeds) -> {
                            var motionRotComponent = speeds.omegaRadiansPerSecond;
                            var motionXComponent = speeds.vxMetersPerSecond * 0.5;
                            var motionYComponent = speeds.vyMetersPerSecond * 0.5;

                            if (this.hasTarget.getAsBoolean()) {
                                motionXComponent = autoDriveVelocities.get().dx;
                                motionYComponent = autoDriveVelocities.get().dy;
                            }

                            swerve.drive(motionXComponent, motionYComponent, motionRotComponent);
                        },
                        swerve::getOdoPose)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    // HELPERS

    public Command customPathFollower(
            Trajectory<SwerveSample> traj,
            ChoreoController controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Supplier<Pose2d> poseSupplier) {

        Timer timer = new Timer();

        return new FunctionalCommand(
                timer::restart,
                () -> {
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    traj.sampleAt(timer.get(), isRed())
                                            .orElse(AutoConstants.kEmptySample)));
                },
                (interrupted) -> {
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                    timer.stop();
                },
                () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.1),
                swerve);
    }

    public Command customPathFollowerNi(
            Trajectory<SwerveSample> traj,
            ChoreoController controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Supplier<Pose2d> poseSupplier) {

        Timer timer = new Timer();

        return new FunctionalCommand(
                timer::restart,
                () -> {
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    traj.sampleAt(timer.get(), isRed())
                                            .orElse(AutoConstants.kEmptySample)));
                },
                (interrupted) -> {
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                    timer.stop();
                },
                () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.3),
                swerve);
    }

    public static ChoreoController choreoSwerveController(
            PIDController xController,
            PIDController yController,
            PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.getChassisSpeeds().vxMetersPerSecond;
            double yFF = referenceState.getChassisSpeeds().vyMetersPerSecond;
            double rotationFF = referenceState.getChassisSpeeds().omegaRadiansPerSecond;
            double xFeedback = xController.calculate(pose.getX(), referenceState.getPose().getX());
            double yFeedback = yController.calculate(pose.getY(), referenceState.getPose().getY());
            double rotationFeedback =
                    rotationController.calculate(
                            pose.getRotation().getRadians(),
                            referenceState.getPose().getRotation().getRadians());

            Logger.recordOutput("AutoController/xFF", xFF);
            Logger.recordOutput("AutoController/referenceX", referenceState.getPose().getX());
            Logger.recordOutput("AutoController/yFF", yFF);
            Logger.recordOutput("AutoController/referenceY", referenceState.getPose().getY());
            Logger.recordOutput("Auto/ReferencePose", referenceState.getPose());

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    rotationFF + rotationFeedback,
                    pose.getRotation());
        };
    }

    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTraj(String path) {
        var trajMaybe = Choreo.loadTrajectory(path);

        if (trajMaybe.isPresent()) {
            return (Trajectory<SwerveSample>) trajMaybe.get();
        } else {
            return AutoConstants.kEmptyTraj;
        }
    }

    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTrajSplit(String path, int splitIndex) {
        var trajMaybe = Choreo.loadTrajectory(path);

        if (trajMaybe.isPresent()) {
            var splitMaybe = trajMaybe.get().getSplit(splitIndex);

            if (splitMaybe.isPresent()) {
                return (Trajectory<SwerveSample>) splitMaybe.get();
            }
        }

        return AutoConstants.kEmptyTraj;
    }

    public Pose2d getInitial(Trajectory<SwerveSample> traj, Alliance color) {
        return traj.getInitialPose(color == Alliance.Red).orElse(new Pose2d());
    }

    public Supplier<Pose2d> getInitialPPLazy(String pathName) {
        return () -> getInitialPP(pathName, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public Supplier<Pose2d> getInitialPPMirroredLazy(String pathName) {
        return () -> getInitialPPMirrored(pathName, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public Pose2d getInitialPP(String pathName, Alliance color) {
        Pose2d startPose;
        try {
            startPose =
                    PathPlannerAuto.getPathGroupFromAutoFile(pathName)
                            .get(0)
                            .getStartingHolonomicPose()
                            .get();
        } catch (FileVersionException | IOException | ParseException e) {
            startPose = new Pose2d();
            e.printStackTrace();
        }
        return AllianceFlip.flip(startPose, color);
    }

    public Pose2d getInitialPPMirrored(String pathName, Alliance color) {
        Pose2d startPose;
        try {
            startPose =
                    PathPlannerPath.fromPathFile(pathName)
                            .mirrorPath()
                            .getStartingHolonomicPose()
                            .get();
        } catch (Exception e) {
            startPose = new Pose2d();
            e.printStackTrace();
        }
        return AllianceFlip.flip(startPose, color);
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;

        setIsRed();
    }

    public void setIsRed() {
        this.isRed = color == Alliance.Red;
    }

    public boolean isRed() {
        return this.isRed;
    }

    // =========================================================================
    // PathPlanner path following
    // =========================================================================

    public Command followPathPlannerPath(
            String pathName, BiFunction<Pose2d, Pose2d, Boolean> finished, boolean mirror) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            if (mirror) {
                path = path.mirrorPath();
            }
            Translation2d finalTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();
            Rotation2d finalRotation = path.getGoalEndState().rotation();
            Pose2d rawFinalPose = new Pose2d(finalTranslation, finalRotation);
            Command ppCommand = AutoBuilder.followPath(path);
            Timer pathTimer = new Timer();
            BooleanSupplier finishedSupplier =
                    () ->
                            ppCommand.isFinished()
                                    && finished.apply(swerve.getRealPose(), rawFinalPose);
            return new FunctionalCommand(
                    () -> {
                        pathTimer.restart();
                        ppCommand.initialize();
                    },
                    ppCommand::execute,
                    ppCommand::end,
                    finishedSupplier,
                    swerve);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followPathPlannerPath(String pathName) {

        BiFunction<Pose2d, Pose2d, Boolean> finished =
                (finalPose, robotPose) -> {
                    return PoseUtils.aligned(swerve.getOdoPose(), finalPose, 0.1, 4);
                };
        return followPathPlannerPath(pathName, finished, false);
    }

    public Command followMirroredPathPlannerPath(String pathName) {
        BiFunction<Pose2d, Pose2d, Boolean> finished =
                (finalPose, robotPose) -> {
                    return PoseUtils.aligned(swerve.getOdoPose(), finalPose, 0.1, 4);
                };
        return followPathPlannerPath(pathName, finished, true);
    }

    /** Run a full PathPlanner auto by name (loads from deploy/pathplanner/autos/). */
    public Command followPathPlannerAuto(String autoName) {
        try {
            return AutoBuilder.buildAuto(autoName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner auto: " + autoName, e.getStackTrace());
            return Commands.none();
        }
    }

    /** Pathfind to a pose using PathPlanner's dynamic pathfinding. */
    public Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /** Pathfind to a pose with default constraints. */
    public Command pathfindToPose(Pose2d targetPose) {
        return pathfindToPose(
                targetPose,
                new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720)));
    }
}
